use macroquad::prelude::*;
use crate::environment::Environment;
use crate::quadtree::Quadtree;
use crate::autonomy::{AutonomyState, update_autonomy};

const MOVE_SPEED: f32 = 500.0;
const TURN_SPEED: f32 = 3.0;
const FRICTION: f32 = 0.90;
/// Base UDP port. Drone N listens on BASE_PORT + N.
const BASE_PORT: u32 = 8000;
/// Maximum number of concurrent drone tasks (port scan range).
const MAX_DRONES: u32 = 10;
/// Ticks before a silent peer is culled (10s at 16ms/tick).
const DEAD_PEER_TICKS: u64 = 625;

pub struct Drone {
    pub id: u32,
    pub(crate) position: Vec2,
    pub(crate) velocity: Vec2,
    pub(crate) heading: f32,
    pub(crate) scan_radius: f32,
    pub(crate) radius: f32,
    pub map: Quadtree,
    pub autonomy: AutonomyState,
    pub(crate) last_position: Vec2,
    pub(crate) stuck_timer: f32,
    /// Internal routing table. Mutated only within drone.rs.
    pub(crate) peer_table: std::collections::HashMap<u32, crate::network::Peer>,
    /// Cached peer positions for autonomy — refreshed each physics tick.
    cached_peer_positions: Vec<(f32, f32)>,
    /// Cached peer paths for autonomy — refreshed each physics tick.
    cached_peer_paths: Vec<Option<Vec<(f32, f32)>>>,
}

impl Drone {
    pub fn new(id: u32, x: f32, y: f32) -> Self {
        Self {
            id,
            position: vec2(x, y),
            velocity: vec2(0.0, 0.0),
            heading: 0.0,
            scan_radius: 300.0,
            radius: 10.0,
            map: Quadtree::new(Rect::new(0.0, 0.0, crate::environment::MAP_WIDTH, crate::environment::MAP_HEIGHT), 9),
            autonomy: AutonomyState::new(),
            last_position: vec2(x, y),
            stuck_timer: 0.0,
            peer_table: std::collections::HashMap::with_capacity(MAX_DRONES as usize),
            cached_peer_positions: Vec::with_capacity(MAX_DRONES as usize),
            cached_peer_paths: Vec::with_capacity(MAX_DRONES as usize),
        }
    }

    pub async fn run_drone_task(
        mut self,
        env: std::sync::Arc<Environment>,
        render_tx: tokio::sync::mpsc::UnboundedSender<crate::RenderEvent>,
    ) {
        let mut ticker = tokio::time::interval(std::time::Duration::from_millis(16));

        let port = BASE_PORT + self.id;
        let socket = match tokio::net::UdpSocket::bind(format!("127.0.0.1:{}", port)).await {
            Ok(s) => s,
            Err(e) => {
                eprintln!("Drone {}: failed to bind UDP port {} — {}", self.id, port, e);
                return;
            }
        };
        if let Err(e) = socket.set_broadcast(true) {
            eprintln!("Drone {}: failed to set broadcast — {}", self.id, e);
            return;
        }

        let mut current_tick: u64 = 0;
        let mut heartbeat_ticker = tokio::time::interval(std::time::Duration::from_secs(2));
        let mut map_gossip_ticker = tokio::time::interval(std::time::Duration::from_secs(5));
        let mut last_gossip_tick: u64 = 0;

        let (private_key, _public_key) = match crate::network::generate_keypair() {
            Ok(kp) => kp,
            Err(e) => {
                eprintln!("Drone {}: failed to generate Noise keypair — {}", self.id, e);
                return;
            }
        };
        let mut sessions = crate::network::NoiseSessionTable::new(private_key);

        let mut buf = [0u8; 65536];

        loop {
            tokio::select! {
                _ = ticker.tick() => {
                    let dt = 0.016;
                    
                    self.update_physics(false, &env, dt, current_tick);
                    
                    let diff = self.map.diff_since(current_tick.saturating_sub(1));
                    if !diff.updates.is_empty() {
                        if let Ok(bytes) = bincode::serialize(&diff) {
                            let _ = render_tx.send(crate::RenderEvent::MapUpdate(bytes));
                        }
                    }

                    current_tick += 1;

                    // Refresh cached peer data before autonomy uses it this tick
                    self.cached_peer_positions.clear();
                    self.cached_peer_paths.clear();
                    for peer in self.peer_table.values() {
                        self.cached_peer_positions.push(peer.position);
                        self.cached_peer_paths.push(peer.path.clone());
                    }

                    // Cull peers that have gone silent for more than DEAD_PEER_TICKS
                    self.peer_table.retain(|_id, peer| {
                        current_tick.saturating_sub(peer.last_seen) < DEAD_PEER_TICKS
                    });

                    let state = crate::RenderState {
                        id: self.id,
                        position: self.position,
                        heading: self.heading,
                        path: self.autonomy.path.clone(),
                        frontiers: self.autonomy.all_frontiers.clone(),
                    };
                    
                    if render_tx.send(crate::RenderEvent::StateUpdate(state)).is_err() {
                        break;
                    }
                }
                _ = heartbeat_ticker.tick() => {
                    // Heartbeats are sent PLAINTEXT (unencrypted) — they are used for
                    // peer discovery, not secret data. This is intentional by design.
                    let path_tuples = self.autonomy.path.as_ref().map(|p| {
                        p.iter().map(|v| (v.x, v.y)).collect::<Vec<_>>()
                    });

                    let msg = crate::network::Message {
                        msg_type: crate::network::MsgType::Heartbeat,
                        sender_id: self.id,
                        sender_pos: (self.position.x, self.position.y),
                        path: path_tuples,
                        peer_list: self.peer_table.keys().copied().collect(),
                        payload: vec![],
                        timestamp: current_tick,
                    };
                    
                    // Broadcast heartbeat to known port range
                    if let Ok(msg_bytes) = msg.to_bytes() {
                    for scan_port in BASE_PORT..(BASE_PORT + MAX_DRONES) {
                        if scan_port != BASE_PORT + self.id {
                            let _ = socket.send_to(&msg_bytes, format!("127.0.0.1:{}", scan_port)).await;
                        }
                    }
                    } // end if let Ok(msg_bytes)

                    // Initiate Noise handshake to any HIGHER-id peer without a session.
                    // Lower-id peer acts as responder — prevents both sides racing to initiate.
                    // Lower-ID peer will act as responder. This prevents both sides racing.
                    let new_peer_ids: Vec<u32> = self.peer_table.keys()
                        .copied()
                        .filter(|&pid| pid > self.id) // Only initiate to higher-ID peers
                        .filter(|&pid| !sessions.has_session(pid))
                        .filter(|&pid| !sessions.handshakes.contains_key(&pid))
                        .collect();

                    for peer_id in new_peer_ids {
                        match sessions.initiate_handshake(peer_id) {
                            Ok(hs_msg) => {
                                let env = crate::network::Envelope {
                                    tag: crate::network::HANDSHAKE_TAG,
                                    sender_id: self.id,
                                    payload: hs_msg,
                                };
                                if let Ok(env_bytes) = env.to_bytes() {
                                    let _ = socket.send_to(&env_bytes, format!("127.0.0.1:{}", BASE_PORT + peer_id)).await;
                                    println!("Drone {} → Drone {}: initiated Noise XX handshake", self.id, peer_id);
                                }
                            }
                            Err(e) => eprintln!("Drone {}: failed to initiate handshake with {}: {}", self.id, peer_id, e),
                        }
                    }
                }
                _ = map_gossip_ticker.tick() => {
                    // MAP_DIFF is sent ENCRYPTED to peers with established sessions.
                    let diff = self.map.diff_since(last_gossip_tick);
                    last_gossip_tick = current_tick;

                    if diff.updates.is_empty() { continue; }

                    if let Ok(plaintext) = bincode::serialize(&diff) {
                        let msg = crate::network::Message {
                            msg_type: crate::network::MsgType::MapDiff,
                            sender_id: self.id,
                            sender_pos: (self.position.x, self.position.y),
                            path: None,
                            peer_list: vec![],
                            payload: plaintext,
                            timestamp: current_tick,
                        };
                        if let Ok(msg_bytes) = msg.to_bytes() {
                        for peer in self.peer_table.values() {
                            if sessions.has_session(peer.id) {
                                match sessions.encrypt(peer.id, &msg_bytes) {
                                    Ok(ciphertext) => {
                                        let env = crate::network::Envelope {
                                            tag: crate::network::TRANSPORT_TAG,
                                            sender_id: self.id,
                                            payload: ciphertext,
                                        };
                                        if let Ok(env_bytes) = env.to_bytes() {
                                            let _ = socket.send_to(&env_bytes, format!("127.0.0.1:{}", BASE_PORT + peer.id)).await;
                                            println!("Drone {} → Drone {}: sent encrypted MAP_DIFF ({} updates)", self.id, peer.id, diff.updates.len());
                                        }
                                    }
                                    Err(e) => eprintln!("Drone {}: encrypt error for peer {}: {}", self.id, peer.id, e),
                                }
                            } else {
                                // Plaintext fallback — session not yet established
                                let _ = socket.send_to(&msg_bytes, format!("127.0.0.1:{}", BASE_PORT + peer.id)).await;
                                println!("Drone {} → Drone {}: MAP_DIFF plaintext (session pending)", self.id, peer.id);
                            }
                        }
                        } // end if let Ok(msg_bytes)
                    }
                }
                result = socket.recv_from(&mut buf) => {
                    if let Ok((len, _addr)) = result {
                        // Try to parse as an Envelope first (handshake or encrypted transport)
                        if let Some(env) = crate::network::Envelope::from_bytes(&buf[..len]) {
                            let sender_id = env.sender_id;

                            if env.tag == crate::network::HANDSHAKE_TAG {
                                match sessions.process_handshake_msg(sender_id, &env.payload) {
                                    Ok(Some(reply)) => {
                                        let reply_env = crate::network::Envelope {
                                            tag: crate::network::HANDSHAKE_TAG,
                                            sender_id: self.id,
                                            payload: reply,
                                        };
                                        if let Ok(reply_bytes) = reply_env.to_bytes() {
                                            let _ = socket.send_to(&reply_bytes, format!("127.0.0.1:{}", BASE_PORT + sender_id)).await;
                                        }
                                    }
                                    Ok(None) => { /* handshake complete, no reply needed */ }
                                    Err(e) => eprintln!("Drone {}: handshake error from peer {}: {}", self.id, sender_id, e),
                                }

                            } else if env.tag == crate::network::TRANSPORT_TAG {
                                // Decrypt and process the payload
                                match sessions.decrypt(sender_id, &env.payload) {
                                    Ok(plaintext) => {
                                        if let Some(msg) = crate::network::Message::from_bytes(&plaintext) {
                                            Self::process_message(self.id, &mut self.peer_table, &mut self.map, msg, current_tick);
                                        }
                                    }
                                    Err(e) => eprintln!("Drone {}: decrypt error from peer {}: {}", self.id, sender_id, e),
                                }
                            } else {
                                eprintln!("Drone {}: unknown envelope tag 0x{:02x} from {}", self.id, env.tag, sender_id);
                            }
                        } else if let Some(msg) = crate::network::Message::from_bytes(&buf[..len]) {
                            // Plaintext message (heartbeats, pre-session fallback)
                            Self::process_message(self.id, &mut self.peer_table, &mut self.map, msg, current_tick);
                        }
                    }
                }
            }
        }
    }

    /// Processes a decoded (plaintext) Message — shared between the plaintext and decrypted paths.
    fn process_message(
        my_id: u32,
        peer_table: &mut std::collections::HashMap<u32, crate::network::Peer>,
        map: &mut Quadtree,
        msg: crate::network::Message,
        current_tick: u64,
    ) {
        match msg.msg_type {
            crate::network::MsgType::Hello | crate::network::MsgType::Heartbeat => {
                // Ignore messages from ourselves (loopback on port-range broadcast)
                if msg.sender_id == my_id { return; }
                let peer = peer_table.entry(msg.sender_id).or_insert(crate::network::Peer {
                    id: msg.sender_id,
                    last_seen: current_tick,
                    position: msg.sender_pos,
                    path: None,
                });
                peer.last_seen = current_tick;
                peer.position = msg.sender_pos;
                peer.path = msg.path;

                // Transitive discovery: exclude self
                for &peer_id in &msg.peer_list {
                    if peer_id != msg.sender_id && peer_id != my_id && !peer_table.contains_key(&peer_id) {
                        peer_table.insert(peer_id, crate::network::Peer {
                            id: peer_id,
                            last_seen: current_tick.saturating_sub(300),
                            position: (0.0, 0.0),
                            path: None,
                        });
                    }
                }
            }
            crate::network::MsgType::MapDiff => {
                if let Ok(diff) = bincode::deserialize::<crate::quadtree::MapDiff>(&msg.payload) {
                    let update_count = diff.updates.len();
                    map.merge(diff);
                    println!("Merged MAP_DIFF from Drone {} ({} updates)", msg.sender_id, update_count);
                }
            }
        }
    }

    fn update_physics(&mut self, is_controlled: bool, env: &Environment, dt: f32, current_tick: u64) {
        let mut acceleration = vec2(0.0, 0.0);

        if is_controlled {
            if is_key_down(KeyCode::W) {
                acceleration += vec2(self.heading.cos(), self.heading.sin()) * MOVE_SPEED;
            }
            if is_key_down(KeyCode::S) {
                acceleration -= vec2(self.heading.cos(), self.heading.sin()) * MOVE_SPEED;
            }

            if is_key_down(KeyCode::A) {
                self.heading -= TURN_SPEED * dt;
            }
            if is_key_down(KeyCode::D) {
                self.heading += TURN_SPEED * dt;
            }
        } else {
            // Use pre-cached peer data (refreshed at start of this tick, not here)
            update_autonomy(&mut self.autonomy, self.position, &self.map, dt, &self.cached_peer_positions, &self.cached_peer_paths);

            if let Some(ref mut path) = self.autonomy.path {
                if let Some(target_pos) = path.first() {
                    let to_target = *target_pos - self.position;
                    let dist = to_target.length();

                    // If close enough, remove waypoint
                    if dist < 15.0 {
                        path.remove(0);
                    } else {
                        // "Seek" behavior with Braking
                        let dir = if self.velocity.length_squared() > 1.0 {
                            self.velocity.normalize()
                        } else {
                            vec2(self.heading.cos(), self.heading.sin())
                        };
                        let target_dir = to_target.normalize();
                        
                        let dot = dir.dot(target_dir);
                        
                        // Brake if we need to make a sharp turn
                        let desired_speed = if dot < 0.0 {
                            MOVE_SPEED * 0.2
                        } else if dot < 0.8 {
                            MOVE_SPEED * 0.5
                        } else {
                            MOVE_SPEED
                        };

                        let desired_velocity = target_dir * desired_speed;
                        let steering = desired_velocity - self.velocity;
                        
                        // Limit steering force
                        let max_force = MOVE_SPEED;
                        let mut steering_force = steering * 2.0;
                        if steering_force.length() > max_force {
                            steering_force = steering_force.normalize() * max_force;
                        }
                        
                        acceleration += steering_force;

                        // Rotate drone visually towards the target
                        self.heading = to_target.y.atan2(to_target.x);
                    }
                }
            }

            // Fail-Safe Stuck Detection
            if self.autonomy.path.is_some() {
                if self.stuck_timer == 0.0 {
                    self.last_position = self.position; // Anchor position
                }
                
                self.stuck_timer += dt;

                if self.position.distance(self.last_position) > 25.0 {
                    self.stuck_timer = 0.0; // Drone made progress, reset timer
                }

                if self.stuck_timer > 1.5 {
                    println!("FAIL-SAFE TRIGGERED: Drone {} is physically stuck at {:?}. Executing recovery maneuver...", self.id, self.position);
                    self.stuck_timer = 0.0;
                    
                    // Reverse velocity and push away hard
                    let push_dir = vec2(-self.heading.cos(), -self.heading.sin());
                    self.velocity = push_dir * MOVE_SPEED;
                    self.position += self.velocity * dt * 20.0; // Immediate bump out of the wall
                    
                    self.autonomy.ban_current_target();
                }
            }
        }

        self.velocity += acceleration * dt;
        self.velocity *= FRICTION;

        let next_pos = self.position + self.velocity * dt;

        let mut collision = false;
        for obs in &env.obstacles {
            if Self::circle_rect_intersect(next_pos, self.radius, obs.rect) {
                collision = true;
                break;
            }
        }

        if collision {
            // Anti-Stick Bounce
            self.velocity = self.velocity * -0.5;
            self.position += self.velocity * dt;
            // Removed: clearing path on collision so the stuck timer can actually do its job over 1.5s
        } else {
            self.position = next_pos;
        }

        self.map.insert_scan(self.position, self.scan_radius, env, current_tick);
    }

    fn circle_rect_intersect(circle_center: Vec2, circle_radius: f32, rect: Rect) -> bool {
        let closest_x = circle_center.x.clamp(rect.x, rect.x + rect.w);
        let closest_y = circle_center.y.clamp(rect.y, rect.y + rect.h);

        let distance_x = circle_center.x - closest_x;
        let distance_y = circle_center.y - closest_y;

        let distance_squared = (distance_x * distance_x) + (distance_y * distance_y);
        distance_squared < (circle_radius * circle_radius)
    }
}
