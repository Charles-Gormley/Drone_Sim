mod environment;
mod quadtree;
mod autonomy;
mod drone;
mod network;

// Drone Mesh Networking Simulator
// Multi-agent autonomous exploration with Noise Protocol encrypted mesh networking.
// Each drone runs as an independent tokio task, discovers peers via UDP heartbeat,
// and gossips encrypted map diffs to synchronize exploration state.

use macroquad::prelude::*;
use environment::Environment;
use drone::Drone;
use std::sync::Arc;
use tokio::sync::mpsc;
use std::collections::HashMap;

/// Number of drones to spawn. Also controls the camera cycle range and UDP port range.
const DRONE_COUNT: u32 = 3;

pub enum RenderEvent {
    StateUpdate(RenderState),
    MapUpdate(Vec<u8>),
}

#[derive(Clone)]
pub struct RenderState {
    pub id: u32,
    pub position: Vec2,
    pub heading: f32,
    pub path: Option<Vec<Vec2>>,
    pub frontiers: Vec<Vec2>,
}

fn window_conf() -> Conf {
    Conf {
        window_title: "Drone Mesh Networking Simulator - Phase 4".to_owned(),
        window_width: 1024,
        window_height: 768,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let env = Arc::new(Environment::new());
    
    let (render_tx, mut render_rx) = mpsc::unbounded_channel();
    
    let env_clone = env.clone();
    std::thread::spawn(move || {
        let rt = tokio::runtime::Runtime::new()
            .expect("Failed to create tokio runtime: OS refused thread pool resources");
        rt.block_on(async {
            for id in 1..=DRONE_COUNT {
                let drone = Drone::new(id, environment::MAP_WIDTH / 2.0 + (id as f32 * 50.0), environment::MAP_HEIGHT / 2.0);
                tokio::spawn(drone.run_drone_task(env_clone.clone(), render_tx.clone()));
            }
            
            loop {
                tokio::time::sleep(std::time::Duration::from_secs(1)).await;
            }
        });
    });

    let mut is_map_open = false;
    let mut is_zoomed_out = false;
    let mut is_path_visible = true;
    let mut drone_states: HashMap<u32, RenderState> = HashMap::new();
    let mut master_map = quadtree::Quadtree::new(Rect::new(0.0, 0.0, environment::MAP_WIDTH, environment::MAP_HEIGHT), 9);
    
    let mut camera_target_id = 1;

    loop {
        if is_key_pressed(KeyCode::O) {
            is_map_open = !is_map_open;
        }
        if is_key_pressed(KeyCode::Z) {
            is_zoomed_out = !is_zoomed_out;
        }
        if is_key_pressed(KeyCode::V) {
            is_path_visible = !is_path_visible;
        }
        if is_key_pressed(KeyCode::C) {
            camera_target_id += 1;
            if camera_target_id > DRONE_COUNT {
                camera_target_id = 1;
            }
        }

        while let Ok(event) = render_rx.try_recv() {
            match event {
                RenderEvent::StateUpdate(state) => {
                    drone_states.insert(state.id, state);
                }
                RenderEvent::MapUpdate(bytes) => {
                    if let Ok(diff) = bincode::deserialize(&bytes) {
                        master_map.merge(diff);
                    }
                }
            }
        }

        clear_background(Color::new(0.1, 0.1, 0.1, 1.0));

        let camera = if is_zoomed_out {
            Camera2D {
                target: vec2(environment::MAP_WIDTH / 2.0, environment::MAP_HEIGHT / 2.0),
                zoom: vec2(1.0 / environment::MAP_WIDTH * 2.0, -1.0 / environment::MAP_HEIGHT * 2.0),
                ..Default::default()
            }
        } else {
            let camera_target_pos = drone_states.get(&camera_target_id).map(|s| s.position).unwrap_or(vec2(environment::MAP_WIDTH / 2.0, environment::MAP_HEIGHT / 2.0));
            Camera2D {
                target: camera_target_pos,
                zoom: vec2(1.0 / screen_width() * 2.0, -1.0 / screen_height() * 2.0),
                ..Default::default()
            }
        };

        set_camera(&camera);

        let camera_bounds = Rect::new(
            camera.target.x - screen_width() / camera.zoom.x.abs() / 2.0,
            camera.target.y - screen_height() / camera.zoom.y.abs() / 2.0,
            screen_width() / camera.zoom.x.abs(),
            screen_height() / camera.zoom.y.abs()
        );

        if is_map_open {
            env.draw();
        } else {
            master_map.draw(camera_bounds);
            
            for state in drone_states.values() {
                draw_circle_lines(state.position.x, state.position.y, 300.0, 2.0, Color::new(1.0, 1.0, 1.0, 0.3));
            }
        }

        for state in drone_states.values() {
            if is_path_visible {
                for &f in &state.frontiers {
                    draw_rectangle_lines(f.x - 50.0, f.y - 50.0, 100.0, 100.0, 2.0, YELLOW);
                }

                if let Some(ref path) = state.path {
                    if !path.is_empty() {
                        let mut prev = state.position;
                        for &p in path {
                            draw_line(prev.x, prev.y, p.x, p.y, 3.0, RED);
                            prev = p;
                        }
                    }
                }
            }

            let p1 = state.position + vec2(state.heading.cos(), state.heading.sin()) * 15.0;
            let p2 = state.position + vec2((state.heading + std::f32::consts::PI * 0.75).cos(), (state.heading + std::f32::consts::PI * 0.75).sin()) * 10.0;
            let p3 = state.position + vec2((state.heading - std::f32::consts::PI * 0.75).cos(), (state.heading - std::f32::consts::PI * 0.75).sin()) * 10.0;
            draw_triangle(p1, p2, p3, RED);
        }

        set_default_camera();

        let map_status = if is_map_open { "OPEN" } else { "CLOSED" };
        let zoom_status = if is_zoomed_out { "ALL" } else { "LOCAL" };
        let path_status = if is_path_visible { "SHOW" } else { "HIDE" };
        
        let ui_text_1 = format!("O: Toggle Map (Current: {})", map_status);
        let ui_text_2 = format!("C: Cycle Camera (Tracking: Drone {})", camera_target_id);
        let ui_text_3 = format!("Z: Toggle Zoom (Current: {})", zoom_status);
        let ui_text_4 = format!("V: Toggle Pathing (Current: {})", path_status);
        
        let text_size = 20.0;
        let padding = 10.0;
        let text_x = screen_width() - 400.0;
        
        draw_rectangle(text_x - padding, padding, 400.0, 110.0, Color::new(0.0, 0.0, 0.0, 0.7));
        draw_text(&ui_text_1, text_x, padding + text_size, text_size, WHITE);
        draw_text(&ui_text_2, text_x, padding + text_size * 2.5, text_size, WHITE);
        draw_text(&ui_text_3, text_x, padding + text_size * 4.0, text_size, WHITE);
        draw_text(&ui_text_4, text_x, padding + text_size * 5.5, text_size, WHITE);

        next_frame().await
    }
}
