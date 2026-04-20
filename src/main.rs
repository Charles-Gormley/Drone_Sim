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
        window_title: "Drone Mesh Networking Simulator - Phase 6".to_owned(),
        window_width: 1024,
        window_height: 768,
        ..Default::default()
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let is_headless = args.contains(&"--headless".to_string());
    
    let mut seed = 12345u32;
    if let Some(pos) = args.iter().position(|a| a == "--seed") {
        if pos + 1 < args.len() {
            if let Ok(s) = args[pos + 1].parse::<u32>() {
                seed = s;
            }
        }
    }

    let mut base_port = 8000u32;
    if let Some(pos) = args.iter().position(|a| a == "--port") {
        if pos + 1 < args.len() {
            if let Ok(p) = args[pos + 1].parse::<u32>() {
                base_port = p;
            }
        }
    }

    if is_headless {
        let rt = tokio::runtime::Runtime::new().unwrap();
        rt.block_on(async {
            run_headless_simulation(seed, base_port).await;
        });
    } else {
        macroquad::Window::from_config(window_conf(), ui_main(seed, base_port));
    }
}

async fn run_headless_simulation(seed: u32, base_port: u32) {
    let env = Arc::new(Environment::new(seed));
    let (render_tx, mut render_rx) = mpsc::unbounded_channel();
    
    for id in 1..=DRONE_COUNT {
        let drone = Drone::new(id, environment::MAP_WIDTH / 2.0 + (id as f32 * 50.0), environment::MAP_HEIGHT / 2.0);
        tokio::spawn(drone.run_drone_task(env.clone(), render_tx.clone(), true, base_port));
    }

    let mut master_map = quadtree::Quadtree::new(Rect::new(0.0, 0.0, environment::MAP_WIDTH, environment::MAP_HEIGHT), 9);
    let start_time = std::time::Instant::now();
    let total_area = environment::MAP_WIDTH * environment::MAP_HEIGHT;
    
    println!("--- HEADLESS SIMULATION STARTED (Seed: {}) ---", seed);

    loop {
        if let Some(event) = render_rx.recv().await {
            match event {
                RenderEvent::StateUpdate(_) => {},
                RenderEvent::MapUpdate(bytes) => {
                    if let Ok(diff) = bincode::deserialize(&bytes) {
                        master_map.merge(diff);
                        let explored = master_map.explored_area();
                        let percent = (explored / total_area) * 100.0;
                        
                        if percent >= 95.0 {
                            let duration = start_time.elapsed();
                            println!("\nSUCCESS: 95% Exploration reached!");
                            println!("COVERAGE: {:.2}%", percent);
                            println!("TIME: {:.2}s", duration.as_secs_f32());
                            std::process::exit(0);
                        }
                    }
                }
            }
        }
        
        // Periodic progress update for orchestrator to see
        if start_time.elapsed().as_secs() % 10 == 0 {
             let explored = master_map.explored_area();
             let percent = (explored / total_area) * 100.0;
             println!("PROGRESS: {:.2}%", percent);
        }
    }
}

async fn ui_main(seed: u32, base_port: u32) {
    let env = Arc::new(Environment::new(seed));
    
    let (render_tx, mut render_rx) = mpsc::unbounded_channel();
    
    let env_clone = env.clone();
    std::thread::spawn(move || {
        let rt = tokio::runtime::Runtime::new()
            .expect("Failed to create tokio runtime: OS refused thread pool resources");
        rt.block_on(async {
            for id in 1..=DRONE_COUNT {
                let drone = Drone::new(id, environment::MAP_WIDTH / 2.0 + (id as f32 * 50.0), environment::MAP_HEIGHT / 2.0);
                tokio::spawn(drone.run_drone_task(env_clone.clone(), render_tx.clone(), false, base_port));
            }
            
            loop {
                tokio::time::sleep(std::time::Duration::from_secs(1)).await;
            }
        });
    });

    let mut is_map_open = false;
    let mut is_zoomed_out = false;
    let mut is_path_visible = true;
    let mut is_heatmap_visible = false;
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
        if is_key_pressed(KeyCode::H) {
            is_heatmap_visible = !is_heatmap_visible;
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
            
            if is_heatmap_visible {
                let main_tick = (get_time() * 60.0) as u64;
                master_map.draw_heatmap(camera_bounds, main_tick);
            }
            
            for state in drone_states.values() {
                draw_circle_lines(state.position.x, state.position.y, 300.0, 2.0, Color::new(1.0, 1.0, 1.0, 0.3));
            }
            
            // Draw mesh topology lines
            for (id1, d1) in &drone_states {
                for (id2, d2) in &drone_states {
                    if id1 < id2 {
                        let dist = d1.position.distance(d2.position);
                        
                        if dist <= 800.0 {
                            draw_line(d1.position.x, d1.position.y, d2.position.x, d2.position.y, 2.0, GREEN);
                        } else if dist <= 2000.0 {
                            draw_line(d1.position.x, d1.position.y, d2.position.x, d2.position.y, 2.0, YELLOW);
                        }
                    }
                }
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
        let heatmap_status = if is_heatmap_visible { "ON" } else { "OFF" };
        
        let ui_text_1 = format!("O: Toggle Map (Current: {})", map_status);
        let ui_text_2 = format!("C: Cycle Camera (Tracking: Drone {})", camera_target_id);
        let ui_text_3 = format!("Z: Toggle Zoom (Current: {})", zoom_status);
        let ui_text_4 = format!("V: Toggle Pathing (Current: {})", path_status);
        let ui_text_5 = format!("H: Toggle Heatmap (Current: {})", heatmap_status);
        
        let text_size = 20.0;
        let padding = 10.0;
        let text_x = screen_width() - 400.0;
        
        draw_rectangle(text_x - padding, padding, 400.0, 135.0, Color::new(0.0, 0.0, 0.0, 0.7));
        draw_text(&ui_text_1, text_x, padding + text_size, text_size, WHITE);
        draw_text(&ui_text_2, text_x, padding + text_size * 2.5, text_size, WHITE);
        draw_text(&ui_text_3, text_x, padding + text_size * 4.0, text_size, WHITE);
        draw_text(&ui_text_4, text_x, padding + text_size * 5.5, text_size, WHITE);
        draw_text(&ui_text_5, text_x, padding + text_size * 7.0, text_size, WHITE);

        next_frame().await
    }
}
