mod environment;
mod quadtree;
mod drone;

use macroquad::prelude::*;
use environment::Environment;
use drone::Drone;

fn window_conf() -> Conf {
    Conf {
        window_title: "Drone Mesh Networking Simulator - Phase 1".to_owned(),
        window_width: 1024,
        window_height: 768,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let env = Environment::new();
    let mut drone = Drone::new(environment::MAP_WIDTH / 2.0, environment::MAP_HEIGHT / 2.0);

    let mut is_map_open = false;
    let mut is_person_controlled = true;
    let mut current_tick: u64 = 0;

    let scan_radius = drone.scan_radius;

    loop {
        let dt = get_frame_time();

        if is_key_pressed(KeyCode::O) {
            is_map_open = !is_map_open;
        }
        if is_key_pressed(KeyCode::P) {
            is_person_controlled = !is_person_controlled;
        }

        drone.update(is_person_controlled, &env, dt, current_tick);
        current_tick += 1;

        if current_tick % 60 == 0 {
            let diff = drone.map.diff_since(0);
            if let Ok(bytes) = bincode::serialize(&diff) {
                println!("Tick {}: Full map state size: {} bytes", current_tick, bytes.len());
            }
        }

        clear_background(Color::new(0.1, 0.1, 0.1, 1.0));

        let camera = Camera2D {
            target: drone.position,
            zoom: vec2(1.0 / screen_width() * 2.0, -1.0 / screen_height() * 2.0),
            ..Default::default()
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
            drone.map.draw(camera_bounds);
            
            let cx = drone.position.x;
            let cy = drone.position.y;
            let r = scan_radius;
            draw_circle_lines(cx, cy, r, 2.0, Color::new(1.0, 1.0, 1.0, 0.3));
        }

        drone.draw();

        set_default_camera();

        let map_status = if is_map_open { "OPEN" } else { "CLOSED" };
        let ctrl_status = if is_person_controlled { "PERSON" } else { "IDLE" };

        let ui_text_1 = format!("O: Toggle Map (Current: {})", map_status);
        let ui_text_2 = format!("P: Toggle Control (Current: {})", ctrl_status);
        
        let text_size = 20.0;
        let padding = 10.0;
        let text_x = screen_width() - 350.0;
        
        draw_rectangle(text_x - padding, padding, 350.0, 60.0, Color::new(0.0, 0.0, 0.0, 0.7));

        draw_text(&ui_text_1, text_x, padding + text_size, text_size, WHITE);
        draw_text(&ui_text_2, text_x, padding + text_size * 2.5, text_size, WHITE);

        if is_person_controlled {
            draw_text("Controls: W A S D to move and turn", 10.0, 20.0, 20.0, LIGHTGRAY);
        }

        next_frame().await
    }
}
