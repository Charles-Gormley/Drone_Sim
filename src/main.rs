mod environment;
mod drone;

use macroquad::prelude::*;
use environment::Environment;
use drone::Drone;

const FOG_OVERDRAW_MARGIN: f32 = 20000.0;

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

    // Generate fog of war mask texture
    let scan_radius = drone.scan_radius;
    let tex_size = (scan_radius * 2.0) as u32;
    let mut mask_img = Image::gen_image_color(tex_size as u16, tex_size as u16, BLACK);
    let r_sq = scan_radius * scan_radius;
    
    for y in 0..tex_size {
        for x in 0..tex_size {
            let dx = (x as f32) - scan_radius;
            let dy = (y as f32) - scan_radius;
            if dx * dx + dy * dy < r_sq {
                mask_img.set_pixel(x, y, Color::new(0.0, 0.0, 0.0, 0.0)); // Transparent hole
            }
        }
    }
    let mask_texture = Texture2D::from_image(&mask_img);

    loop {
        let dt = get_frame_time();

        if is_key_pressed(KeyCode::O) {
            is_map_open = !is_map_open;
        }
        if is_key_pressed(KeyCode::P) {
            is_person_controlled = !is_person_controlled;
        }

        drone.update(is_person_controlled, &env, dt);

        clear_background(Color::new(0.1, 0.1, 0.1, 1.0));

        let camera = Camera2D {
            target: drone.position,
            zoom: vec2(1.0 / screen_width() * 2.0, -1.0 / screen_height() * 2.0),
            ..Default::default()
        };

        if is_map_open {
        }

        set_camera(&camera);

        env.draw();
        drone.draw();

        if !is_map_open {
            let cx = drone.position.x;
            let cy = drone.position.y;
            let r = scan_radius;
            
            draw_rectangle(cx - FOG_OVERDRAW_MARGIN, cy - FOG_OVERDRAW_MARGIN, FOG_OVERDRAW_MARGIN * 2.0, FOG_OVERDRAW_MARGIN - r, BLACK);
            draw_rectangle(cx - FOG_OVERDRAW_MARGIN, cy + r, FOG_OVERDRAW_MARGIN * 2.0, FOG_OVERDRAW_MARGIN - r, BLACK);
            draw_rectangle(cx - FOG_OVERDRAW_MARGIN, cy - r, FOG_OVERDRAW_MARGIN - r, r * 2.0, BLACK);
            draw_rectangle(cx + r, cy - r, FOG_OVERDRAW_MARGIN - r, r * 2.0, BLACK);

            draw_texture(&mask_texture, cx - r, cy - r, WHITE);
            
            draw_circle_lines(cx, cy, r, 2.0, GRAY);
        }

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
