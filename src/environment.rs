use macroquad::prelude::*;

pub const MAP_WIDTH: f32 = 10000.0;
pub const MAP_HEIGHT: f32 = 10000.0;

pub struct Obstacle {
    pub rect: Rect,
}

pub struct Environment {
    pub obstacles: Vec<Obstacle>,
}

impl Environment {
    pub fn new(seed: u32) -> Self {
        let mut obstacles = Vec::with_capacity(1004);
        
        obstacles.push(Obstacle { rect: Rect::new(-100.0, -100.0, MAP_WIDTH + 200.0, 100.0) });
        obstacles.push(Obstacle { rect: Rect::new(-100.0, MAP_HEIGHT, MAP_WIDTH + 200.0, 100.0) });
        obstacles.push(Obstacle { rect: Rect::new(-100.0, -100.0, 100.0, MAP_HEIGHT + 200.0) });
        obstacles.push(Obstacle { rect: Rect::new(MAP_WIDTH, -100.0, 100.0, MAP_HEIGHT + 200.0) });

        let mut n = seed;
        let mut rand_f32 = || -> f32 {
            n = n.wrapping_mul(1664525).wrapping_add(1013904223);
            (n as f32) / (u32::MAX as f32)
        };

        for _ in 0..1000 {
            let width = 50.0 + rand_f32() * 200.0;
            let height = 50.0 + rand_f32() * 200.0;
            let x = rand_f32() * (MAP_WIDTH - width);
            let y = rand_f32() * (MAP_HEIGHT - height);
            
            let rect = Rect::new(x, y, width, height);
            let center = vec2(MAP_WIDTH / 2.0, MAP_HEIGHT / 2.0);
            let center_rect = Rect::new(center.x - 200.0, center.y - 200.0, 400.0, 400.0);
            
            if !rect.overlaps(&center_rect) {
                obstacles.push(Obstacle { rect });
            }
        }

        Self { obstacles }
    }

    pub fn draw(&self) {
        let grid_size = 100.0;
        let x_lines = (MAP_WIDTH / grid_size) as i32;
        let y_lines = (MAP_HEIGHT / grid_size) as i32;
        
        for i in 0..=x_lines {
            let x = i as f32 * grid_size;
            draw_line(x, 0.0, x, MAP_HEIGHT, 1.0, Color::new(0.2, 0.2, 0.2, 1.0));
        }
        for i in 0..=y_lines {
            let y = i as f32 * grid_size;
            draw_line(0.0, y, MAP_WIDTH, y, 1.0, Color::new(0.2, 0.2, 0.2, 1.0));
        }

        for obs in &self.obstacles {
            draw_rectangle(obs.rect.x, obs.rect.y, obs.rect.w, obs.rect.h, GRAY);
            draw_rectangle_lines(obs.rect.x, obs.rect.y, obs.rect.w, obs.rect.h, 2.0, DARKGRAY);
        }
    }
}
