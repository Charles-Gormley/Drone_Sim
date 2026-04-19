use macroquad::prelude::*;
use crate::environment::Environment;

const MOVE_SPEED: f32 = 500.0;
const TURN_SPEED: f32 = 3.0;
const FRICTION: f32 = 0.90;

pub struct Drone {
    pub(crate) position: Vec2,
    pub(crate) velocity: Vec2,
    pub(crate) heading: f32,
    pub(crate) scan_radius: f32,
    pub(crate) radius: f32,
}

impl Drone {
    pub fn new(x: f32, y: f32) -> Self {
        Self {
            position: vec2(x, y),
            velocity: vec2(0.0, 0.0),
            heading: 0.0,
            scan_radius: 300.0,
            radius: 10.0,
        }
    }

    pub fn update(&mut self, is_controlled: bool, env: &Environment, dt: f32) {
        let mut acceleration = vec2(0.0, 0.0);

        if is_controlled {
            if is_key_down(KeyCode::A) {
                self.heading += TURN_SPEED * dt;
            }
            if is_key_down(KeyCode::D) {
                self.heading -= TURN_SPEED * dt;
            }

            let heading_vec = vec2(self.heading.cos(), self.heading.sin());
            if is_key_down(KeyCode::W) {
                acceleration += heading_vec * MOVE_SPEED;
            }
            if is_key_down(KeyCode::S) {
                acceleration -= heading_vec * MOVE_SPEED;
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
            self.velocity = vec2(0.0, 0.0);
        } else {
            self.position = next_pos;
        }
    }

    pub fn draw(&self) {
        let p1 = self.position + vec2(self.heading.cos(), self.heading.sin()) * 15.0;
        let p2 = self.position + vec2((self.heading + std::f32::consts::PI * 0.75).cos(), (self.heading + std::f32::consts::PI * 0.75).sin()) * 10.0;
        let p3 = self.position + vec2((self.heading - std::f32::consts::PI * 0.75).cos(), (self.heading - std::f32::consts::PI * 0.75).sin()) * 10.0;

        draw_triangle(p1, p2, p3, RED);
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
