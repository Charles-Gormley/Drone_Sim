use macroquad::prelude::*;
use serde::{Serialize, Deserialize};
use crate::environment::Environment;

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CellState {
    pub(crate) explored: bool,
    pub(crate) obstacle: bool,
    pub(crate) last_seen: u64,
}

impl Default for CellState {
    fn default() -> Self {
        Self {
            explored: false,
            obstacle: false,
            last_seen: 0,
        }
    }
}

#[derive(Clone, Serialize, Deserialize)]
pub enum QuadNode {
    Leaf(CellState),
    Internal(Box<[QuadNode; 4]>),
}

#[derive(Serialize, Deserialize)]
pub struct MapDiff {
    pub(crate) timestamp: u64,
    pub(crate) updates: Vec<((f32, f32, f32, f32), CellState)>,
}

pub struct Quadtree {
    pub(crate) root: QuadNode,
    pub(crate) bounds: Rect,
    pub(crate) max_depth: u8,
}

impl Quadtree {
    pub fn new(bounds: Rect, max_depth: u8) -> Self {
        Self {
            root: QuadNode::Leaf(CellState::default()),
            bounds,
            max_depth,
        }
    }

    pub fn insert_scan(&mut self, center: Vec2, radius: f32, env: &Environment, tick: u64) {
        let scan_bounds = Rect::new(center.x - radius, center.y - radius, radius * 2.0, radius * 2.0);
        self.root = Self::insert_node(std::mem::replace(&mut self.root, QuadNode::Leaf(CellState::default())), self.bounds, scan_bounds, center, radius, env, tick, 0, self.max_depth);
    }

    fn insert_node(
        node: QuadNode,
        node_bounds: Rect,
        scan_bounds: Rect,
        center: Vec2,
        radius: f32,
        env: &Environment,
        tick: u64,
        depth: u8,
        max_depth: u8,
    ) -> QuadNode {
        if !node_bounds.overlaps(&scan_bounds) {
            return node;
        }

        if depth >= max_depth {
            let cell_center = vec2(node_bounds.x + node_bounds.w / 2.0, node_bounds.y + node_bounds.h / 2.0);
            if cell_center.distance(center) <= radius {
                let mut is_obstacle = false;
                for obs in &env.obstacles {
                    if obs.rect.overlaps(&node_bounds) {
                        is_obstacle = true;
                        break;
                    }
                }

                return QuadNode::Leaf(CellState {
                    explored: true,
                    obstacle: is_obstacle,
                    last_seen: tick,
                });
            } else {
                return node;
            }
        }

        let mut children = match node {
            QuadNode::Leaf(state) => {
                Box::new([
                    QuadNode::Leaf(state),
                    QuadNode::Leaf(state),
                    QuadNode::Leaf(state),
                    QuadNode::Leaf(state),
                ])
            }
            QuadNode::Internal(c) => c,
        };

        let hw = node_bounds.w / 2.0;
        let hh = node_bounds.h / 2.0;
        let x = node_bounds.x;
        let y = node_bounds.y;

        let quadrants = [
            Rect::new(x, y, hw, hh),             // Top-Left
            Rect::new(x + hw, y, hw, hh),        // Top-Right
            Rect::new(x, y + hh, hw, hh),        // Bottom-Left
            Rect::new(x + hw, y + hh, hw, hh),   // Bottom-Right
        ];

        for i in 0..4 {
            children[i] = Self::insert_node(
                std::mem::replace(&mut children[i], QuadNode::Leaf(CellState::default())),
                quadrants[i],
                scan_bounds,
                center,
                radius,
                env,
                tick,
                depth + 1,
                max_depth,
            );
        }

        if let (
            QuadNode::Leaf(s0),
            QuadNode::Leaf(s1),
            QuadNode::Leaf(s2),
            QuadNode::Leaf(s3),
        ) = (&children[0], &children[1], &children[2], &children[3])
        {
            if s0 == s1 && s1 == s2 && s2 == s3 {
                return QuadNode::Leaf(*s0);
            }
        }

        QuadNode::Internal(children)
    }

    pub fn diff_since(&self, since_tick: u64) -> MapDiff {
        let mut updates = Vec::new();
        Self::collect_diff(&self.root, self.bounds, since_tick, &mut updates);
        MapDiff {
            timestamp: since_tick,
            updates: updates.into_iter().map(|(r, s)| ((r.x, r.y, r.w, r.h), s)).collect(),
        }
    }

    fn collect_diff(node: &QuadNode, bounds: Rect, since_tick: u64, updates: &mut Vec<(Rect, CellState)>) {
        match node {
            QuadNode::Leaf(state) => {
                if state.last_seen >= since_tick && state.explored {
                    updates.push((bounds, *state));
                }
            }
            QuadNode::Internal(children) => {
                let hw = bounds.w / 2.0;
                let hh = bounds.h / 2.0;
                let x = bounds.x;
                let y = bounds.y;

                let quadrants = [
                    Rect::new(x, y, hw, hh),
                    Rect::new(x + hw, y, hw, hh),
                    Rect::new(x, y + hh, hw, hh),
                    Rect::new(x + hw, y + hh, hw, hh),
                ];

                for i in 0..4 {
                    Self::collect_diff(&children[i], quadrants[i], since_tick, updates);
                }
            }
        }
    }

    pub fn merge(&mut self, diff: MapDiff) {
        for (rect_tuple, state) in diff.updates {
            let rect = Rect::new(rect_tuple.0, rect_tuple.1, rect_tuple.2, rect_tuple.3);
            self.root = Self::merge_node(
                std::mem::replace(&mut self.root, QuadNode::Leaf(CellState::default())),
                self.bounds,
                rect,
                state,
                0,
                self.max_depth,
            );
        }
    }

    fn merge_node(
        node: QuadNode,
        node_bounds: Rect,
        target_rect: Rect,
        new_state: CellState,
        depth: u8,
        max_depth: u8,
    ) -> QuadNode {
        if !node_bounds.overlaps(&target_rect) {
            return node;
        }

        const EPSILON: f32 = 0.1;
        if (node_bounds.x - target_rect.x).abs() < EPSILON
            && (node_bounds.y - target_rect.y).abs() < EPSILON
            && (node_bounds.w - target_rect.w).abs() < EPSILON
            && (node_bounds.h - target_rect.h).abs() < EPSILON
            || depth >= max_depth
        {
            return QuadNode::Leaf(new_state);
        }

        let mut children = match node {
            QuadNode::Leaf(state) => Box::new([
                QuadNode::Leaf(state),
                QuadNode::Leaf(state),
                QuadNode::Leaf(state),
                QuadNode::Leaf(state),
            ]),
            QuadNode::Internal(c) => c,
        };

        let hw = node_bounds.w / 2.0;
        let hh = node_bounds.h / 2.0;
        let x = node_bounds.x;
        let y = node_bounds.y;

        let quadrants = [
            Rect::new(x, y, hw, hh),
            Rect::new(x + hw, y, hw, hh),
            Rect::new(x, y + hh, hw, hh),
            Rect::new(x + hw, y + hh, hw, hh),
        ];

        for i in 0..4 {
            children[i] = Self::merge_node(
                std::mem::replace(&mut children[i], QuadNode::Leaf(CellState::default())),
                quadrants[i],
                target_rect,
                new_state,
                depth + 1,
                max_depth,
            );
        }

        if let (
            QuadNode::Leaf(s0),
            QuadNode::Leaf(s1),
            QuadNode::Leaf(s2),
            QuadNode::Leaf(s3),
        ) = (&children[0], &children[1], &children[2], &children[3])
        {
            if s0 == s1 && s1 == s2 && s2 == s3 {
                return QuadNode::Leaf(*s0);
            }
        }

        QuadNode::Internal(children)
    }

    pub fn draw(&self, camera_bounds: Rect) {
        Self::draw_node(&self.root, self.bounds, camera_bounds);
    }

    fn draw_node(node: &QuadNode, bounds: Rect, camera_bounds: Rect) {
        if !bounds.overlaps(&camera_bounds) {
            return;
        }

        match node {
            QuadNode::Leaf(state) => {
                if state.explored {
                    let color = if state.obstacle {
                        Color::new(0.5, 0.5, 0.5, 1.0)
                    } else {
                        Color::new(0.2, 0.2, 0.2, 1.0)
                    };
                    draw_rectangle(bounds.x, bounds.y, bounds.w, bounds.h, color);
                }
            }
            QuadNode::Internal(children) => {
                let hw = bounds.w / 2.0;
                let hh = bounds.h / 2.0;
                let x = bounds.x;
                let y = bounds.y;

                let quadrants = [
                    Rect::new(x, y, hw, hh),
                    Rect::new(x + hw, y, hw, hh),
                    Rect::new(x, y + hh, hw, hh),
                    Rect::new(x + hw, y + hh, hw, hh),
                ];

                for i in 0..4 {
                    Self::draw_node(&children[i], quadrants[i], camera_bounds);
                }
            }
        }
    }
}
