use macroquad::prelude::*;
use pathfinding::prelude::astar;
use crate::quadtree::Quadtree;
use crate::environment::{MAP_WIDTH, MAP_HEIGHT};

const GRID_RES: f32 = 25.0;
const GRID_WIDTH: i32 = (MAP_WIDTH / GRID_RES) as i32;
const GRID_HEIGHT: i32 = (MAP_HEIGHT / GRID_RES) as i32;

/// Grid cell state encoding used in the rasterized pathfinding grid.
const CELL_UNKNOWN: u8 = 0;
const CELL_EXPLORED: u8 = 1;
const CELL_OBSTACLE: u8 = 2;

pub struct AutonomyState {
    /// The drone's current planned waypoint path.
    pub path: Option<Vec<Vec2>>,
    /// The frontier cell this drone is currently targeting.
    pub target_frontier: Option<Vec2>,
    /// All detected frontier cells (for rendering).
    pub all_frontiers: Vec<Vec2>,
    /// Internal cooldown to prevent thrashing on recalculation.
    pub(crate) recalculate_cooldown: f32,
    /// Frontiers that have been proven unreachable and should be skipped.
    pub(crate) banned_frontiers: Vec<Vec2>,
    /// Countdown timer for forced periodic repathing.
    pub(crate) repath_timer: f32,
}

impl Default for AutonomyState {
    fn default() -> Self {
        Self {
            path: None,
            target_frontier: None,
            all_frontiers: Vec::new(),
            recalculate_cooldown: 0.0,
            banned_frontiers: Vec::new(),
            repath_timer: 0.0,
        }
    }
}

impl AutonomyState {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn ban_current_target(&mut self) {
        if let Some(target) = self.target_frontier {
            self.banned_frontiers.push(target);
            self.target_frontier = None;
            self.path = None;
        }
    }

    /// Updates autonomy: rebuilds the grid, finds frontiers, and picks/recalculates
    /// the best target path. Called every physics tick.
    pub fn update(
        &mut self,
        position: Vec2,
        map: &Quadtree,
        dt: f32,
        peer_positions: &[(f32, f32)],
        peer_paths: &[Option<Vec<(f32, f32)>>],
    ) {
        if self.recalculate_cooldown > 0.0 {
            self.recalculate_cooldown -= dt;
            return;
        }

        self.repath_timer -= dt;

        if self.path.is_none() || self.path.as_ref().unwrap().is_empty() || self.repath_timer <= 0.0 {
            self.repath_timer = 0.25;

            let mut grid = vec![CELL_UNKNOWN; (GRID_WIDTH * GRID_HEIGHT) as usize];
            for y in 0..GRID_HEIGHT {
                for x in 0..GRID_WIDTH {
                    let center = vec2(x as f32 * GRID_RES + GRID_RES / 2.0, y as f32 * GRID_RES + GRID_RES / 2.0);
                    if let Some(s) = map.query_point(center) {
                        if s.obstacle {
                            grid[(y * GRID_WIDTH + x) as usize] = CELL_OBSTACLE;
                        } else if s.explored {
                            grid[(y * GRID_WIDTH + x) as usize] = CELL_EXPLORED;
                        }
                    }
                }
            }

            // Reuse the existing Vec to avoid re-allocation every 250ms
            find_frontiers(&grid, &mut self.all_frontiers);

            if let Some(target) = pick_nearest_frontier(position, &self.all_frontiers, &self.banned_frontiers, peer_positions, peer_paths) {
                self.target_frontier = Some(target);
                self.path = calculate_path(position, target, &grid);

                if self.path.is_none() {
                    println!("WARNING: Target frontier at {:?} is unreachable. Banning it.", target);
                    self.banned_frontiers.push(target);
                }
            } else {
                if self.all_frontiers.is_empty() {
                    println!("SUCCESS: Exploration Complete! No frontiers left.");
                } else {
                    println!("WARNING: All {} remaining frontiers are banned!", self.all_frontiers.len());
                }
                self.target_frontier = None;
                self.path = None;
                self.recalculate_cooldown = 1.0;
            }
        }
    }
}


// Free-function shim for backwards compatibility during refactor.
// Delegates to the method form.
pub fn update_autonomy(
    state: &mut AutonomyState,
    position: Vec2,
    map: &Quadtree,
    dt: f32,
    peer_positions: &[(f32, f32)],
    peer_paths: &[Option<Vec<(f32, f32)>>],
) {
    state.update(position, map, dt, peer_positions, peer_paths);
}

fn pick_nearest_frontier(
    position: Vec2,
    frontiers: &[Vec2],
    banned: &[Vec2],
    peer_positions: &[(f32, f32)],
    peer_paths: &[Option<Vec<(f32, f32)>>],
) -> Option<Vec2> {
    let mut best = None;
    let mut min_score = f32::MAX;

    for &f in frontiers {
        if banned.iter().any(|&b| b.distance_squared(f) < 1.0) {
            continue;
        }

        let mut score = position.distance_squared(f);

        // Penalize frontiers near other drones' current positions
        for &(px, py) in peer_positions {
            let peer_pos = vec2(px, py);
            let peer_dist = peer_pos.distance_squared(f);
            if peer_dist < 500.0 * 500.0 {
                // The closer the peer is to this frontier, the heavier the penalty
                score += (500.0 * 500.0 - peer_dist) * 2.0;
            }
        }

        // Penalize frontiers near other drones' planned paths
        for path_opt in peer_paths {
            if let Some(path) = path_opt {
                for &(wx, wy) in path {
                    let waypoint = vec2(wx, wy);
                    let wp_dist = waypoint.distance_squared(f);
                    if wp_dist < 300.0 * 300.0 {
                        score += (300.0 * 300.0 - wp_dist) * 1.0;
                    }
                }
            }
        }

        if score < min_score {
            min_score = score;
            best = Some(f);
        }
    }
    best
}

fn find_frontiers(grid: &[u8], frontiers: &mut Vec<Vec2>) {
    frontiers.clear();

    for y in 0..GRID_HEIGHT {
        for x in 0..GRID_WIDTH {
            let idx = (y * GRID_WIDTH + x) as usize;

            if grid[idx] == CELL_EXPLORED {
                let mut is_frontier = false;
                'outer: for dx in -1..=1 {
                    for dy in -1..=1 {
                        if dx == 0 && dy == 0 { continue; }
                        let nx = x + dx;
                        let ny = y + dy;
                        if nx >= 0 && nx < GRID_WIDTH && ny >= 0 && ny < GRID_HEIGHT {
                            let n_idx = (ny * GRID_WIDTH + nx) as usize;
                            if grid[n_idx] == CELL_UNKNOWN {
                                is_frontier = true;
                                break 'outer;
                            }
                        }
                    }
                }

                if is_frontier {
                    let center = vec2(x as f32 * GRID_RES + GRID_RES / 2.0, y as f32 * GRID_RES + GRID_RES / 2.0);
                    frontiers.push(center);
                }
            }
        }
    }
}

fn calculate_path(start: Vec2, target: Vec2, grid: &[u8]) -> Option<Vec<Vec2>> {
    let start_grid = (
        (start.x / GRID_RES).clamp(0.0, (GRID_WIDTH - 1) as f32) as i32,
        (start.y / GRID_RES).clamp(0.0, (GRID_HEIGHT - 1) as f32) as i32,
    );
    let target_grid = (
        (target.x / GRID_RES).clamp(0.0, (GRID_WIDTH - 1) as f32) as i32,
        (target.y / GRID_RES).clamp(0.0, (GRID_HEIGHT - 1) as f32) as i32,
    );

    let result = astar(
        &start_grid,
        |&(x, y)| {
            let mut successors = Vec::new();
            for dx in -1..=1 {
                for dy in -1..=1 {
                    if dx == 0 && dy == 0 { continue; }
                    let nx = x + dx;
                    let ny = y + dy;
                    if nx >= 0 && nx < GRID_WIDTH && ny >= 0 && ny < GRID_HEIGHT {
                        let is_target = nx == target_grid.0 && ny == target_grid.1;
                        let n_idx = (ny * GRID_WIDTH + nx) as usize;
                        
                        if grid[n_idx] == CELL_EXPLORED || is_target {
                            // DIAGONAL SQUEEZE CHECK
                            let mut can_move = true;
                            if dx != 0 && dy != 0 {
                                let is_obstacle = |cx, cy| -> bool {
                                    if cx >= 0 && cx < GRID_WIDTH && cy >= 0 && cy < GRID_HEIGHT {
                                        grid[(cy * GRID_WIDTH + cx) as usize] == CELL_OBSTACLE
                                    } else {
                                        false
                                    }
                                };
                                
                                if is_obstacle(nx, y) || is_obstacle(x, ny) {
                                    can_move = false;
                                }
                            }

                            if can_move {
                                let cost = if dx != 0 && dy != 0 { 14 } else { 10 };
                                
                                let mut penalty = 0;
                                for cx in -2..=2 {
                                    for cy in -2..=2 {
                                        if cx == 0 && cy == 0 { continue; }
                                        let chk_x = nx + cx;
                                        let chk_y = ny + cy;
                                        if chk_x >= 0 && chk_x < GRID_WIDTH && chk_y >= 0 && chk_y < GRID_HEIGHT {
                                        if grid[(chk_y * GRID_WIDTH + chk_x) as usize] == CELL_OBSTACLE {
                                                let dist_sq = cx * cx + cy * cy;
                                                if dist_sq <= 1 {
                                                    penalty += 1000;
                                                } else if dist_sq <= 4 {
                                                    penalty += 200;
                                                }
                                            }
                                        }
                                    }
                                }
                                
                                successors.push(((nx, ny), cost + penalty));
                            }
                        }
                    }
                }
            }
            successors
        },
        |&(x, y)| {
            let dx = (x - target_grid.0).abs();
            let dy = (y - target_grid.1).abs();
            (dx + dy) * 10
        },
        |&p| p == target_grid,
    );

    if let Some((path, _cost)) = result {
        let mut world_path = Vec::new();
        for (x, y) in path {
            world_path.push(vec2(x as f32 * GRID_RES + GRID_RES / 2.0, y as f32 * GRID_RES + GRID_RES / 2.0));
        }
        // Remove the starting node so the drone moves to the NEXT node immediately
        if !world_path.is_empty() {
            world_path.remove(0);
        }
        Some(world_path)
    } else {
        None
    }
}
