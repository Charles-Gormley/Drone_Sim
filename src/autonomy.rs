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
    /// Pre-allocated grid for pathfinding and frontier detection.
    pub(crate) grid: Vec<u8>,
    /// Pre-allocated grid for staleness calculation.
    pub(crate) last_seen_grid: Vec<u64>,
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
            grid: vec![CELL_UNKNOWN; (GRID_WIDTH * GRID_HEIGHT) as usize],
            last_seen_grid: vec![0u64; (GRID_WIDTH * GRID_HEIGHT) as usize],
        }
    }
}

pub struct SwarmContext<'a> {
    pub peer_positions: &'a [(f32, f32)],
    pub peer_paths: &'a [Option<Vec<(f32, f32)>>],
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
        current_tick: u64,
        swarm: &SwarmContext,
    ) {
        if self.recalculate_cooldown > 0.0 {
            self.recalculate_cooldown -= dt;
            return;
        }

        self.repath_timer -= dt;

        if self.path.is_none() || self.path.as_ref().unwrap().is_empty() || self.repath_timer <= 0.0 {
            self.repath_timer = 0.25;

            // Clear the pre-allocated grids
            self.grid.fill(CELL_UNKNOWN);
            self.last_seen_grid.fill(0);

            for y in 0..GRID_HEIGHT {
                for x in 0..GRID_WIDTH {
                    let center = vec2(x as f32 * GRID_RES + GRID_RES / 2.0, y as f32 * GRID_RES + GRID_RES / 2.0);
                    if let Some(s) = map.query_point(center) {
                        self.last_seen_grid[(y * GRID_WIDTH + x) as usize] = s.last_seen;
                        if s.obstacle {
                            self.grid[(y * GRID_WIDTH + x) as usize] = CELL_OBSTACLE;
                        } else if s.explored {
                            self.grid[(y * GRID_WIDTH + x) as usize] = CELL_EXPLORED;
                        }
                    }
                }
            }

            // Reuse the existing Vec to avoid re-allocation every 250ms
            let mut frontier_values = Vec::new();
            find_frontiers(&self.grid, &self.last_seen_grid, current_tick, &mut frontier_values);
            self.all_frontiers = frontier_values.iter().map(|(p, _)| *p).collect();

            if let Some(target) = pick_nearest_frontier(position, &frontier_values, &self.banned_frontiers, swarm) {
                self.target_frontier = Some(target);
                self.path = calculate_path(position, target, &self.grid);

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
    current_tick: u64,
    swarm: &SwarmContext,
) {
    state.update(position, map, dt, current_tick, swarm);
}

fn pick_nearest_frontier(
    position: Vec2,
    frontiers: &[(Vec2, f32)],
    banned: &[Vec2],
    swarm: &SwarmContext,
) -> Option<Vec2> {
    let mut best = None;
    let mut max_utility = f32::MIN;

    for &(f, value) in frontiers {
        if banned.iter().any(|&b| b.distance_squared(f) < 1.0) {
            continue;
        }

        let distance_cost = position.distance(f);
        let mut peer_cost = 0.0;

        // Penalize frontiers near other drones' current positions
        for &(px, py) in swarm.peer_positions {
            let peer_pos = vec2(px, py);
            let peer_dist = peer_pos.distance(f);
            if peer_dist < 500.0 {
                peer_cost += (500.0 - peer_dist) * 2.0;
            }
        }

        // Penalize frontiers near other drones' planned paths
        for path_opt in swarm.peer_paths {
            if let Some(path) = path_opt {
                for &(wx, wy) in path {
                    let waypoint = vec2(wx, wy);
                    let wp_dist = waypoint.distance(f);
                    if wp_dist < 300.0 {
                        peer_cost += (300.0 - wp_dist) * 1.5;
                    }
                }
            }
        }

        let cost = distance_cost + peer_cost;
        let utility = value - cost;

        if utility > max_utility {
            max_utility = utility;
            best = Some(f);
        }
    }
    best
}

fn find_frontiers(grid: &[u8], last_seen_grid: &[u64], current_tick: u64, frontiers: &mut Vec<(Vec2, f32)>) {
    frontiers.clear();
    let mut has_unknown_frontiers = false;

    for y in 0..GRID_HEIGHT {
        for x in 0..GRID_WIDTH {
            let idx = (y * GRID_WIDTH + x) as usize;

            if grid[idx] == CELL_EXPLORED {
                let mut is_unknown_frontier = false;
                'outer: for dx in -1..=1 {
                    for dy in -1..=1 {
                        if dx == 0 && dy == 0 { continue; }
                        let nx = x + dx;
                        let ny = y + dy;
                        if nx >= 0 && nx < GRID_WIDTH && ny >= 0 && ny < GRID_HEIGHT {
                            let n_idx = (ny * GRID_WIDTH + nx) as usize;
                            if grid[n_idx] == CELL_UNKNOWN {
                                is_unknown_frontier = true;
                                break 'outer;
                            }
                        }
                    }
                }

                let center = vec2(x as f32 * GRID_RES + GRID_RES / 2.0, y as f32 * GRID_RES + GRID_RES / 2.0);

                if is_unknown_frontier {
                    frontiers.push((center, 1_000_000.0)); // Huge value for unknown frontiers
                    has_unknown_frontiers = true;
                } else if !has_unknown_frontiers {
                    // Collect stale explored cells if we haven't found any unknown frontiers yet.
                    // If we find an unknown frontier, we will filter these out later.
                    let age = current_tick.saturating_sub(last_seen_grid[idx]) as f32;
                    if age > 600.0 { // Must be at least 10 seconds stale
                        frontiers.push((center, age));
                    }
                }
            }
        }
    }

    // If we found ANY unknown frontiers, discard the stale ones.
    if has_unknown_frontiers {
        frontiers.retain(|&(_, val)| val == 1_000_000.0);
    } else {
        // If we only have stale frontiers, sort and keep the top 100 stalest to reduce compute
        frontiers.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        frontiers.truncate(100);
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
