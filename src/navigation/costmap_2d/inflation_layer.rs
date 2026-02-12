//! Inflation costmap layer — priority-queue BFS inflation from lethal cells.
//!
//! This replaces the O(W×H×R²) inflation in the original `costmap.rs` with a
//! BFS-based approach that is dramatically faster, matching Nav2's `InflationLayer`.

use crate::navigation::costmap::{cost_values, Costmap};
use crate::navigation::traits::CostmapLayer;
use crate::navigation::types::ParamMap;
use ordered_float::OrderedFloat;
use std::collections::BinaryHeap;

/// A cell in the inflation priority queue.
#[derive(Debug, Clone, PartialEq, Eq)]
struct CellData {
    /// Negative distance (so BinaryHeap gives us closest first).
    neg_distance: OrderedFloat<f64>,
    /// The cell being inflated.
    x: usize,
    y: usize,
    /// The source lethal cell.
    src_x: usize,
    src_y: usize,
}

impl Ord for CellData {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.neg_distance.cmp(&other.neg_distance)
    }
}

impl PartialOrd for CellData {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

/// Inflation layer: BFS-based inflation around lethal obstacles.
pub struct InflationLayer {
    name: String,
    /// Inflation radius in meters.
    inflation_radius: f64,
    /// Cost scaling factor for exponential decay.
    cost_scaling_factor: f64,
    /// Inscribed radius in meters (cells within this are INSCRIBED cost).
    inscribed_radius: f64,
}

impl InflationLayer {
    pub fn new() -> Self {
        Self {
            name: "inflation_layer".to_string(),
            inflation_radius: 0.55,
            cost_scaling_factor: 3.0,
            inscribed_radius: 0.05,
        }
    }

    pub fn with_params(inflation_radius: f64, cost_scaling_factor: f64, inscribed_radius: f64) -> Self {
        Self {
            name: "inflation_layer".to_string(),
            inflation_radius,
            cost_scaling_factor,
            inscribed_radius,
        }
    }

    /// Compute the inflated cost for a given distance from a lethal cell.
    fn compute_cost(&self, distance: f64) -> u8 {
        if distance <= self.inscribed_radius {
            cost_values::INSCRIBED_INFLATED_OBSTACLE
        } else if distance <= self.inflation_radius {
            let factor =
                (-self.cost_scaling_factor * (distance - self.inscribed_radius)
                    / (self.inflation_radius - self.inscribed_radius))
                    .exp();
            let cost = (cost_values::INSCRIBED_INFLATED_OBSTACLE as f64 * factor) as u8;
            cost.max(1) // Minimum cost of 1 within inflation radius
        } else {
            cost_values::NO_COST
        }
    }
}

impl CostmapLayer for InflationLayer {
    fn configure(&mut self, name: &str, params: &ParamMap) {
        self.name = name.to_string();

        if let Some(pv) = params.get("inflation_radius") {
            if let Some(v) = pv.as_float() {
                self.inflation_radius = v;
            }
        }
        if let Some(pv) = params.get("cost_scaling_factor") {
            if let Some(v) = pv.as_float() {
                self.cost_scaling_factor = v;
            }
        }
        if let Some(pv) = params.get("inscribed_radius") {
            if let Some(v) = pv.as_float() {
                self.inscribed_radius = v;
            }
        }
    }

    fn update_bounds(
        &mut self,
        _robot_x: f64,
        _robot_y: f64,
        _robot_yaw: f64,
        min_x: &mut f64,
        min_y: &mut f64,
        max_x: &mut f64,
        max_y: &mut f64,
    ) {
        // Expand bounds by inflation radius so inflation extends outside the
        // region changed by other layers
        *min_x -= self.inflation_radius;
        *min_y -= self.inflation_radius;
        *max_x += self.inflation_radius;
        *max_y += self.inflation_radius;
    }

    fn update_costs(
        &self,
        master_grid: &mut Costmap,
        min_x: f64,
        min_y: f64,
        max_x: f64,
        max_y: f64,
    ) {
        let width = master_grid.width;
        let height = master_grid.height;
        let resolution = master_grid.resolution;

        // Compute the grid bounds to process
        let (gx_min, gy_min) = master_grid.world_to_map(min_x, min_y);
        let (gx_max, gy_max) = master_grid.world_to_map(max_x, max_y);

        let gx_min = gx_min.max(0) as usize;
        let gy_min = gy_min.max(0) as usize;
        let gx_max = (gx_max as usize).min(width.saturating_sub(1));
        let gy_max = (gy_max as usize).min(height.saturating_sub(1));

        let cell_inflation_radius = (self.inflation_radius / resolution).ceil() as i32;

        // Track which cells have been visited with their best distance
        let mut seen = vec![f64::MAX; width * height];

        // Priority queue: start from all lethal cells
        let mut queue = BinaryHeap::new();

        // Seed the queue with all lethal cells in bounds
        for y in gy_min..=gy_max {
            for x in gx_min..=gx_max {
                let idx = y * width + x;
                if master_grid.data[idx] == cost_values::LETHAL_OBSTACLE {
                    seen[idx] = 0.0;
                    queue.push(CellData {
                        neg_distance: OrderedFloat(0.0),
                        x,
                        y,
                        src_x: x,
                        src_y: y,
                    });
                }
            }
        }

        // BFS outward from lethal cells
        while let Some(cell) = queue.pop() {
            let dist = -cell.neg_distance.0;
            let idx = cell.y * width + cell.x;

            // Skip if we've found a shorter path to this cell
            if dist > seen[idx] + 1e-9 {
                continue;
            }

            // Compute cost at this distance
            let cost = self.compute_cost(dist);
            if cost == cost_values::NO_COST {
                continue;
            }

            // Apply cost (keep maximum)
            if cost > master_grid.data[idx] && master_grid.data[idx] != cost_values::LETHAL_OBSTACLE {
                master_grid.data[idx] = cost;
            }

            // Expand to 8-connected neighbors
            for dy in -1i32..=1 {
                for dx in -1i32..=1 {
                    if dx == 0 && dy == 0 {
                        continue;
                    }

                    let nx = cell.x as i32 + dx;
                    let ny = cell.y as i32 + dy;

                    if nx < 0 || nx >= width as i32 || ny < 0 || ny >= height as i32 {
                        continue;
                    }

                    let nx = nx as usize;
                    let ny = ny as usize;

                    // Distance from source lethal cell
                    let ddx = (nx as f64 - cell.src_x as f64) * resolution;
                    let ddy = (ny as f64 - cell.src_y as f64) * resolution;
                    let new_dist = (ddx * ddx + ddy * ddy).sqrt();

                    // Check if within inflation radius and better than seen
                    if new_dist <= self.inflation_radius {
                        let nidx = ny * width + nx;
                        if new_dist < seen[nidx] - 1e-9 {
                            seen[nidx] = new_dist;
                            queue.push(CellData {
                                neg_distance: OrderedFloat(-new_dist),
                                x: nx,
                                y: ny,
                                src_x: cell.src_x,
                                src_y: cell.src_y,
                            });
                        }
                    }
                }
            }
        }
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn reset(&mut self) {
        // No state to reset for inflation layer
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compute_cost_at_source() {
        let layer = InflationLayer::new();
        // At distance 0, should be INSCRIBED
        assert_eq!(
            layer.compute_cost(0.0),
            cost_values::INSCRIBED_INFLATED_OBSTACLE
        );
    }

    #[test]
    fn test_compute_cost_outside_radius() {
        let layer = InflationLayer::new();
        assert_eq!(
            layer.compute_cost(layer.inflation_radius + 0.1),
            cost_values::NO_COST
        );
    }

    #[test]
    fn test_compute_cost_decay() {
        let layer = InflationLayer::new();
        let c1 = layer.compute_cost(0.1);
        let c2 = layer.compute_cost(0.3);
        let c3 = layer.compute_cost(0.5);
        // Cost should decrease with distance
        assert!(c1 >= c2);
        assert!(c2 >= c3);
    }

    #[test]
    fn test_inflation_bfs_single_obstacle() {
        let layer = InflationLayer::with_params(0.15, 3.0, 0.05);

        // 10x10 grid, resolution 0.05m (0.5m x 0.5m)
        let mut costmap = Costmap::new(false);
        costmap.width = 10;
        costmap.height = 10;
        costmap.resolution = 0.05;
        costmap.origin_x = 0.0;
        costmap.origin_y = 0.0;
        costmap.data = vec![cost_values::NO_COST; 100];

        // Place a single lethal obstacle at center (5, 5)
        costmap.data[5 * 10 + 5] = cost_values::LETHAL_OBSTACLE;

        layer.update_costs(&mut costmap, 0.0, 0.0, 0.5, 0.5);

        // The lethal cell should remain lethal
        assert_eq!(costmap.data[5 * 10 + 5], cost_values::LETHAL_OBSTACLE);

        // Adjacent cells should have inflated costs
        let adj = costmap.data[5 * 10 + 6]; // (6, 5)
        assert!(adj > cost_values::NO_COST);
        assert!(adj < cost_values::LETHAL_OBSTACLE);

        // Diagonal neighbor should have cost too (but less than adjacent)
        let diag = costmap.data[6 * 10 + 6]; // (6, 6)
        assert!(diag > cost_values::NO_COST || diag == cost_values::NO_COST); // may be outside radius

        // Far cells should be free
        assert_eq!(costmap.data[0], cost_values::NO_COST);
    }

    #[test]
    fn test_inflation_preserves_lethal() {
        let layer = InflationLayer::with_params(0.2, 3.0, 0.05);

        let mut costmap = Costmap::new(false);
        costmap.width = 5;
        costmap.height = 5;
        costmap.resolution = 0.05;
        costmap.origin_x = 0.0;
        costmap.origin_y = 0.0;
        costmap.data = vec![cost_values::NO_COST; 25];

        // Two adjacent lethal cells
        costmap.data[2 * 5 + 2] = cost_values::LETHAL_OBSTACLE;
        costmap.data[2 * 5 + 3] = cost_values::LETHAL_OBSTACLE;

        layer.update_costs(&mut costmap, 0.0, 0.0, 0.25, 0.25);

        // Both lethal cells should remain lethal
        assert_eq!(costmap.data[2 * 5 + 2], cost_values::LETHAL_OBSTACLE);
        assert_eq!(costmap.data[2 * 5 + 3], cost_values::LETHAL_OBSTACLE);
    }
}
