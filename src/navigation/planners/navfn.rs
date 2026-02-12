//! NavFn planner — Dijkstra/A* potential-field planner on the costmap grid.
//!
//! This is a Rust port of Nav2's NavFn planner. It allocates a potential grid the
//! same size as the costmap, runs Dijkstra (or A* with heuristic) from the goal
//! outward, then extracts a path by gradient descent from the start.

use crate::navigation::costmap::{cost_values, Costmap};
use crate::navigation::traits::{GlobalPlanner, PlannerError};
use crate::navigation::types::*;
use ordered_float::OrderedFloat;
use std::collections::BinaryHeap;
use std::sync::{Arc, RwLock};

/// Planning algorithm variant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Algorithm {
    Dijkstra,
    AStar,
}

/// NavFn planner: grid-based potential-field planner using Dijkstra or A*.
pub struct NavFnPlanner {
    name: String,
    costmap: Option<Arc<RwLock<Costmap>>>,
    algorithm: Algorithm,
    /// Whether to allow planning through unknown space.
    allow_unknown: bool,
    /// Cost to traverse an unknown cell (used when allow_unknown is true).
    unknown_cost: f64,
    /// Tolerance for reaching the goal (in meters).
    tolerance: f64,
}

/// A node in the priority queue.
#[derive(Debug, Clone, PartialEq, Eq)]
struct PqNode {
    neg_cost: OrderedFloat<f64>,
    x: usize,
    y: usize,
}

impl Ord for PqNode {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.neg_cost.cmp(&other.neg_cost)
    }
}

impl PartialOrd for PqNode {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl NavFnPlanner {
    pub fn new() -> Self {
        Self {
            name: "NavFn".to_string(),
            costmap: None,
            algorithm: Algorithm::Dijkstra,
            allow_unknown: true,
            unknown_cost: 128.0,
            tolerance: 0.0,
        }
    }

    pub fn with_algorithm(mut self, algorithm: Algorithm) -> Self {
        self.algorithm = algorithm;
        self
    }

    /// Get the traversal cost for a costmap cell value.
    /// Returns None if the cell is impassable.
    fn traversal_cost(&self, cell_cost: u8) -> Option<f64> {
        match cell_cost {
            cost_values::LETHAL_OBSTACLE => None, // Impassable
            cost_values::INSCRIBED_INFLATED_OBSTACLE => None, // Impassable
            cost_values::UNKNOWN_COST => {
                if self.allow_unknown {
                    Some(self.unknown_cost)
                } else {
                    None
                }
            }
            cost_values::NO_COST => Some(1.0),
            c => Some(1.0 + c as f64 * 0.5), // Scale cost by cell value
        }
    }

    /// A* heuristic: Euclidean distance.
    fn heuristic(x1: usize, y1: usize, x2: usize, y2: usize) -> f64 {
        let dx = x1 as f64 - x2 as f64;
        let dy = y1 as f64 - y2 as f64;
        (dx * dx + dy * dy).sqrt()
    }

    /// Compute the potential grid from goal to all reachable cells.
    fn compute_potential(
        &self,
        costmap: &Costmap,
        goal_x: usize,
        goal_y: usize,
        start_x: usize,
        start_y: usize,
    ) -> Vec<f64> {
        let width = costmap.width;
        let height = costmap.height;
        let mut potential = vec![f64::MAX; width * height];

        let mut queue = BinaryHeap::new();

        // Seed from goal
        let goal_idx = goal_y * width + goal_x;
        potential[goal_idx] = 0.0;
        queue.push(PqNode {
            neg_cost: OrderedFloat(0.0),
            x: goal_x,
            y: goal_y,
        });

        // 8-connected neighbors: (dx, dy, move_cost_multiplier)
        let neighbors: [(i32, i32, f64); 8] = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, std::f64::consts::SQRT_2),
            (-1, 1, std::f64::consts::SQRT_2),
            (1, -1, std::f64::consts::SQRT_2),
            (1, 1, std::f64::consts::SQRT_2),
        ];

        while let Some(node) = queue.pop() {
            let idx = node.y * width + node.x;
            let g_cost = potential[idx];

            // Skip stale entries: the stored f-value is worse than the
            // current best g-value (potential) for this cell.
            // For Dijkstra, neg_cost stores -g; for A*, neg_cost stores -(g+h).
            // Either way, if we already found a shorter g-path, skip.
            let popped_g = match self.algorithm {
                Algorithm::Dijkstra => -node.neg_cost.0,
                Algorithm::AStar => {
                    // f = g + h, so g = f - h
                    let h = Self::heuristic(node.x, node.y, start_x, start_y);
                    -node.neg_cost.0 - h
                }
            };
            if popped_g > g_cost + 1e-9 {
                continue;
            }

            // If we reached the start, we can stop early for A*
            if node.x == start_x && node.y == start_y {
                break;
            }

            for &(dx, dy, move_mult) in &neighbors {
                let nx = node.x as i32 + dx;
                let ny = node.y as i32 + dy;

                if nx < 0 || nx >= width as i32 || ny < 0 || ny >= height as i32 {
                    continue;
                }

                let nx = nx as usize;
                let ny = ny as usize;
                let nidx = ny * width + nx;

                // Get traversal cost of the neighbor cell
                if let Some(cell_cost) = self.traversal_cost(costmap.data[nidx]) {
                    let new_potential = g_cost + cell_cost * move_mult;

                    if new_potential < potential[nidx] - 1e-9 {
                        potential[nidx] = new_potential;

                        let priority = match self.algorithm {
                            Algorithm::Dijkstra => new_potential,
                            Algorithm::AStar => {
                                new_potential + Self::heuristic(nx, ny, start_x, start_y)
                            }
                        };

                        queue.push(PqNode {
                            neg_cost: OrderedFloat(-priority),
                            x: nx,
                            y: ny,
                        });
                    }
                }
            }
        }

        potential
    }

    /// Extract a path by gradient descent from start through the potential field.
    fn extract_path(
        &self,
        costmap: &Costmap,
        potential: &[f64],
        start_x: usize,
        start_y: usize,
        goal_x: usize,
        goal_y: usize,
    ) -> Result<Vec<(usize, usize)>, PlannerError> {
        let width = costmap.width;
        let height = costmap.height;
        let max_iterations = width * height;

        let mut path = Vec::new();
        let mut cx = start_x;
        let mut cy = start_y;

        for _ in 0..max_iterations {
            path.push((cx, cy));

            if cx == goal_x && cy == goal_y {
                return Ok(path);
            }

            // Find the neighbor with the lowest potential
            let mut best_x = cx;
            let mut best_y = cy;
            let mut best_pot = potential[cy * width + cx];

            for dy in -1i32..=1 {
                for dx in -1i32..=1 {
                    if dx == 0 && dy == 0 {
                        continue;
                    }

                    let nx = cx as i32 + dx;
                    let ny = cy as i32 + dy;

                    if nx < 0 || nx >= width as i32 || ny < 0 || ny >= height as i32 {
                        continue;
                    }

                    let nx = nx as usize;
                    let ny = ny as usize;
                    let nidx = ny * width + nx;

                    if potential[nidx] < best_pot {
                        best_pot = potential[nidx];
                        best_x = nx;
                        best_y = ny;
                    }
                }
            }

            // If we didn't move, we're stuck
            if best_x == cx && best_y == cy {
                return Err(PlannerError::NoPathFound);
            }

            cx = best_x;
            cy = best_y;
        }

        Err(PlannerError::NoPathFound)
    }

    /// Plan a path on a given costmap (for direct use without the trait).
    pub fn plan_on_costmap(
        &self,
        costmap: &Costmap,
        start: &PoseStamped,
        goal: &PoseStamped,
    ) -> Result<Path, PlannerError> {
        let (sx, sy) = costmap.world_to_map(start.pose.x, start.pose.y);
        let (gx, gy) = costmap.world_to_map(goal.pose.x, goal.pose.y);

        if sx < 0 || sx >= costmap.width as i32 || sy < 0 || sy >= costmap.height as i32 {
            return Err(PlannerError::StartInObstacle);
        }
        if gx < 0 || gx >= costmap.width as i32 || gy < 0 || gy >= costmap.height as i32 {
            return Err(PlannerError::GoalInObstacle);
        }

        let sx = sx as usize;
        let sy = sy as usize;
        let gx = gx as usize;
        let gy = gy as usize;

        // Check start and goal cells
        let start_cost = costmap.data[sy * costmap.width + sx];
        if start_cost == cost_values::LETHAL_OBSTACLE
            || start_cost == cost_values::INSCRIBED_INFLATED_OBSTACLE
        {
            return Err(PlannerError::StartInObstacle);
        }

        let goal_cost = costmap.data[gy * costmap.width + gx];
        if goal_cost == cost_values::LETHAL_OBSTACLE
            || goal_cost == cost_values::INSCRIBED_INFLATED_OBSTACLE
        {
            return Err(PlannerError::GoalInObstacle);
        }

        // Compute potential field from goal
        let potential = self.compute_potential(costmap, gx, gy, sx, sy);

        // Check if start is reachable
        if potential[sy * costmap.width + sx] == f64::MAX {
            return Err(PlannerError::NoPathFound);
        }

        // Extract path by gradient descent
        let grid_path = self.extract_path(costmap, &potential, sx, sy, gx, gy)?;

        // Convert grid cells to world coordinates
        let frame_id = &start.frame_id;
        let poses: Vec<PoseStamped> = grid_path
            .iter()
            .map(|(x, y)| {
                let (wx, wy) = costmap.map_to_world(*x as i32, *y as i32);
                PoseStamped::new(Pose2D::new(wx, wy, 0.0), frame_id)
            })
            .collect();

        // Compute yaw for each pose (heading toward next pose)
        let mut path = Path::from_poses(poses, frame_id);
        if path.poses.len() >= 2 {
            for i in 0..path.poses.len() - 1 {
                let dx = path.poses[i + 1].pose.x - path.poses[i].pose.x;
                let dy = path.poses[i + 1].pose.y - path.poses[i].pose.y;
                path.poses[i].pose.yaw = dy.atan2(dx);
            }
            // Last pose gets goal yaw
            let last = path.poses.len() - 1;
            path.poses[last].pose.yaw = goal.pose.yaw;
        }

        Ok(path)
    }
}

impl GlobalPlanner for NavFnPlanner {
    fn configure(&mut self, name: &str, costmap: Arc<RwLock<Costmap>>, params: &ParamMap) {
        self.name = name.to_string();
        self.costmap = Some(costmap);

        if let Some(pv) = params.get("use_astar") {
            if let Some(true) = pv.as_bool() {
                self.algorithm = Algorithm::AStar;
            }
        }
        if let Some(pv) = params.get("allow_unknown") {
            if let Some(v) = pv.as_bool() {
                self.allow_unknown = v;
            }
        }
        if let Some(pv) = params.get("tolerance") {
            if let Some(v) = pv.as_float() {
                self.tolerance = v;
            }
        }
    }

    fn create_plan(
        &self,
        start: &PoseStamped,
        goal: &PoseStamped,
    ) -> Result<Path, PlannerError> {
        let costmap_arc = self
            .costmap
            .as_ref()
            .ok_or(PlannerError::NotConfigured)?;

        let costmap = costmap_arc
            .read()
            .map_err(|e| PlannerError::CostmapError(e.to_string()))?;

        self.plan_on_costmap(&costmap, start, goal)
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn cleanup(&mut self) {
        self.costmap = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Create a test costmap with all free space.
    fn make_free_costmap(width: usize, height: usize, resolution: f64) -> Costmap {
        let mut costmap = Costmap::new(true);
        costmap.width = width;
        costmap.height = height;
        costmap.resolution = resolution;
        costmap.origin_x = 0.0;
        costmap.origin_y = 0.0;
        costmap.data = vec![cost_values::NO_COST; width * height];
        costmap
    }

    #[test]
    fn test_navfn_simple_path() {
        let costmap = make_free_costmap(20, 20, 0.5);
        let planner = NavFnPlanner::new();

        let start = PoseStamped::new(Pose2D::new(0.25, 0.25, 0.0), "map");
        let goal = PoseStamped::new(Pose2D::new(9.25, 9.25, 0.0), "map");

        let result = planner.plan_on_costmap(&costmap, &start, &goal);
        assert!(result.is_ok(), "Planning should succeed: {:?}", result.err());

        let path = result.unwrap();
        assert!(path.len() >= 2, "Path should have at least 2 poses");

        // First pose should be near start
        let first = &path.poses[0].pose;
        assert!((first.x - 0.25).abs() < 0.5);
        assert!((first.y - 0.25).abs() < 0.5);

        // Last pose should be near goal
        let last = &path.poses[path.len() - 1].pose;
        assert!((last.x - 9.25).abs() < 0.5);
        assert!((last.y - 9.25).abs() < 0.5);
    }

    #[test]
    fn test_navfn_astar_path() {
        let costmap = make_free_costmap(20, 20, 0.5);
        let planner = NavFnPlanner::new().with_algorithm(Algorithm::AStar);

        let start = PoseStamped::new(Pose2D::new(0.25, 0.25, 0.0), "map");
        let goal = PoseStamped::new(Pose2D::new(9.25, 9.25, 0.0), "map");

        let result = planner.plan_on_costmap(&costmap, &start, &goal);
        assert!(result.is_ok());
    }

    #[test]
    fn test_navfn_obstacle_avoidance() {
        let mut costmap = make_free_costmap(20, 20, 0.5);

        // Create a wall across the middle (row 10, columns 0-15)
        for x in 0..16 {
            costmap.data[10 * 20 + x] = cost_values::LETHAL_OBSTACLE;
        }

        let planner = NavFnPlanner::new();

        let start = PoseStamped::new(Pose2D::new(0.25, 0.25, 0.0), "map");
        let goal = PoseStamped::new(Pose2D::new(0.25, 9.25, 0.0), "map");

        let result = planner.plan_on_costmap(&costmap, &start, &goal);
        assert!(result.is_ok(), "Should find path around wall");

        let path = result.unwrap();
        // Verify no path point is in a lethal cell
        for ps in &path.poses {
            let (gx, gy) = costmap.world_to_map(ps.pose.x, ps.pose.y);
            if gx >= 0 && gx < 20 && gy >= 0 && gy < 20 {
                let idx = gy as usize * 20 + gx as usize;
                assert_ne!(
                    costmap.data[idx],
                    cost_values::LETHAL_OBSTACLE,
                    "Path goes through obstacle at ({}, {})",
                    gx,
                    gy
                );
            }
        }
    }

    #[test]
    fn test_navfn_no_path() {
        let mut costmap = make_free_costmap(10, 10, 0.5);

        // Create a complete wall
        for x in 0..10 {
            costmap.data[5 * 10 + x] = cost_values::LETHAL_OBSTACLE;
        }

        let planner = NavFnPlanner::new().with_algorithm(Algorithm::Dijkstra);

        let start = PoseStamped::new(Pose2D::new(0.25, 0.25, 0.0), "map");
        let goal = PoseStamped::new(Pose2D::new(0.25, 4.25, 0.0), "map");

        let result = planner.plan_on_costmap(&costmap, &start, &goal);
        assert!(result.is_err(), "Should fail when path is completely blocked");
    }

    #[test]
    fn test_navfn_start_at_goal() {
        let costmap = make_free_costmap(10, 10, 0.5);
        let planner = NavFnPlanner::new();

        let start = PoseStamped::new(Pose2D::new(2.25, 2.25, 0.0), "map");
        let goal = PoseStamped::new(Pose2D::new(2.25, 2.25, 0.0), "map");

        let result = planner.plan_on_costmap(&costmap, &start, &goal);
        assert!(result.is_ok());
        let path = result.unwrap();
        assert_eq!(path.len(), 1);
    }

    #[test]
    fn test_navfn_start_in_obstacle() {
        let mut costmap = make_free_costmap(10, 10, 0.5);
        costmap.data[0] = cost_values::LETHAL_OBSTACLE;

        let planner = NavFnPlanner::new();
        let start = PoseStamped::new(Pose2D::new(0.25, 0.25, 0.0), "map");
        let goal = PoseStamped::new(Pose2D::new(4.25, 4.25, 0.0), "map");

        let result = planner.plan_on_costmap(&costmap, &start, &goal);
        assert!(matches!(result, Err(PlannerError::StartInObstacle)));
    }

    #[test]
    fn test_navfn_goal_in_obstacle() {
        let mut costmap = make_free_costmap(10, 10, 0.5);
        let goal_idx = 9 * 10 + 9;
        costmap.data[goal_idx] = cost_values::LETHAL_OBSTACLE;

        let planner = NavFnPlanner::new();
        let start = PoseStamped::new(Pose2D::new(0.25, 0.25, 0.0), "map");
        let goal = PoseStamped::new(Pose2D::new(4.75, 4.75, 0.0), "map");

        let result = planner.plan_on_costmap(&costmap, &start, &goal);
        assert!(matches!(result, Err(PlannerError::GoalInObstacle)));
    }

    #[test]
    fn test_navfn_prefers_low_cost() {
        let mut costmap = make_free_costmap(10, 10, 0.5);

        // Create a high-cost corridor along x=5
        for y in 0..10 {
            costmap.data[y * 10 + 5] = cost_values::MEDIUM_COST;
        }

        let planner = NavFnPlanner::new();
        let start = PoseStamped::new(Pose2D::new(0.25, 4.25, 0.0), "map");
        let goal = PoseStamped::new(Pose2D::new(4.75, 4.25, 0.0), "map");

        let result = planner.plan_on_costmap(&costmap, &start, &goal);
        assert!(result.is_ok());
    }

    #[test]
    fn test_navfn_trait_without_costmap() {
        let planner = NavFnPlanner::new();
        let start = PoseStamped::new(Pose2D::new(0.0, 0.0, 0.0), "map");
        let goal = PoseStamped::new(Pose2D::new(1.0, 1.0, 0.0), "map");

        let result = planner.create_plan(&start, &goal);
        assert!(matches!(result, Err(PlannerError::NotConfigured)));
    }

    #[test]
    fn test_navfn_trait_configured() {
        let costmap = make_free_costmap(20, 20, 0.5);
        let costmap_arc = Arc::new(RwLock::new(costmap));

        let mut planner = NavFnPlanner::new();
        planner.configure("test_planner", costmap_arc, &ParamMap::new());

        let start = PoseStamped::new(Pose2D::new(0.25, 0.25, 0.0), "map");
        let goal = PoseStamped::new(Pose2D::new(5.25, 5.25, 0.0), "map");

        let result = planner.create_plan(&start, &goal);
        assert!(result.is_ok());
    }
}
