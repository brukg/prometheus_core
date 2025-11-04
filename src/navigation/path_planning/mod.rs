use crate::navigation::planner::PathPlanner;
use crate::navigation::costmap::{Costmap, CostmapManager, WorldPoint, cost_values};
use std::error::Error;
use std::sync::{Arc, RwLock};

pub mod cubic_spline_planner;

pub struct CubicSplinePlanner {
    resolution: f64, // Path resolution (ds parameter)
}

impl CubicSplinePlanner {
    pub fn new(resolution: f64) -> Self {
        CubicSplinePlanner { resolution }
    }
    
    /// Plan a path with obstacle avoidance using costmap
    pub fn plan_with_costmap(
        &self, 
        start: (f64, f64), 
        goal: (f64, f64),
        costmap_manager: &CostmapManager
    ) -> Result<Vec<(f64, f64)>, Box<dyn Error>> {
        // First try a direct path
        let direct_path = self.plan_direct_path(start, goal);
        
        // Check if the direct path is valid
        if let Ok(is_valid) = costmap_manager.is_path_valid(&direct_path) {
            if is_valid {
                return Ok(direct_path);
            }
        }
        
        // If direct path is not valid, try path with intermediate points
        println!("Direct path has obstacles, trying path with intermediate points");
        let path_with_intermediates = self.plan_path_with_intermediates(start, goal);
        
        // Check if path with intermediates is valid
        if let Ok(is_valid) = costmap_manager.is_path_valid(&path_with_intermediates) {
            if is_valid {
                return Ok(path_with_intermediates);
            }
        }
        
        // If still not valid, we would implement more advanced planning here
        // For now, just return the direct path and let higher-level logic handle it
        println!("Could not find a valid path with simple planning, need more advanced planner");
        Ok(direct_path)
    }
    
    /// Plan a direct path between two points
    fn plan_direct_path(&self, start: (f64, f64), goal: (f64, f64)) -> Vec<(f64, f64)> {
        // For short distances, use simple linear path
        if self.is_short_distance(start, goal) {
            return vec![start, goal];
        }
        
        // Create x and y vectors for the spline
        let x = vec![start.0, goal.0];
        let y = vec![start.1, goal.1];
        
        // Generate smooth path with cubic spline
        let (path, _, _, _) = cubic_spline_planner::calc_spline_course(x, y, self.resolution);
        path
    }
    
    /// Plan a path with intermediate points to avoid obstacles
    fn plan_path_with_intermediates(&self, start: (f64, f64), goal: (f64, f64)) -> Vec<(f64, f64)> {
        // Calculate the direct distance
        let distance = ((goal.0 - start.0).powi(2) + (goal.1 - start.1).powi(2)).sqrt();
        
        // If the distance is large, add intermediate points
        if distance > 2.0 {
            // Add several intermediate points for smoother path
            let num_points = (distance / 2.0).ceil() as usize;
            let mut x = Vec::with_capacity(num_points + 2);
            let mut y = Vec::with_capacity(num_points + 2);
            
            x.push(start.0);
            y.push(start.1);
            
            for i in 1..=num_points {
                let t = i as f64 / (num_points as f64 + 1.0);
                
                // Add some lateral offset for potential obstacle avoidance
                // This is a simple heuristic - in a real system, this would be smarter
                let mid_x = start.0 + t * (goal.0 - start.0);
                let mid_y = start.1 + t * (goal.1 - start.1);
                
                // Add a slight offset perpendicular to the path
                let dx = goal.0 - start.0;
                let dy = goal.1 - start.1;
                
                // Perpendicular direction
                let perp_x = -dy;
                let perp_y = dx;
                
                // Normalize
                let length = (perp_x * perp_x + perp_y * perp_y).sqrt();
                let perp_x = perp_x / length;
                let perp_y = perp_y / length;
                
                // Add slight randomization to offset (alternating sides)
                let offset = if i % 2 == 0 { 0.3 } else { -0.3 };
                
                x.push(mid_x + offset * perp_x);
                y.push(mid_y + offset * perp_y);
            }
            
            x.push(goal.0);
            y.push(goal.1);
            
            // Generate smooth path with cubic spline
            let (path, _, _, _) = cubic_spline_planner::calc_spline_course(x, y, self.resolution);
            path
        } else {
            // For short distances, use simple linear path
            vec![start, goal]
        }
    }
    
    /// Check if the distance between points is short
    fn is_short_distance(&self, start: (f64, f64), goal: (f64, f64)) -> bool {
        let distance = ((goal.0 - start.0).powi(2) + (goal.1 - start.1).powi(2)).sqrt();
        distance < 1.0 // Less than 1 meter is considered short
    }
}

// Implement the imported PathPlanner trait
impl PathPlanner for CubicSplinePlanner {
    fn plan_path(
        &self,
        start: (f64, f64),
        goal: (f64, f64),
    ) -> Result<Vec<(f64, f64)>, Box<dyn Error>> {
        // When no costmap is provided, we still provide a reasonable path
        let distance = ((goal.0 - start.0).powi(2) + (goal.1 - start.1).powi(2)).sqrt();
        
        // If the distance is large, add intermediate points
        let path = if distance > 2.0 {
            // Add a midpoint for smoother path
            let mid_x = (start.0 + goal.0) / 2.0;
            let mid_y = (start.1 + goal.1) / 2.0;

            let x = vec![start.0, mid_x, goal.0];
            let y = vec![start.1, mid_y, goal.1];

            // Generate smooth path with cubic spline
            let (path, _, _, _) = cubic_spline_planner::calc_spline_course(x, y, self.resolution);
            path
        } else {
            // For short distances, use simple linear path
            vec![start, goal]
        };

        Ok(path)
    }
}
