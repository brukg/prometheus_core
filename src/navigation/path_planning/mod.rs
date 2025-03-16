use crate::navigation::planner::PathPlanner;
use std::error::Error;

pub mod cubic_spline_planner;

pub struct CubicSplinePlanner {
    resolution: f64, // Path resolution (ds parameter)
}

impl CubicSplinePlanner {
    pub fn new(resolution: f64) -> Self {
        CubicSplinePlanner { resolution }
    }
}

// Implement the imported PathPlanner trait
impl PathPlanner for CubicSplinePlanner {
    fn plan_path(
        &self,
        start: (f64, f64),
        goal: (f64, f64),
    ) -> Result<Vec<(f64, f64)>, Box<dyn Error>> {
        // Create x and y vectors for the spline
        let x = vec![start.0, goal.0];
        let y = vec![start.1, goal.1];

        // If the distance is large, add intermediate points
        let distance = ((goal.0 - start.0).powi(2) + (goal.1 - start.1).powi(2)).sqrt();
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
