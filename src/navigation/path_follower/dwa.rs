//! Dynamic Window Approach (DWA) path follower implementation
//! 
//! Based on the algorithm from:
//! https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/DynamicWindowApproach/dynamic_window_approach.py

use std::collections::HashMap;
use std::f64::consts::PI;

use super::PathFollower;

/// Dynamic Window Approach path follower
///
/// This implementation uses the Dynamic Window Approach algorithm to compute
/// velocity commands that follow a path while avoiding obstacles.
#[derive(Debug)]
pub struct DwaPathFollower {
    // Configuration parameters
    max_linear_velocity: f64,
    max_angular_velocity: f64,
    linear_acceleration: f64,
    angular_acceleration: f64,
    velocity_resolution: f64,
    simulation_time: f64,
    goal_weight: f64,
    obstacle_weight: f64,
    heading_weight: f64,
    clearance_weight: f64,
    velocity_weight: f64,
}

impl PathFollower for DwaPathFollower {
    fn new() -> Self {
        DwaPathFollower {
            max_linear_velocity: 0.5,
            max_angular_velocity: 1.0,
            linear_acceleration: 0.2,
            angular_acceleration: 0.4,
            velocity_resolution: 0.01,
            simulation_time: 3.0,
            goal_weight: 0.5,
            obstacle_weight: 0.8,
            heading_weight: 0.8,
            clearance_weight: 0.2,
            velocity_weight: 0.2,
        }
    }

    fn compute_velocity(&self, current_pose: (f64, f64, f64), path: &[(f64, f64)]) -> (f64, f64) {
        if path.is_empty() {
            return (0.0, 0.0);
        }

        // Find the closest point on the path
        let (closest_idx, _) = self.find_closest_point(current_pose, path);
        
        // Get the goal point (a few points ahead on the path)
        let lookahead = 5;
        let goal_idx = (closest_idx + lookahead).min(path.len() - 1);
        let goal = path[goal_idx];
        
        // Compute the dynamic window
        let dw = self.compute_dynamic_window(current_pose);
        
        // Search for the best velocity in the dynamic window
        let (best_v, best_w) = self.search_best_velocity(current_pose, goal, dw);
        
        (best_v, best_w)
    }

    fn name(&self) -> &str {
        "DWA"
    }

    fn configure(&mut self, params: &HashMap<String, f64>) -> Result<(), String> {
        if let Some(&max_linear_velocity) = params.get("max_linear_velocity") {
            self.max_linear_velocity = max_linear_velocity;
        }
        if let Some(&max_angular_velocity) = params.get("max_angular_velocity") {
            self.max_angular_velocity = max_angular_velocity;
        }
        if let Some(&linear_acceleration) = params.get("linear_acceleration") {
            self.linear_acceleration = linear_acceleration;
        }
        if let Some(&angular_acceleration) = params.get("angular_acceleration") {
            self.angular_acceleration = angular_acceleration;
        }
        if let Some(&velocity_resolution) = params.get("velocity_resolution") {
            self.velocity_resolution = velocity_resolution;
        }
        if let Some(&simulation_time) = params.get("simulation_time") {
            self.simulation_time = simulation_time;
        }
        if let Some(&goal_weight) = params.get("goal_weight") {
            self.goal_weight = goal_weight;
        }
        if let Some(&obstacle_weight) = params.get("obstacle_weight") {
            self.obstacle_weight = obstacle_weight;
        }
        if let Some(&heading_weight) = params.get("heading_weight") {
            self.heading_weight = heading_weight;
        }
        if let Some(&clearance_weight) = params.get("clearance_weight") {
            self.clearance_weight = clearance_weight;
        }
        if let Some(&velocity_weight) = params.get("velocity_weight") {
            self.velocity_weight = velocity_weight;
        }
        Ok(())
    }
}

impl DwaPathFollower {
    /// Find the closest point on the path to the current pose
    fn find_closest_point(&self, current_pose: (f64, f64, f64), path: &[(f64, f64)]) -> (usize, f64) {
        let mut closest_idx = 0;
        let mut min_dist = f64::MAX;
        
        for (i, point) in path.iter().enumerate() {
            let dx = point.0 - current_pose.0;
            let dy = point.1 - current_pose.1;
            let dist = (dx * dx + dy * dy).sqrt();
            
            if dist < min_dist {
                min_dist = dist;
                closest_idx = i;
            }
        }
        
        (closest_idx, min_dist)
    }
    
    /// Compute the dynamic window based on current velocity and acceleration limits
    fn compute_dynamic_window(&self, current_pose: (f64, f64, f64)) -> ((f64, f64), (f64, f64)) {
        // Assume current velocity is (0, 0) for simplicity
        // In a real implementation, we would track the current velocity
        let current_v = 0.0;
        let current_w = 0.0;
        
        // Velocity limits from robot specifications
        let v_min = 0.0;
        let v_max = self.max_linear_velocity;
        let w_min = -self.max_angular_velocity;
        let w_max = self.max_angular_velocity;
        
        // Dynamic window from motion model
        let v_min_dw = (current_v - self.linear_acceleration).max(v_min);
        let v_max_dw = (current_v + self.linear_acceleration).min(v_max);
        let w_min_dw = (current_w - self.angular_acceleration).max(w_min);
        let w_max_dw = (current_w + self.angular_acceleration).min(w_max);
        
        ((v_min_dw, v_max_dw), (w_min_dw, w_max_dw))
    }
    
    /// Search for the best velocity in the dynamic window
    fn search_best_velocity(&self, current_pose: (f64, f64, f64), goal: (f64, f64), dw: ((f64, f64), (f64, f64))) -> (f64, f64) {
        let ((v_min, v_max), (w_min, w_max)) = dw;
        
        let mut best_score = -f64::MAX;
        let mut best_v = 0.0;
        let mut best_w = 0.0;
        
        // Discretize the velocity space and evaluate each velocity
        let mut v = v_min;
        while v <= v_max {
            let mut w = w_min;
            while w <= w_max {
                // Predict trajectory for this velocity
                let trajectory = self.predict_trajectory(current_pose, v, w);
                
                // Evaluate trajectory
                let heading_score = self.heading_cost(trajectory.last().copied().unwrap_or(current_pose), goal);
                let dist_score = self.distance_cost(trajectory.last().copied().unwrap_or(current_pose), goal);
                let velocity_score = self.velocity_cost(v);
                
                // No obstacle cost for now (would require a map)
                let obstacle_score = 1.0;
                let clearance_score = 1.0;
                
                // Compute final score (higher is better)
                let score = self.heading_weight * heading_score +
                            self.goal_weight * dist_score +
                            self.velocity_weight * velocity_score +
                            self.obstacle_weight * obstacle_score +
                            self.clearance_weight * clearance_score;
                
                if score > best_score {
                    best_score = score;
                    best_v = v;
                    best_w = w;
                }
                
                w += self.velocity_resolution;
            }
            v += self.velocity_resolution;
        }
        
        (best_v, best_w)
    }
    
    /// Predict the trajectory for a given velocity
    fn predict_trajectory(&self, current_pose: (f64, f64, f64), v: f64, w: f64) -> Vec<(f64, f64, f64)> {
        let dt = 0.1; // Time step for simulation
        let steps = (self.simulation_time / dt) as usize;
        
        let mut trajectory = Vec::with_capacity(steps);
        let mut pose = current_pose;
        
        for _ in 0..steps {
            // Simple motion model
            let next_x = pose.0 + v * pose.2.cos() * dt;
            let next_y = pose.1 + v * pose.2.sin() * dt;
            let next_theta = pose.2 + w * dt;
            
            pose = (next_x, next_y, next_theta);
            trajectory.push(pose);
        }
        
        trajectory
    }
    
    /// Compute heading cost (how well the robot is aligned with the goal)
    fn heading_cost(&self, pose: (f64, f64, f64), goal: (f64, f64)) -> f64 {
        let goal_heading = (goal.1 - pose.1).atan2(goal.0 - pose.0);
        let heading_error = (goal_heading - pose.2).abs();
        let normalized_error = if heading_error > PI { 2.0 * PI - heading_error } else { heading_error };
        
        1.0 - normalized_error / PI
    }
    
    /// Compute distance cost (how close the robot gets to the goal)
    fn distance_cost(&self, pose: (f64, f64, f64), goal: (f64, f64)) -> f64 {
        let dx = goal.0 - pose.0;
        let dy = goal.1 - pose.1;
        let distance = (dx * dx + dy * dy).sqrt();
        
        // Normalize by a reasonable maximum distance
        let max_dist = 5.0;
        1.0 - (distance / max_dist).min(1.0)
    }
    
    /// Compute velocity cost (prefer higher velocities)
    fn velocity_cost(&self, v: f64) -> f64 {
        v / self.max_linear_velocity
    }
}