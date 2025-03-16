//! Simple path follower implementation

use std::collections::HashMap;
use super::PathFollower;

/// A simple path follower for the robot
#[derive(Debug)]
pub struct SimplePathFollower {
    // Path follower configuration
    linear_gain: f64,
    angular_gain: f64,
}

impl PathFollower for SimplePathFollower {
    fn new() -> Self {
        SimplePathFollower {
            linear_gain: 0.2,
            angular_gain: 0.5,
        }
    }
    
    fn compute_velocity(&self, current_pose: (f64, f64, f64), path: &[(f64, f64)]) -> (f64, f64) {
        if path.is_empty() {
            return (0.0, 0.0);
        }
        
        // Simple implementation: just head toward the next waypoint
        let target = path[0];
        let dx = target.0 - current_pose.0;
        let dy = target.1 - current_pose.1;
        
        let distance = (dx * dx + dy * dy).sqrt();
        let target_angle = dy.atan2(dx);
        let angle_diff = target_angle - current_pose.2;
        
        // Normalize angle difference to [-pi, pi]
        let angle_diff = if angle_diff > std::f64::consts::PI {
            angle_diff - 2.0 * std::f64::consts::PI
        } else if angle_diff < -std::f64::consts::PI {
            angle_diff + 2.0 * std::f64::consts::PI
        } else {
            angle_diff
        };
        
        // Simple proportional control
        let linear_vel = self.linear_gain * distance.min(1.0);
        let angular_vel = self.angular_gain * angle_diff;
        
        (linear_vel, angular_vel)
    }
    
    fn name(&self) -> &str {
        "SimplePathFollower"
    }
    
    fn configure(&mut self, params: &HashMap<String, f64>) -> Result<(), String> {
        if let Some(&linear_gain) = params.get("linear_gain") {
            self.linear_gain = linear_gain;
        }
        
        if let Some(&angular_gain) = params.get("angular_gain") {
            self.angular_gain = angular_gain;
        }
        
        Ok(())
    }
} 