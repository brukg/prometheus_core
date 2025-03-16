//! Controllers for the robot

/// A controller for differential drive robots
pub struct DifferentialDriveController {
    // Controller parameters
    linear_gain: f64,
    angular_gain: f64,
}

impl DifferentialDriveController {
    /// Create a new controller
    pub fn new() -> Self {
        DifferentialDriveController {
            linear_gain: 0.5,
            angular_gain: 1.0,
        }
    }
    
    /// Compute control commands
    pub fn compute_velocity(&self, current_pose: (f64, f64, f64), target_pose: (f64, f64, f64)) -> (f64, f64) {
        let dx = target_pose.0 - current_pose.0;
        let dy = target_pose.1 - current_pose.1;
        
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
        
        // Proportional control
        let linear_vel = self.linear_gain * distance.min(1.0);
        let angular_vel = self.angular_gain * angle_diff;
        
        (linear_vel, angular_vel)
    }
} 