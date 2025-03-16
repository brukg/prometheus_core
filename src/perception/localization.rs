//! Localization module

/// A localizer for the robot
pub struct Localizer {
    // Localizer state
    pose: (f64, f64, f64),
}

impl Localizer {
    /// Create a new localizer
    pub fn new() -> Self {
        Localizer {
            pose: (0.0, 0.0, 0.0),
        }
    }
    
    /// Update the pose estimate
    pub fn update(&mut self, odom_delta: (f64, f64, f64)) {
        // Simple odometry update
        self.pose.0 += odom_delta.0;
        self.pose.1 += odom_delta.1;
        self.pose.2 += odom_delta.2;
        
        // Normalize angle to [-pi, pi]
        if self.pose.2 > std::f64::consts::PI {
            self.pose.2 -= 2.0 * std::f64::consts::PI;
        } else if self.pose.2 < -std::f64::consts::PI {
            self.pose.2 += 2.0 * std::f64::consts::PI;
        }
    }
    
    /// Get the current pose estimate
    pub fn get_pose(&self) -> (f64, f64, f64) {
        self.pose
    }
} 