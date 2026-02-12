//! Recovery behavior implementations: Spin, BackUp, Wait.
//!
//! These are simple recovery behaviors matching Nav2's behavior server plugins.

use crate::navigation::traits::{Behavior, BehaviorStatus};
use crate::navigation::types::*;
use std::time::{Duration, Instant};

/// Spin behavior: rotate in place by a target angle.
pub struct SpinBehavior {
    name: String,
    target_yaw: f64,
    angular_velocity: f64,
    time_allowance: Duration,
    accumulated_yaw: f64,
    start_time: Option<Instant>,
}

impl SpinBehavior {
    pub fn new(target_yaw: f64) -> Self {
        Self {
            name: "spin".to_string(),
            target_yaw,
            angular_velocity: 1.0,
            time_allowance: Duration::from_secs(10),
            accumulated_yaw: 0.0,
            start_time: None,
        }
    }

    /// Get the velocity command for this tick.
    pub fn get_velocity(&self) -> Twist {
        let direction = if self.target_yaw >= 0.0 { 1.0 } else { -1.0 };
        Twist::new(0.0, direction * self.angular_velocity)
    }
}

impl Behavior for SpinBehavior {
    fn configure(&mut self, name: &str, params: &ParamMap) {
        self.name = name.to_string();
        if let Some(pv) = params.get("target_yaw") {
            if let Some(v) = pv.as_float() {
                self.target_yaw = v;
            }
        }
        if let Some(pv) = params.get("angular_velocity") {
            if let Some(v) = pv.as_float() {
                self.angular_velocity = v;
            }
        }
        if let Some(pv) = params.get("time_allowance") {
            if let Some(v) = pv.as_float() {
                self.time_allowance = Duration::from_secs_f64(v);
            }
        }
    }

    fn execute(&mut self) -> BehaviorStatus {
        let now = Instant::now();

        if self.start_time.is_none() {
            self.start_time = Some(now);
            self.accumulated_yaw = 0.0;
        }

        // Check time allowance
        if now.duration_since(self.start_time.unwrap()) > self.time_allowance {
            self.start_time = None;
            return BehaviorStatus::Failed;
        }

        // Simulate rotation (in real system, would check odometry)
        let dt = 0.1; // Assume 10Hz tick rate
        self.accumulated_yaw += self.angular_velocity * dt;

        if self.accumulated_yaw.abs() >= self.target_yaw.abs() {
            self.start_time = None;
            BehaviorStatus::Succeeded
        } else {
            BehaviorStatus::Running
        }
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn cleanup(&mut self) {
        self.start_time = None;
        self.accumulated_yaw = 0.0;
    }
}

/// BackUp behavior: drive backward by a target distance.
pub struct BackUpBehavior {
    name: String,
    target_distance: f64,
    speed: f64,
    time_allowance: Duration,
    accumulated_distance: f64,
    start_time: Option<Instant>,
}

impl BackUpBehavior {
    pub fn new(target_distance: f64) -> Self {
        Self {
            name: "backup".to_string(),
            target_distance,
            speed: 0.1,
            time_allowance: Duration::from_secs(15),
            accumulated_distance: 0.0,
            start_time: None,
        }
    }

    /// Get the velocity command for this tick.
    pub fn get_velocity(&self) -> Twist {
        Twist::new(-self.speed, 0.0)
    }
}

impl Behavior for BackUpBehavior {
    fn configure(&mut self, name: &str, params: &ParamMap) {
        self.name = name.to_string();
        if let Some(pv) = params.get("target_distance") {
            if let Some(v) = pv.as_float() {
                self.target_distance = v;
            }
        }
        if let Some(pv) = params.get("speed") {
            if let Some(v) = pv.as_float() {
                self.speed = v;
            }
        }
        if let Some(pv) = params.get("time_allowance") {
            if let Some(v) = pv.as_float() {
                self.time_allowance = Duration::from_secs_f64(v);
            }
        }
    }

    fn execute(&mut self) -> BehaviorStatus {
        let now = Instant::now();

        if self.start_time.is_none() {
            self.start_time = Some(now);
            self.accumulated_distance = 0.0;
        }

        // Check time allowance
        if now.duration_since(self.start_time.unwrap()) > self.time_allowance {
            self.start_time = None;
            return BehaviorStatus::Failed;
        }

        // Simulate backward movement
        let dt = 0.1;
        self.accumulated_distance += self.speed * dt;

        if self.accumulated_distance >= self.target_distance {
            self.start_time = None;
            BehaviorStatus::Succeeded
        } else {
            BehaviorStatus::Running
        }
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn cleanup(&mut self) {
        self.start_time = None;
        self.accumulated_distance = 0.0;
    }
}

/// Wait behavior: do nothing for a specified duration.
pub struct WaitBehavior {
    name: String,
    wait_duration: Duration,
    start_time: Option<Instant>,
}

impl WaitBehavior {
    pub fn new(wait_duration: Duration) -> Self {
        Self {
            name: "wait".to_string(),
            wait_duration,
            start_time: None,
        }
    }
}

impl Behavior for WaitBehavior {
    fn configure(&mut self, name: &str, params: &ParamMap) {
        self.name = name.to_string();
        if let Some(pv) = params.get("wait_duration") {
            if let Some(v) = pv.as_float() {
                self.wait_duration = Duration::from_secs_f64(v);
            }
        }
    }

    fn execute(&mut self) -> BehaviorStatus {
        let now = Instant::now();

        if self.start_time.is_none() {
            self.start_time = Some(now);
        }

        if now.duration_since(self.start_time.unwrap()) >= self.wait_duration {
            self.start_time = None;
            BehaviorStatus::Succeeded
        } else {
            BehaviorStatus::Running
        }
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn cleanup(&mut self) {
        self.start_time = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_spin_behavior() {
        let mut spin = SpinBehavior::new(0.5);
        spin.angular_velocity = 10.0; // Fast for testing

        // Should succeed after enough ticks
        let mut status = BehaviorStatus::Running;
        for _ in 0..100 {
            status = spin.execute();
            if status == BehaviorStatus::Succeeded {
                break;
            }
        }
        assert_eq!(status, BehaviorStatus::Succeeded);
    }

    #[test]
    fn test_spin_velocity() {
        let spin = SpinBehavior::new(1.0);
        let vel = spin.get_velocity();
        assert_eq!(vel.linear_x, 0.0);
        assert!(vel.angular_z > 0.0);
    }

    #[test]
    fn test_backup_behavior() {
        let mut backup = BackUpBehavior::new(0.3);
        backup.speed = 10.0; // Fast for testing

        let mut status = BehaviorStatus::Running;
        for _ in 0..100 {
            status = backup.execute();
            if status == BehaviorStatus::Succeeded {
                break;
            }
        }
        assert_eq!(status, BehaviorStatus::Succeeded);
    }

    #[test]
    fn test_backup_velocity() {
        let backup = BackUpBehavior::new(0.3);
        let vel = backup.get_velocity();
        assert!(vel.linear_x < 0.0, "Backup should have negative linear vel");
        assert_eq!(vel.angular_z, 0.0);
    }

    #[test]
    fn test_wait_behavior() {
        let mut wait = WaitBehavior::new(Duration::from_millis(50));

        // First tick should be Running
        assert_eq!(wait.execute(), BehaviorStatus::Running);

        // Wait for the duration to pass
        std::thread::sleep(Duration::from_millis(60));

        // Should now succeed
        assert_eq!(wait.execute(), BehaviorStatus::Succeeded);
    }

    #[test]
    fn test_behavior_cleanup() {
        let mut spin = SpinBehavior::new(1.0);
        spin.execute();
        assert!(spin.start_time.is_some());
        spin.cleanup();
        assert!(spin.start_time.is_none());
        assert_eq!(spin.accumulated_yaw, 0.0);
    }
}
