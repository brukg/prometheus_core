//! Core navigation types for the Nav2 Rust port.
//!
//! These types replace raw tuples `(f64, f64)` and `(f64, f64, f64)` used throughout the
//! original codebase, providing proper semantic types that match Nav2 message conventions.

use std::collections::HashMap;
use std::time::Duration;

/// A 2D pose with position and heading.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose2D {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl Pose2D {
    pub fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self { x, y, yaw }
    }

    /// Euclidean distance to another pose (ignores heading).
    pub fn distance_to(&self, other: &Pose2D) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Normalize yaw to [-pi, pi].
    pub fn normalize_yaw(&mut self) {
        self.yaw = normalize_angle(self.yaw);
    }
}

impl Default for Pose2D {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
        }
    }
}

impl From<(f64, f64, f64)> for Pose2D {
    fn from(tuple: (f64, f64, f64)) -> Self {
        Self {
            x: tuple.0,
            y: tuple.1,
            yaw: tuple.2,
        }
    }
}

impl From<Pose2D> for (f64, f64, f64) {
    fn from(pose: Pose2D) -> Self {
        (pose.x, pose.y, pose.yaw)
    }
}

impl From<(f64, f64)> for Pose2D {
    fn from(tuple: (f64, f64)) -> Self {
        Self {
            x: tuple.0,
            y: tuple.1,
            yaw: 0.0,
        }
    }
}

/// A stamped pose with frame and timestamp information.
#[derive(Debug, Clone, PartialEq)]
pub struct PoseStamped {
    pub pose: Pose2D,
    pub frame_id: String,
    pub stamp: Duration,
}

impl PoseStamped {
    pub fn new(pose: Pose2D, frame_id: &str) -> Self {
        Self {
            pose,
            frame_id: frame_id.to_string(),
            stamp: Duration::ZERO,
        }
    }

    pub fn with_stamp(mut self, stamp: Duration) -> Self {
        self.stamp = stamp;
        self
    }
}

impl Default for PoseStamped {
    fn default() -> Self {
        Self {
            pose: Pose2D::default(),
            frame_id: "map".to_string(),
            stamp: Duration::ZERO,
        }
    }
}

/// A velocity command (linear and angular components).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Twist {
    pub linear_x: f64,
    pub linear_y: f64,
    pub angular_z: f64,
}

impl Twist {
    pub fn new(linear_x: f64, angular_z: f64) -> Self {
        Self {
            linear_x,
            linear_y: 0.0,
            angular_z,
        }
    }

    pub fn zero() -> Self {
        Self {
            linear_x: 0.0,
            linear_y: 0.0,
            angular_z: 0.0,
        }
    }
}

impl Default for Twist {
    fn default() -> Self {
        Self::zero()
    }
}

/// A sequence of stamped poses forming a path.
#[derive(Debug, Clone)]
pub struct Path {
    pub poses: Vec<PoseStamped>,
    pub frame_id: String,
}

impl Path {
    pub fn new(frame_id: &str) -> Self {
        Self {
            poses: Vec::new(),
            frame_id: frame_id.to_string(),
        }
    }

    pub fn from_poses(poses: Vec<PoseStamped>, frame_id: &str) -> Self {
        Self {
            poses,
            frame_id: frame_id.to_string(),
        }
    }

    /// Create a Path from a vector of (x, y) tuples (for backwards compatibility).
    pub fn from_xy_vec(points: &[(f64, f64)], frame_id: &str) -> Self {
        let poses = points
            .iter()
            .map(|(x, y)| PoseStamped::new(Pose2D::new(*x, *y, 0.0), frame_id))
            .collect();
        Self {
            poses,
            frame_id: frame_id.to_string(),
        }
    }

    /// Convert to a vector of (x, y) tuples (for backwards compatibility).
    pub fn to_xy_vec(&self) -> Vec<(f64, f64)> {
        self.poses.iter().map(|ps| (ps.pose.x, ps.pose.y)).collect()
    }

    pub fn len(&self) -> usize {
        self.poses.len()
    }

    pub fn is_empty(&self) -> bool {
        self.poses.is_empty()
    }

    /// Total path length (sum of euclidean segment distances).
    pub fn total_length(&self) -> f64 {
        if self.poses.len() < 2 {
            return 0.0;
        }
        self.poses
            .windows(2)
            .map(|w| w[0].pose.distance_to(&w[1].pose))
            .sum()
    }
}

impl Default for Path {
    fn default() -> Self {
        Self::new("map")
    }
}

/// Parameter value types for plugin configuration (matches ROS2 parameter types).
#[derive(Debug, Clone)]
pub enum ParamValue {
    Bool(bool),
    Int(i64),
    Float(f64),
    String(String),
}

impl ParamValue {
    pub fn as_bool(&self) -> Option<bool> {
        match self {
            ParamValue::Bool(v) => Some(*v),
            _ => None,
        }
    }

    pub fn as_int(&self) -> Option<i64> {
        match self {
            ParamValue::Int(v) => Some(*v),
            _ => None,
        }
    }

    pub fn as_float(&self) -> Option<f64> {
        match self {
            ParamValue::Float(v) => Some(*v),
            ParamValue::Int(v) => Some(*v as f64),
            _ => None,
        }
    }

    pub fn as_str(&self) -> Option<&str> {
        match self {
            ParamValue::String(v) => Some(v),
            _ => None,
        }
    }
}

/// A map of string keys to parameter values.
pub type ParamMap = HashMap<String, ParamValue>;

/// Normalize an angle to [-pi, pi].
pub fn normalize_angle(angle: f64) -> f64 {
    let mut a = angle;
    while a > std::f64::consts::PI {
        a -= 2.0 * std::f64::consts::PI;
    }
    while a < -std::f64::consts::PI {
        a += 2.0 * std::f64::consts::PI;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_pose2d_distance() {
        let a = Pose2D::new(0.0, 0.0, 0.0);
        let b = Pose2D::new(3.0, 4.0, 0.0);
        assert!((a.distance_to(&b) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_pose2d_from_tuple() {
        let p: Pose2D = (1.0, 2.0, 0.5).into();
        assert_eq!(p.x, 1.0);
        assert_eq!(p.y, 2.0);
        assert_eq!(p.yaw, 0.5);
    }

    #[test]
    fn test_twist_zero() {
        let t = Twist::zero();
        assert_eq!(t.linear_x, 0.0);
        assert_eq!(t.angular_z, 0.0);
    }

    #[test]
    fn test_path_from_xy_vec() {
        let points = vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)];
        let path = Path::from_xy_vec(&points, "map");
        assert_eq!(path.len(), 3);
        assert!((path.total_length() - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_path_roundtrip() {
        let points = vec![(1.0, 2.0), (3.0, 4.0)];
        let path = Path::from_xy_vec(&points, "map");
        let roundtrip = path.to_xy_vec();
        assert_eq!(roundtrip, points);
    }

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(0.0) - 0.0).abs() < 1e-10);
        assert!((normalize_angle(PI) - PI).abs() < 1e-10);
        assert!((normalize_angle(-PI) - (-PI)).abs() < 1e-10);
        assert!((normalize_angle(3.0 * PI) - PI).abs() < 1e-10);
        assert!((normalize_angle(-3.0 * PI) - (-PI)).abs() < 1e-10);
    }

    #[test]
    fn test_param_value_conversions() {
        let v = ParamValue::Float(3.14);
        assert!((v.as_float().unwrap() - 3.14).abs() < 1e-10);

        let v = ParamValue::Int(42);
        assert_eq!(v.as_int().unwrap(), 42);
        assert!((v.as_float().unwrap() - 42.0).abs() < 1e-10);

        let v = ParamValue::Bool(true);
        assert_eq!(v.as_bool().unwrap(), true);

        let v = ParamValue::String("test".to_string());
        assert_eq!(v.as_str().unwrap(), "test");
    }
}
