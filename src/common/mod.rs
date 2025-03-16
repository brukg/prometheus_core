//! Common utilities and types for Prometheus robot

/// Common types and utilities used across the codebase
pub mod types {
    /// A 2D point
    pub type Point2D = (f64, f64);

    /// A 3D pose (x, y, theta)
    pub type Pose2D = (f64, f64, f64);
}
