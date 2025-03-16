//! Path following module with multiple algorithm implementations

use std::collections::HashMap;
use std::fmt::Debug;

/// Trait for path following algorithms
pub trait PathFollower: Debug + Send + Sync {
    /// Create a new instance with default parameters
    fn new() -> Self
    where
        Self: Sized;

    /// Compute velocity commands to follow a path
    fn compute_velocity(&self, current_pose: (f64, f64, f64), path: &[(f64, f64)]) -> (f64, f64);

    /// Get the name of this path follower
    fn name(&self) -> &str;

    /// Configure the path follower with parameters
    fn configure(&mut self, params: &HashMap<String, f64>) -> Result<(), String>;
}

// Re-export specific implementations
pub mod pure_pursuit;
pub mod simple;
pub mod dwa;
// Add more implementations as needed: pub mod dwa;

// Default implementation
pub use simple::SimplePathFollower as DefaultPathFollower;
