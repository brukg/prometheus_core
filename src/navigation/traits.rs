//! Nav2-equivalent traits for the Rust port.
//!
//! These traits define the plugin interfaces for planners, controllers, costmap layers,
//! smoothers, and recovery behaviors. Each matches the corresponding Nav2 C++ interface.

use crate::navigation::costmap::Costmap;
use crate::navigation::types::*;
use std::sync::{Arc, RwLock};
use std::time::Duration;

/// Error types for planner operations.
#[derive(Debug, thiserror::Error)]
pub enum PlannerError {
    #[error("no valid path found from start to goal")]
    NoPathFound,
    #[error("start pose is in lethal obstacle")]
    StartInObstacle,
    #[error("goal pose is in lethal obstacle")]
    GoalInObstacle,
    #[error("planner timed out after {0:?}")]
    Timeout(Duration),
    #[error("planner not configured")]
    NotConfigured,
    #[error("costmap error: {0}")]
    CostmapError(String),
    #[error("{0}")]
    Other(String),
}

/// Error types for controller operations.
#[derive(Debug, thiserror::Error)]
pub enum ControllerError {
    #[error("no valid path set")]
    NoPath,
    #[error("failed to compute velocity: {0}")]
    ComputeError(String),
    #[error("controller not configured")]
    NotConfigured,
    #[error("path blocked by obstacle")]
    PathBlocked,
    #[error("{0}")]
    Other(String),
}

/// Error types for smoother operations.
#[derive(Debug, thiserror::Error)]
pub enum SmootherError {
    #[error("smoothing timed out after {0:?}")]
    Timeout(Duration),
    #[error("smoother not configured")]
    NotConfigured,
    #[error("path collides with obstacle after smoothing")]
    CollisionAfterSmoothing,
    #[error("{0}")]
    Other(String),
}

/// Status of a recovery behavior execution.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BehaviorStatus {
    Succeeded,
    Failed,
    Running,
}

/// Global planner interface (Nav2 `GlobalPlanner` equivalent).
///
/// Computes a global path from start to goal using a costmap.
pub trait GlobalPlanner: Send + Sync {
    /// Configure the planner with a name, costmap, and parameters.
    fn configure(&mut self, name: &str, costmap: Arc<RwLock<Costmap>>, params: &ParamMap);

    /// Create a plan from start to goal.
    fn create_plan(
        &self,
        start: &PoseStamped,
        goal: &PoseStamped,
    ) -> Result<Path, PlannerError>;

    /// Get the planner's name.
    fn name(&self) -> &str;

    /// Clean up resources.
    fn cleanup(&mut self);
}

/// Local controller interface (Nav2 `Controller` equivalent).
///
/// Computes velocity commands to follow a path while avoiding local obstacles.
pub trait Controller: Send + Sync {
    /// Configure the controller with a name, costmap, and parameters.
    fn configure(&mut self, name: &str, costmap: Arc<RwLock<Costmap>>, params: &ParamMap);

    /// Set the global plan for the controller to follow.
    fn set_plan(&mut self, path: &Path);

    /// Compute velocity commands given current pose and velocity.
    fn compute_velocity_commands(
        &self,
        pose: &PoseStamped,
        velocity: &Twist,
    ) -> Result<Twist, ControllerError>;

    /// Check if the goal has been reached within tolerance.
    fn is_goal_reached(
        &self,
        pose: &PoseStamped,
        goal: &PoseStamped,
        tolerance: f64,
    ) -> bool {
        pose.pose.distance_to(&goal.pose) <= tolerance
    }

    /// Get the controller's name.
    fn name(&self) -> &str;

    /// Clean up resources.
    fn cleanup(&mut self);
}

/// Costmap layer interface (Nav2 `CostmapLayer` equivalent).
///
/// Layers update bounds and costs on a master costmap grid.
pub trait CostmapLayer: Send + Sync {
    /// Configure the layer with a name and parameters.
    fn configure(&mut self, name: &str, params: &ParamMap);

    /// Compute the bounds that this layer will update.
    fn update_bounds(
        &mut self,
        robot_x: f64,
        robot_y: f64,
        robot_yaw: f64,
        min_x: &mut f64,
        min_y: &mut f64,
        max_x: &mut f64,
        max_y: &mut f64,
    );

    /// Apply cost updates to the master grid within the given bounds.
    fn update_costs(
        &self,
        master_grid: &mut Costmap,
        min_x: f64,
        min_y: f64,
        max_x: f64,
        max_y: f64,
    );

    /// Get the layer's name.
    fn name(&self) -> &str;

    /// Reset the layer to its initial state.
    fn reset(&mut self);
}

/// Path smoother interface (Nav2 `Smoother` equivalent).
pub trait Smoother: Send + Sync {
    /// Configure the smoother with a name, costmap, and parameters.
    fn configure(&mut self, name: &str, costmap: Arc<RwLock<Costmap>>, params: &ParamMap);

    /// Smooth a path within a time budget.
    fn smooth(&self, path: &Path, max_duration: Duration) -> Result<Path, SmootherError>;

    /// Get the smoother's name.
    fn name(&self) -> &str;

    /// Clean up resources.
    fn cleanup(&mut self);
}

/// Recovery behavior interface (Nav2 `Behavior` equivalent).
pub trait Behavior: Send + Sync {
    /// Configure the behavior with a name and parameters.
    fn configure(&mut self, name: &str, params: &ParamMap);

    /// Execute one tick of the behavior. Returns status.
    fn execute(&mut self) -> BehaviorStatus;

    /// Get the behavior's name.
    fn name(&self) -> &str;

    /// Clean up resources.
    fn cleanup(&mut self);
}

/// Plugin metadata trait for the plugin registry.
pub trait Plugin: Send + Sync {
    /// Get the plugin's registered name.
    fn plugin_name(&self) -> &str;

    /// Get the plugin type identifier (e.g., "planner", "controller").
    fn plugin_type(&self) -> &str;
}
