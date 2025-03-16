//! Navigation module for Prometheus robot
pub mod costmap;
pub mod path_follower;
pub mod path_planning;
pub mod planner;

use self::costmap::Costmap;
use self::path_follower::DefaultPathFollower;
use self::path_follower::PathFollower;
use self::path_planning::CubicSplinePlanner;
use self::planner::PathPlanner;
use crate::lifecycle::{LifecycleNode, LifecycleNodeBase, State};
use std::any::Any;
use std::collections::HashMap;

/// Navigation stack for the robot
pub struct NavigationStack {
    base: LifecycleNodeBase,
    planner: Box<dyn PathPlanner>,
    costmap: Costmap,
    path_follower: Box<dyn PathFollower>,
    current_path: Vec<(f64, f64)>,
}

impl NavigationStack {
    /// Create a new navigation stack
    pub fn new() -> Self {
        NavigationStack {
            base: LifecycleNodeBase::new("navigation_stack"),
            planner: Box::new(CubicSplinePlanner::new(0.1)),
            costmap: Costmap::new(),
            path_follower: Box::new(DefaultPathFollower::new()),
            current_path: Vec::new(),
        }
    }

    /// Create a new navigation stack with a specific path follower
    pub fn with_path_follower<T: PathFollower + 'static>(path_follower: T) -> Self {
        NavigationStack {
            base: LifecycleNodeBase::new("navigation_stack"),
            planner: Box::new(CubicSplinePlanner::new(0.1)),
            costmap: Costmap::new(),
            path_follower: Box::new(path_follower),
            current_path: Vec::new(),
        }
    }

    /// Set the path follower
    pub fn set_path_follower<T: PathFollower + 'static>(&mut self, path_follower: T) {
        self.path_follower = Box::new(path_follower);
    }

    /// Configure the path follower
    pub fn configure_path_follower(&mut self, params: &HashMap<String, f64>) -> Result<(), String> {
        self.path_follower.configure(params)
    }

    /// Plan a path from start to goal
    pub fn plan_path(&self, start: (f64, f64), goal: (f64, f64)) -> Vec<(f64, f64)> {
        // First update the costmap
        self.costmap.update();

        // Then plan the path using the cubic spline planner
        match self.planner.plan_path(start, goal) {
            Ok(path) => path,
            Err(e) => {
                println!("Path planning failed: {}", e);
                // Fall back to simple linear path
                vec![start, goal]
            }
        }
    }

    /// Follow a path and generate control commands
    pub fn follow_path(&self, current_pose: (f64, f64, f64), path: &[(f64, f64)]) -> (f64, f64) {
        self.path_follower.compute_velocity(current_pose, path)
    }

    /// Get the name of the current path follower
    pub fn path_follower_name(&self) -> &str {
        self.path_follower.name()
    }

    /// Set the current path to follow
    pub fn set_path(&mut self, path: &[(f64, f64)]) {
        // Store the path for the path follower to use
        println!("Setting path with {} points", path.len());
        self.current_path = path.to_vec();
    }

    /// Compute velocity command for the current pose
    pub fn compute_velocity_command(&mut self, pose: (f64, f64, f64)) -> Option<VelocityCommand> {
        if self.current_path.is_empty() {
            // No path to follow
            return None;
        }

        // Use the path follower to compute velocity commands
        let velocity = self
            .path_follower
            .compute_velocity(pose, &self.current_path);

        // Convert to VelocityCommand
        Some(VelocityCommand {
            linear: velocity.0,
            angular: velocity.1,
        })
    }
}

/// Velocity command for the robot
pub struct VelocityCommand {
    pub linear: f64,
    pub angular: f64,
}

impl LifecycleNode for NavigationStack {
    fn on_configure(&mut self) -> Result<(), String> {
        println!("Configuring navigation stack");
        self.base.set_state(State::Inactive);
        Ok(())
    }

    fn on_activate(&mut self) -> Result<(), String> {
        println!("Activating navigation stack");
        self.base.set_state(State::Active);
        Ok(())
    }

    fn on_deactivate(&mut self) -> Result<(), String> {
        println!("Deactivating navigation stack");
        self.base.set_state(State::Inactive);
        Ok(())
    }

    fn on_cleanup(&mut self) -> Result<(), String> {
        println!("Cleaning up navigation stack");
        self.base.set_state(State::Unconfigured);
        Ok(())
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}
