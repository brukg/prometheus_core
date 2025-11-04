//! Navigation module for Prometheus robot
pub mod costmap;
pub mod path_follower;
pub mod path_planning;
pub mod planner;

use self::costmap::{CostmapManager, WorldPoint};
use self::path_follower::DefaultPathFollower;
use self::path_follower::PathFollower;
use self::path_follower::pure_pursuit::PurePursuitFollower;
use self::path_planning::CubicSplinePlanner;
use self::planner::PathPlanner;
use crate::lifecycle::{LifecycleNode, LifecycleNodeBase, State};
use std::any::Any;
use std::collections::HashMap;
use std::sync::{Arc, RwLock};

/// Navigation stack for the robot
pub struct NavigationStack {
    base: LifecycleNodeBase,
    planner: Box<dyn PathPlanner>,
    costmap_manager: Arc<CostmapManager>,
    path_follower: Box<dyn PathFollower>,
    current_path: Vec<(f64, f64)>,
    cubic_spline_planner: CubicSplinePlanner,
}

impl NavigationStack {
    /// Create a new navigation stack
    pub fn new() -> Self {
        let costmap_manager = Arc::new(CostmapManager::new());
        
        let mut path_follower = PurePursuitFollower::new();
        path_follower.set_costmap_manager(Arc::clone(&costmap_manager));
        
        NavigationStack {
            base: LifecycleNodeBase::new("navigation_stack"),
            planner: Box::new(CubicSplinePlanner::new(0.1)),
            costmap_manager: Arc::clone(&costmap_manager),
            path_follower: Box::new(path_follower),
            current_path: Vec::new(),
            cubic_spline_planner: CubicSplinePlanner::new(0.1),
        }
    }

    /// Create a new navigation stack with a specific path follower
    pub fn with_path_follower<T: PathFollower + 'static>(mut path_follower: T) -> Self {
        let costmap_manager = Arc::new(CostmapManager::new());
        
        // If path_follower is a PurePursuitFollower, connect the costmap
        if let Some(pp_follower) = (&mut path_follower as &mut dyn Any).downcast_mut::<PurePursuitFollower>() {
            pp_follower.set_costmap_manager(Arc::clone(&costmap_manager));
        }
        
        NavigationStack {
            base: LifecycleNodeBase::new("navigation_stack"),
            planner: Box::new(CubicSplinePlanner::new(0.1)),
            costmap_manager,
            path_follower: Box::new(path_follower),
            current_path: Vec::new(),
            cubic_spline_planner: CubicSplinePlanner::new(0.1),
        }
    }

    /// Set the path follower
    pub fn set_path_follower<T: PathFollower + 'static>(&mut self, mut path_follower: T) {
        // If path_follower is a PurePursuitFollower, connect the costmap
        if let Some(pp_follower) = (&mut path_follower as &mut dyn Any).downcast_mut::<PurePursuitFollower>() {
            pp_follower.set_costmap_manager(Arc::clone(&self.costmap_manager));
        }
        
        self.path_follower = Box::new(path_follower);
    }

    /// Configure the path follower
    pub fn configure_path_follower(&mut self, params: &HashMap<String, f64>) -> Result<(), String> {
        self.path_follower.configure(params)
    }
    
    /// Configure the costmaps
    pub fn configure_costmaps(&self, global_params: &HashMap<String, f64>, local_params: &HashMap<String, f64>) -> Result<(), String> {
        self.costmap_manager.configure(global_params, local_params)
    }
    
    /// Update the global costmap from a map
    pub fn update_global_map(&self, map_data: &[u8], width: usize, height: usize, resolution: f64, 
                            origin_x: f64, origin_y: f64) -> Result<(), String> {
        self.costmap_manager.update_global_map(map_data, width, height, resolution, origin_x, origin_y)
    }
    
    /// Update the local costmap from sensor data
    pub fn update_local_map(&self, sensor_data: &[WorldPoint], sensor_range: f64) -> Result<(), String> {
        self.costmap_manager.update_local_map(sensor_data, sensor_range)
    }

    /// Plan a path from start to goal
    pub fn plan_path(&self, start: (f64, f64), goal: (f64, f64)) -> Vec<(f64, f64)> {
        println!("Planning path from {:?} to {:?} with obstacle avoidance", start, goal);
        
        // Use the enhanced cubic spline planner with costmap
        match self.cubic_spline_planner.plan_with_costmap(start, goal, &self.costmap_manager) {
            Ok(path) => {
                if path.is_empty() {
                    println!("Path planning returned empty path, falling back to simple path");
                    vec![start, goal] // Fallback to simple direct path
                } else {
                    println!("Successfully planned path with {} points", path.len());
                    path
                }
            },
            Err(e) => {
                println!("Path planning with costmap failed: {}", e);
                
                // Fallback to basic path planning without costmap
                match self.planner.plan_path(start, goal) {
                    Ok(path) => {
                        println!("Using fallback path planning without costmap");
                        path
                    },
                    Err(e) => {
                        println!("All path planning failed: {}", e);
                        // Last resort: just return start and goal
                        vec![start, goal]
                    }
                }
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

    /// Get access to the costmap manager
    pub fn costmap_manager(&self) -> Arc<CostmapManager> {
        Arc::clone(&self.costmap_manager)
    }

    /// Compute velocity command for the current pose
    pub fn compute_velocity_command(&mut self, pose: (f64, f64, f64)) -> Option<VelocityCommand> {
        if self.current_path.is_empty() {
            // No path to follow
            return None;
        }

        // Use the path follower to compute velocity commands
        // The PurePursuitFollower will automatically handle obstacle detection
        // through its costmap integration
        let velocity = self
            .path_follower
            .compute_velocity(pose, &self.current_path);
            
        // If velocity is zero, we might need to replan
        if velocity.0 == 0.0 && velocity.1 == 0.0 {
            println!("Path follower returned zero velocity, obstacle detected");
            // In a real implementation, you would trigger replanning here
        }

        // Convert to VelocityCommand
        Some(VelocityCommand {
            linear: velocity.0,
            angular: velocity.1,
        })
    }
    
    /// Check if there are obstacles ahead of the robot
    fn check_obstacles_ahead(&self, pose: (f64, f64, f64), look_ahead_distance: f64) -> bool {
        // Calculate a point ahead of the robot
        let ahead_x = pose.0 + look_ahead_distance * pose.2.cos();
        let ahead_y = pose.1 + look_ahead_distance * pose.2.sin();
        
        // Check if this point is an obstacle
        match self.costmap_manager.is_obstacle(ahead_x, ahead_y) {
            Ok(is_obstacle) => is_obstacle,
            Err(_) => false, // If we can't check, assume it's safe
        }
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
