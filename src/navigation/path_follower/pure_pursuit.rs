//! Pure Pursuit path follower implementation

use super::PathFollower;
use crate::navigation::costmap::{CostmapManager, WorldPoint, cost_values};
use std::collections::HashMap;
use std::sync::Arc;

/// Pure Pursuit path follower for the robot
#[derive(Debug)]
pub struct PurePursuitFollower {
    // Pure Pursuit parameters
    lookahead_distance: f64,
    k: f64, // look forward gain
    max_linear_velocity: f64,
    wheel_base: f64, // wheel base of vehicle
    // Optional costmap manager for obstacle awareness
    costmap_manager: Option<Arc<CostmapManager>>,
    // Safety parameters
    obstacle_deceleration_distance: f64, // Start slowing down at this distance from an obstacle
    min_obstacle_clearance: f64,        // Minimum distance to maintain from obstacles
}

// State representation for pure pursuit calculations
#[derive(Debug, Copy, Clone)]
struct State {
    x: f64,
    y: f64,
    yaw: f64,
    v: f64,
    rear_x: f64,
    rear_y: f64,
    wb: f64,
}

impl State {
    fn new(x: f64, y: f64, yaw: f64, v: f64, wb: f64) -> State {
        State {
            x,
            y,
            yaw,
            v,
            rear_x: x - ((wb / 2.) * yaw.cos()),
            rear_y: y - ((wb / 2.) * yaw.sin()),
            wb,
        }
    }

    fn calc_distance(&self, point_x: f64, point_y: f64) -> f64 {
        let dx = self.rear_x - point_x;
        let dy = self.rear_y - point_y;
        (dx.powi(2) + dy.powi(2)).sqrt()
    }
}

struct TargetCourse {
    cx: Vec<f64>,
    cy: Vec<f64>,
    old_nearest_point_index: i32,
    k: f64,
    lfc: f64,
}

impl TargetCourse {
    fn new(path: &[(f64, f64)], k: f64, lfc: f64) -> TargetCourse {
        let mut cx = Vec::with_capacity(path.len());
        let mut cy = Vec::with_capacity(path.len());

        for point in path {
            cx.push(point.0);
            cy.push(point.1);
        }

        TargetCourse {
            cx,
            cy,
            old_nearest_point_index: -1,
            k,
            lfc,
        }
    }

    fn search_target_index(&mut self, state: State) -> (i32, f64) {
        if self.cx.is_empty() {
            return (0, self.lfc);
        }

        // Declare ind_min outside the if blocks so it's available throughout the function
        let mut ind_min = 0;

        if self.old_nearest_point_index == -1 {
            let mut d_min = std::f64::MAX;
            // Use the already declared ind_min
            for i in 0..self.cx.len() {
                let d = state.calc_distance(self.cx[i], self.cy[i]);
                if d < d_min {
                    d_min = d;
                    ind_min = i;
                }
            }
            self.old_nearest_point_index = ind_min as i32;
        } else {
            ind_min = self.old_nearest_point_index as usize;
            if ind_min + 1 < self.cx.len() {
                let mut distance_this_index =
                    state.calc_distance(self.cx[ind_min], self.cy[ind_min]);
                loop {
                    if ind_min + 1 >= self.cx.len() {
                        break;
                    }

                    let distance_next_index =
                        state.calc_distance(self.cx[ind_min + 1], self.cy[ind_min + 1]);
                    if distance_this_index < distance_next_index {
                        break;
                    }

                    ind_min += 1;
                    distance_this_index = distance_next_index;

                    if ind_min + 1 >= self.cx.len() {
                        break;
                    }
                }
                self.old_nearest_point_index = ind_min as i32;
            }
        }

        let lf = self.k * state.v + self.lfc;

        // Find lookahead target point
        let mut target_ind = ind_min;
        while target_ind < self.cx.len()
            && lf > state.calc_distance(self.cx[target_ind], self.cy[target_ind])
        {
            target_ind += 1;
            if target_ind >= self.cx.len() {
                break;
            }
        }

        // If we went beyond the path length, use the last point
        if target_ind >= self.cx.len() {
            target_ind = self.cx.len() - 1;
        }

        (target_ind as i32, lf)
    }
}

impl PathFollower for PurePursuitFollower {
    fn new() -> Self {
        PurePursuitFollower {
            lookahead_distance: 2.0,
            k: 0.1,
            max_linear_velocity: 0.5,
            wheel_base: 0.5,
            costmap_manager: None,
            obstacle_deceleration_distance: 1.0,
            min_obstacle_clearance: 0.3,
        }
    }

    fn compute_velocity(&self, current_pose: (f64, f64, f64), path: &[(f64, f64)]) -> (f64, f64) {
        if path.is_empty() {
            return (0.0, 0.0);
        }

        // Create state from current pose
        let state = State::new(
            current_pose.0,
            current_pose.1,
            current_pose.2,
            self.max_linear_velocity,
            self.wheel_base,
        );

        // Create target course from path
        let mut target_course = TargetCourse::new(path, self.k, self.lookahead_distance);

        // Calculate steering angle using pure pursuit
        let (delta, target_idx) = self.pure_pursuit_control(state, &mut target_course, 0);

        // Start with max velocity
        let mut linear_velocity = self.max_linear_velocity;
        
        // If we have a costmap manager, check for obstacles
        if let Some(costmap_manager) = &self.costmap_manager {
            // Check ahead for obstacles and adjust speed accordingly
            if let Some(distance_to_obstacle) = self.check_obstacles_ahead(current_pose, &path[target_idx as usize..]) {
                // Adjust velocity based on distance to obstacle
                if distance_to_obstacle < self.min_obstacle_clearance {
                    // Too close to an obstacle, stop
                    linear_velocity = 0.0;
                    println!("Pure Pursuit: Stopping due to obstacle at distance {}", distance_to_obstacle);
                } else if distance_to_obstacle < self.obstacle_deceleration_distance {
                    // Scale velocity based on distance to obstacle
                    let scale_factor = (distance_to_obstacle - self.min_obstacle_clearance) / 
                                      (self.obstacle_deceleration_distance - self.min_obstacle_clearance);
                    linear_velocity = linear_velocity * scale_factor;
                    println!("Pure Pursuit: Slowing down to {} due to obstacle at distance {}", 
                             linear_velocity, distance_to_obstacle);
                }
            }
        }

        // Convert steering angle to angular velocity
        let angular_velocity = linear_velocity * (delta).tan() / self.wheel_base;

        (linear_velocity, angular_velocity)
    }

    fn name(&self) -> &str {
        "PurePursuitFollower"
    }

    fn configure(&mut self, params: &HashMap<String, f64>) -> Result<(), String> {
        if let Some(&lookahead) = params.get("lookahead_distance") {
            if lookahead <= 0.0 {
                return Err("Lookahead distance must be positive".to_string());
            }
            self.lookahead_distance = lookahead;
        }

        if let Some(&k) = params.get("k") {
            if k <= 0.0 {
                return Err("Look forward gain must be positive".to_string());
            }
            self.k = k;
        }

        if let Some(&max_linear) = params.get("max_linear_velocity") {
            if max_linear <= 0.0 {
                return Err("Max linear velocity must be positive".to_string());
            }
            self.max_linear_velocity = max_linear;
        }

        if let Some(&wheel_base) = params.get("wheel_base") {
            if wheel_base <= 0.0 {
                return Err("Wheel base must be positive".to_string());
            }
            self.wheel_base = wheel_base;
        }
        
        if let Some(&obstacle_decel) = params.get("obstacle_deceleration_distance") {
            if obstacle_decel <= 0.0 {
                return Err("Obstacle deceleration distance must be positive".to_string());
            }
            self.obstacle_deceleration_distance = obstacle_decel;
        }
        
        if let Some(&min_clearance) = params.get("min_obstacle_clearance") {
            if min_clearance <= 0.0 {
                return Err("Minimum obstacle clearance must be positive".to_string());
            }
            self.min_obstacle_clearance = min_clearance;
        }

        Ok(())
    }
}

impl PurePursuitFollower {
    /// Set the costmap manager to enable obstacle awareness
    pub fn set_costmap_manager(&mut self, costmap_manager: Arc<CostmapManager>) {
        self.costmap_manager = Some(costmap_manager);
    }
    
    /// Check for obstacles along a path segment, returns distance to nearest obstacle if found
    fn check_obstacles_ahead(&self, current_pose: (f64, f64, f64), path_segment: &[(f64, f64)]) -> Option<f64> {
        if let Some(costmap_manager) = &self.costmap_manager {
            let mut min_distance = std::f64::MAX;
            let mut obstacle_found = false;
            
            // Check along the path for obstacles
            for i in 0..std::cmp::min(path_segment.len(), 10) { // Only check the next 10 points
                let point = path_segment[i];
                
                // Check if this point is an obstacle
                match costmap_manager.is_obstacle(point.0, point.1) {
                    Ok(true) => {
                        // Calculate distance to this obstacle
                        let dx = point.0 - current_pose.0;
                        let dy = point.1 - current_pose.1;
                        let distance = (dx * dx + dy * dy).sqrt();
                        
                        if distance < min_distance {
                            min_distance = distance;
                            obstacle_found = true;
                        }
                    },
                    _ => {}
                }
                
                // Also check the line segment to this point
                if i > 0 {
                    let prev = path_segment[i-1];
                    // Sample points along the line
                    let steps = 5;
                    for j in 1..steps {
                        let t = j as f64 / steps as f64;
                        let x = prev.0 + t * (point.0 - prev.0);
                        let y = prev.1 + t * (point.1 - prev.1);
                        
                        match costmap_manager.is_obstacle(x, y) {
                            Ok(true) => {
                                // Calculate distance to this obstacle
                                let dx = x - current_pose.0;
                                let dy = y - current_pose.1;
                                let distance = (dx * dx + dy * dy).sqrt();
                                
                                if distance < min_distance {
                                    min_distance = distance;
                                    obstacle_found = true;
                                }
                            },
                            _ => {}
                        }
                    }
                }
            }
            
            if obstacle_found {
                return Some(min_distance);
            }
        }
        
        None // No obstacles found
    }

    fn pure_pursuit_control(
        &self,
        state: State,
        trajectory: &mut TargetCourse,
        pind: i32,
    ) -> (f64, i32) {
        let (target_ind, lf) = trajectory.search_target_index(state);
        
        // If we couldn't find a valid target index, return 0 steering
        if target_ind < 0 {
            return (0.0, 0);
        }
        
        // If the last given index is ahead of our current index, use it
        let target_ind = if pind >= target_ind {
            pind
        } else {
            target_ind
        };

        if target_ind < trajectory.cx.len() as i32 {
            let tx = trajectory.cx[target_ind as usize];
            let ty = trajectory.cy[target_ind as usize];

            // Calculate angle difference
            let alpha = (ty - state.rear_y).atan2(tx - state.rear_x) - state.yaw;
            let delta = (2.0 * state.wb * alpha.sin() / lf).atan();
            
            return (delta, target_ind);
        }

        (0.0, target_ind)
    }
}
