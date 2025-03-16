//! Pure Pursuit path follower implementation

use super::PathFollower;
use std::collections::HashMap;

/// Pure Pursuit path follower for the robot
#[derive(Debug)]
pub struct PurePursuitFollower {
    // Pure Pursuit parameters
    lookahead_distance: f64,
    k: f64, // look forward gain
    max_linear_velocity: f64,
    wheel_base: f64, // wheel base of vehicle
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
        let (delta, _) = self.pure_pursuit_control(state, &mut target_course, 0);

        // Convert steering angle to angular velocity
        let angular_velocity = self.max_linear_velocity * (delta).tan() / self.wheel_base;

        (self.max_linear_velocity, angular_velocity)
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

        Ok(())
    }
}

impl PurePursuitFollower {
    /// Calculate steering angle using pure pursuit algorithm
    fn pure_pursuit_control(
        &self,
        state: State,
        trajectory: &mut TargetCourse,
        pind: i32,
    ) -> (f64, i32) {
        let pair = trajectory.search_target_index(state);
        let mut ind = pair.0;
        let lf = pair.1;

        if pind > ind {
            ind = pind;
        }

        // If the target index is out of bounds, use the last point
        let tx: f64;
        let ty: f64;
        let traj_len = trajectory.cx.len();

        if traj_len == 0 {
            return (0.0, 0);
        }

        if ind < traj_len as i32 {
            tx = trajectory.cx[ind as usize];
            ty = trajectory.cy[ind as usize];
        } else {
            tx = trajectory.cx[traj_len - 1];
            ty = trajectory.cy[traj_len - 1];
            ind = traj_len as i32 - 1;
        }

        // Calculate the angle between the vehicle and the target point
        let alpha = (ty - state.rear_y).atan2(tx - state.rear_x) - state.yaw;

        // Calculate the steering angle
        let delta = (2.0 * state.wb * alpha.sin() / lf).atan2(1.0);

        (delta, ind)
    }
}
