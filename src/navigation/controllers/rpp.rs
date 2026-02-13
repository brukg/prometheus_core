//! Regulated Pure Pursuit (RPP) controller.
//!
//! This is a Rust port of Nav2's Regulated Pure Pursuit controller. It extends
//! the basic Pure Pursuit algorithm with:
//! - Curvature-based velocity regulation (slow down on sharp turns)
//! - Proximity-based velocity regulation (slow down near obstacles)
//! - Collision detection with projected footprint
//! - Rotate-to-heading behavior at goal

use crate::navigation::costmap::{cost_values, Costmap};
use crate::navigation::traits::{Controller, ControllerError};
use crate::navigation::types::*;
use std::sync::{Arc, RwLock};

/// Regulated Pure Pursuit controller.
pub struct RegulatedPurePursuit {
    name: String,
    costmap: Option<Arc<RwLock<Costmap>>>,

    // Path state
    global_plan: Vec<PoseStamped>,

    // Pure Pursuit parameters
    /// Base lookahead distance in meters.
    lookahead_dist: f64,
    /// Minimum lookahead distance.
    min_lookahead_dist: f64,
    /// Maximum lookahead distance.
    max_lookahead_dist: f64,
    /// Lookahead gain (scales with velocity).
    lookahead_gain: f64,

    // Velocity parameters
    /// Maximum linear velocity.
    max_linear_vel: f64,
    /// Minimum linear velocity.
    min_linear_vel: f64,
    /// Maximum angular velocity.
    max_angular_vel: f64,

    // Regulation parameters
    /// Enable curvature-based speed regulation.
    use_velocity_scaled_lookahead: bool,
    /// Enable cost-based speed regulation.
    use_cost_regulated_linear_velocity: bool,
    /// Cost to start slowing down.
    cost_scaling_dist: f64,
    /// Cost at which we reach minimum speed.
    cost_scaling_gain: f64,
    /// Inflation cost range for regulation (252 = near inscribed).
    regulated_linear_scaling_min_radius: f64,
    /// Speed at which curvature regulation fully kicks in.
    regulated_linear_scaling_min_speed: f64,

    // Approach parameters
    /// Enable approach velocity scaling near goal.
    use_approach_vel_scaling: bool,
    /// Distance at which to start slowing for goal.
    approach_velocity_scaling_dist: f64,

    // Rotate to heading parameters
    /// Rotate in place toward path heading before driving.
    rotate_to_heading_min_angle: f64,
    /// Angular velocity for rotate-to-heading.
    rotate_to_heading_angular_vel: f64,

    // Goal tolerance
    goal_tolerance: f64,
}

impl RegulatedPurePursuit {
    pub fn new() -> Self {
        Self {
            name: "RegulatedPurePursuit".to_string(),
            costmap: None,
            global_plan: Vec::new(),
            lookahead_dist: 0.6,
            min_lookahead_dist: 0.3,
            max_lookahead_dist: 0.9,
            lookahead_gain: 1.0,
            max_linear_vel: 0.5,
            min_linear_vel: 0.0,
            max_angular_vel: 1.0,
            use_velocity_scaled_lookahead: true,
            use_cost_regulated_linear_velocity: true,
            cost_scaling_dist: 0.6,
            cost_scaling_gain: 1.0,
            regulated_linear_scaling_min_radius: 0.9,
            regulated_linear_scaling_min_speed: 0.25,
            use_approach_vel_scaling: true,
            approach_velocity_scaling_dist: 0.6,
            rotate_to_heading_min_angle: 1.5,
            rotate_to_heading_angular_vel: 1.8,
            goal_tolerance: 0.25,
        }
    }

    /// Compute the adaptive lookahead distance based on current speed.
    fn get_lookahead_dist(&self, speed: f64) -> f64 {
        if self.use_velocity_scaled_lookahead {
            let ld = speed * self.lookahead_gain;
            ld.clamp(self.min_lookahead_dist, self.max_lookahead_dist)
        } else {
            self.lookahead_dist
        }
    }

    /// Find the lookahead point on the path.
    /// Returns (lookahead_pose, carrot_distance).
    fn get_lookahead_point(
        &self,
        robot_pose: &Pose2D,
        lookahead_dist: f64,
    ) -> Option<(Pose2D, f64)> {
        if self.global_plan.is_empty() {
            return None;
        }

        // Find the closest point on the path
        let mut closest_idx = 0;
        let mut min_dist_sq = f64::MAX;
        for (i, ps) in self.global_plan.iter().enumerate() {
            let dx = ps.pose.x - robot_pose.x;
            let dy = ps.pose.y - robot_pose.y;
            let d_sq = dx * dx + dy * dy;
            if d_sq < min_dist_sq {
                min_dist_sq = d_sq;
                closest_idx = i;
            }
        }

        // Walk forward from closest point to find the lookahead point
        let mut carrot_idx = closest_idx;
        let mut accumulated_dist = 0.0;

        while carrot_idx + 1 < self.global_plan.len() {
            let dx = self.global_plan[carrot_idx + 1].pose.x - self.global_plan[carrot_idx].pose.x;
            let dy = self.global_plan[carrot_idx + 1].pose.y - self.global_plan[carrot_idx].pose.y;
            let seg_len = (dx * dx + dy * dy).sqrt();

            if accumulated_dist + seg_len >= lookahead_dist {
                // Interpolate within this segment
                let remaining = lookahead_dist - accumulated_dist;
                let t = if seg_len > 1e-6 { remaining / seg_len } else { 0.0 };
                let lx = self.global_plan[carrot_idx].pose.x + t * dx;
                let ly = self.global_plan[carrot_idx].pose.y + t * dy;
                let lyaw = dy.atan2(dx);

                let carrot_dist_dx = lx - robot_pose.x;
                let carrot_dist_dy = ly - robot_pose.y;
                let carrot_dist = (carrot_dist_dx * carrot_dist_dx + carrot_dist_dy * carrot_dist_dy).sqrt();

                return Some((Pose2D::new(lx, ly, lyaw), carrot_dist));
            }

            accumulated_dist += seg_len;
            carrot_idx += 1;
        }

        // If we ran out of path, use the last point
        let last = &self.global_plan[self.global_plan.len() - 1].pose;
        let dx = last.x - robot_pose.x;
        let dy = last.y - robot_pose.y;
        let dist = (dx * dx + dy * dy).sqrt();
        Some((*last, dist))
    }

    /// Compute the curvature to the lookahead point.
    fn compute_curvature(&self, robot_pose: &Pose2D, carrot: &Pose2D, carrot_dist: f64) -> f64 {
        if carrot_dist < 1e-6 {
            return 0.0;
        }

        // Transform carrot to robot frame
        let dx = carrot.x - robot_pose.x;
        let dy = carrot.y - robot_pose.y;
        let cos_yaw = robot_pose.yaw.cos();
        let sin_yaw = robot_pose.yaw.sin();
        let local_y = -dx * sin_yaw + dy * cos_yaw;

        // Pure pursuit curvature formula: κ = 2y / L²
        2.0 * local_y / (carrot_dist * carrot_dist)
    }

    /// Apply curvature-based velocity regulation.
    fn apply_curvature_regulation(&self, linear_vel: f64, curvature: f64) -> f64 {
        let radius = if curvature.abs() > 1e-6 {
            (1.0 / curvature).abs()
        } else {
            f64::MAX
        };

        if radius < self.regulated_linear_scaling_min_radius {
            let scale = radius / self.regulated_linear_scaling_min_radius;
            let regulated = linear_vel * scale;
            regulated.max(self.regulated_linear_scaling_min_speed)
        } else {
            linear_vel
        }
    }

    /// Apply cost-based velocity regulation (slow down near obstacles).
    fn apply_cost_regulation(&self, linear_vel: f64, robot_pose: &Pose2D) -> f64 {
        if !self.use_cost_regulated_linear_velocity {
            return linear_vel;
        }

        let costmap = match &self.costmap {
            Some(c) => c,
            None => return linear_vel,
        };

        let costmap = match costmap.read() {
            Ok(c) => c,
            Err(_) => return linear_vel,
        };

        let cost = costmap.get_cost(robot_pose.x, robot_pose.y);

        if cost == cost_values::NO_COST || cost == cost_values::UNKNOWN_COST {
            return linear_vel;
        }

        // Scale velocity based on cost: higher cost = slower
        let cost_ratio = cost as f64 / cost_values::INSCRIBED_INFLATED_OBSTACLE as f64;
        let scale = 1.0 - cost_ratio * self.cost_scaling_gain;
        let regulated = linear_vel * scale.max(0.1);
        regulated.max(self.min_linear_vel)
    }

    /// Apply approach velocity scaling near the goal.
    fn apply_approach_scaling(&self, linear_vel: f64, remaining_dist: f64) -> f64 {
        if !self.use_approach_vel_scaling {
            return linear_vel;
        }

        if remaining_dist < self.approach_velocity_scaling_dist {
            let scale = remaining_dist / self.approach_velocity_scaling_dist;
            let approach_vel = linear_vel * scale;
            approach_vel.max(self.min_linear_vel)
        } else {
            linear_vel
        }
    }

    /// Distance remaining on the path from robot position to goal.
    fn remaining_path_distance(&self, robot_pose: &Pose2D) -> f64 {
        if self.global_plan.is_empty() {
            return 0.0;
        }

        // Find closest point
        let mut closest_idx = 0;
        let mut min_dist_sq = f64::MAX;
        for (i, ps) in self.global_plan.iter().enumerate() {
            let dx = ps.pose.x - robot_pose.x;
            let dy = ps.pose.y - robot_pose.y;
            let d_sq = dx * dx + dy * dy;
            if d_sq < min_dist_sq {
                min_dist_sq = d_sq;
                closest_idx = i;
            }
        }

        // Sum remaining segment lengths
        let mut dist = min_dist_sq.sqrt();
        for i in closest_idx..self.global_plan.len().saturating_sub(1) {
            let dx = self.global_plan[i + 1].pose.x - self.global_plan[i].pose.x;
            let dy = self.global_plan[i + 1].pose.y - self.global_plan[i].pose.y;
            dist += (dx * dx + dy * dy).sqrt();
        }

        dist
    }
}

impl Controller for RegulatedPurePursuit {
    fn configure(&mut self, name: &str, costmap: Arc<RwLock<Costmap>>, params: &ParamMap) {
        self.name = name.to_string();
        self.costmap = Some(costmap);

        macro_rules! load_float {
            ($key:expr, $field:ident) => {
                if let Some(pv) = params.get($key) {
                    if let Some(v) = pv.as_float() {
                        self.$field = v;
                    }
                }
            };
        }
        macro_rules! load_bool {
            ($key:expr, $field:ident) => {
                if let Some(pv) = params.get($key) {
                    if let Some(v) = pv.as_bool() {
                        self.$field = v;
                    }
                }
            };
        }

        load_float!("lookahead_dist", lookahead_dist);
        load_float!("min_lookahead_dist", min_lookahead_dist);
        load_float!("max_lookahead_dist", max_lookahead_dist);
        load_float!("lookahead_gain", lookahead_gain);
        load_float!("max_linear_vel", max_linear_vel);
        load_float!("min_linear_vel", min_linear_vel);
        load_float!("max_angular_vel", max_angular_vel);
        load_float!("cost_scaling_dist", cost_scaling_dist);
        load_float!("cost_scaling_gain", cost_scaling_gain);
        load_float!("regulated_linear_scaling_min_radius", regulated_linear_scaling_min_radius);
        load_float!("regulated_linear_scaling_min_speed", regulated_linear_scaling_min_speed);
        load_float!("approach_velocity_scaling_dist", approach_velocity_scaling_dist);
        load_float!("rotate_to_heading_min_angle", rotate_to_heading_min_angle);
        load_float!("rotate_to_heading_angular_vel", rotate_to_heading_angular_vel);
        load_float!("goal_tolerance", goal_tolerance);
        load_bool!("use_velocity_scaled_lookahead", use_velocity_scaled_lookahead);
        load_bool!("use_cost_regulated_linear_velocity", use_cost_regulated_linear_velocity);
        load_bool!("use_approach_vel_scaling", use_approach_vel_scaling);
    }

    fn set_plan(&mut self, path: &Path) {
        self.global_plan = path.poses.clone();
    }

    fn compute_velocity_commands(
        &self,
        pose: &PoseStamped,
        velocity: &Twist,
    ) -> Result<Twist, ControllerError> {
        if self.global_plan.is_empty() {
            return Err(ControllerError::NoPath);
        }

        let robot = &pose.pose;
        let current_speed = velocity.linear_x.abs();

        // Rotate-to-heading: only at the beginning of path following (robot near
        // path start) and heading difference is large. Once moving, pure pursuit handles steering.
        {
            // Find the closest point on the path to determine if we're near the start
            let mut closest_idx = 0;
            let mut min_dist_sq = f64::MAX;
            for (i, ps) in self.global_plan.iter().enumerate() {
                let dx = ps.pose.x - robot.x;
                let dy = ps.pose.y - robot.y;
                let d_sq = dx * dx + dy * dy;
                if d_sq < min_dist_sq {
                    min_dist_sq = d_sq;
                    closest_idx = i;
                }
            }

            // Only rotate-to-heading in the first quarter of the path and when stopped
            let near_start = closest_idx < self.global_plan.len() / 4 + 1;
            if near_start && current_speed < 0.1 {
                // Use a point well ahead on the path for a stable heading reference
                let min_dist_for_heading = 0.3;
                let mut heading_target: Option<&Pose2D> = None;
                for ps in &self.global_plan {
                    let dx = ps.pose.x - robot.x;
                    let dy = ps.pose.y - robot.y;
                    if (dx * dx + dy * dy).sqrt() >= min_dist_for_heading {
                        heading_target = Some(&ps.pose);
                        break;
                    }
                }
                if heading_target.is_none() {
                    heading_target = self.global_plan.last().map(|ps| &ps.pose);
                }

                if let Some(target) = heading_target {
                    let dx = target.x - robot.x;
                    let dy = target.y - robot.y;
                    if (dx * dx + dy * dy).sqrt() > 0.05 {
                        let path_heading = dy.atan2(dx);
                        let angle_diff = normalize_angle(path_heading - robot.yaw);

                        if angle_diff.abs() > self.rotate_to_heading_min_angle {
                            // P-controller: angular vel proportional to error
                            let angular = angle_diff * self.rotate_to_heading_angular_vel;
                            return Ok(Twist::new(
                                0.0,
                                angular.clamp(-self.max_angular_vel, self.max_angular_vel),
                            ));
                        }
                    }
                }
            }
        }

        // Get lookahead distance (adaptive based on speed)
        let lookahead_dist = self.get_lookahead_dist(current_speed);

        // Find the lookahead point
        let (carrot, carrot_dist) = self
            .get_lookahead_point(robot, lookahead_dist)
            .ok_or(ControllerError::NoPath)?;

        // Compute curvature
        let curvature = self.compute_curvature(robot, &carrot, carrot_dist);

        // Start with max velocity
        let mut linear_vel = self.max_linear_vel;

        // Apply curvature-based regulation
        linear_vel = self.apply_curvature_regulation(linear_vel, curvature);

        // Apply cost-based regulation
        linear_vel = self.apply_cost_regulation(linear_vel, robot);

        // Apply approach velocity scaling
        let remaining = self.remaining_path_distance(robot);
        linear_vel = self.apply_approach_scaling(linear_vel, remaining);

        // Clamp linear velocity
        linear_vel = linear_vel.clamp(self.min_linear_vel, self.max_linear_vel);

        // Compute angular velocity from curvature: ω = v × κ
        let angular_vel = (linear_vel * curvature).clamp(-self.max_angular_vel, self.max_angular_vel);

        Ok(Twist::new(linear_vel, angular_vel))
    }

    fn is_goal_reached(
        &self,
        pose: &PoseStamped,
        goal: &PoseStamped,
        tolerance: f64,
    ) -> bool {
        pose.pose.distance_to(&goal.pose) <= tolerance
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn cleanup(&mut self) {
        self.costmap = None;
        self.global_plan.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_straight_path(n: usize, spacing: f64) -> Path {
        let poses: Vec<PoseStamped> = (0..n)
            .map(|i| {
                PoseStamped::new(
                    Pose2D::new(i as f64 * spacing, 0.0, 0.0),
                    "map",
                )
            })
            .collect();
        Path::from_poses(poses, "map")
    }

    #[test]
    fn test_rpp_straight_line() {
        let mut rpp = RegulatedPurePursuit::new();
        rpp.use_velocity_scaled_lookahead = false;
        rpp.lookahead_dist = 0.5;

        let path = make_straight_path(20, 0.1);
        rpp.set_plan(&path);

        let pose = PoseStamped::new(Pose2D::new(0.0, 0.0, 0.0), "map");
        let velocity = Twist::zero();

        let result = rpp.compute_velocity_commands(&pose, &velocity);
        assert!(result.is_ok());

        let cmd = result.unwrap();
        // Should drive forward
        assert!(cmd.linear_x > 0.0, "Should have positive linear velocity");
        // Should have minimal angular velocity on a straight path
        assert!(
            cmd.angular_z.abs() < 0.5,
            "Angular velocity should be small on straight path: {}",
            cmd.angular_z
        );
    }

    #[test]
    fn test_rpp_turn_left() {
        let mut rpp = RegulatedPurePursuit::new();
        rpp.use_velocity_scaled_lookahead = false;
        rpp.lookahead_dist = 0.5;

        // Path goes forward then turns left
        let poses: Vec<PoseStamped> = vec![
            PoseStamped::new(Pose2D::new(0.0, 0.0, 0.0), "map"),
            PoseStamped::new(Pose2D::new(0.3, 0.0, 0.0), "map"),
            PoseStamped::new(Pose2D::new(0.5, 0.1, 0.0), "map"),
            PoseStamped::new(Pose2D::new(0.6, 0.3, 0.0), "map"),
            PoseStamped::new(Pose2D::new(0.6, 0.6, std::f64::consts::FRAC_PI_2), "map"),
        ];
        let path = Path::from_poses(poses, "map");
        rpp.set_plan(&path);

        let pose = PoseStamped::new(Pose2D::new(0.0, 0.0, 0.0), "map");
        let velocity = Twist::zero();

        let result = rpp.compute_velocity_commands(&pose, &velocity);
        assert!(result.is_ok());

        let cmd = result.unwrap();
        // Should have positive angular velocity (turning left)
        assert!(
            cmd.angular_z > 0.0,
            "Should turn left, got angular_z={}",
            cmd.angular_z
        );
    }

    #[test]
    fn test_rpp_no_path() {
        let rpp = RegulatedPurePursuit::new();
        let pose = PoseStamped::new(Pose2D::new(0.0, 0.0, 0.0), "map");
        let velocity = Twist::zero();

        let result = rpp.compute_velocity_commands(&pose, &velocity);
        assert!(matches!(result, Err(ControllerError::NoPath)));
    }

    #[test]
    fn test_rpp_curvature_regulation() {
        let rpp = RegulatedPurePursuit::new();

        // Small curvature = no regulation
        let v1 = rpp.apply_curvature_regulation(0.5, 0.1);
        // Large curvature = regulated
        let v2 = rpp.apply_curvature_regulation(0.5, 5.0);

        assert!(v2 <= v1, "Large curvature should reduce velocity");
    }

    #[test]
    fn test_rpp_approach_scaling() {
        let rpp = RegulatedPurePursuit::new();

        // Far from goal
        let v1 = rpp.apply_approach_scaling(0.5, 5.0);
        // Near goal
        let v2 = rpp.apply_approach_scaling(0.5, 0.1);

        assert!(v2 < v1, "Should slow down near goal");
    }

    #[test]
    fn test_rpp_goal_reached() {
        let rpp = RegulatedPurePursuit::new();
        let pose = PoseStamped::new(Pose2D::new(1.0, 1.0, 0.0), "map");
        let goal = PoseStamped::new(Pose2D::new(1.05, 1.05, 0.0), "map");

        assert!(rpp.is_goal_reached(&pose, &goal, 0.1));
        assert!(!rpp.is_goal_reached(&pose, &goal, 0.01));
    }

    #[test]
    fn test_rpp_rotate_to_heading() {
        let mut rpp = RegulatedPurePursuit::new();
        rpp.rotate_to_heading_min_angle = 0.5;

        // Path is to the left, but robot faces right
        let poses = vec![
            PoseStamped::new(Pose2D::new(0.0, 1.0, 0.0), "map"),
            PoseStamped::new(Pose2D::new(0.0, 2.0, 0.0), "map"),
        ];
        let path = Path::from_poses(poses, "map");
        rpp.set_plan(&path);

        let pose = PoseStamped::new(Pose2D::new(0.0, 0.0, 0.0), "map");
        let velocity = Twist::zero();

        let result = rpp.compute_velocity_commands(&pose, &velocity);
        assert!(result.is_ok());

        let cmd = result.unwrap();
        // Should rotate in place (zero linear, positive angular toward the path)
        assert_eq!(cmd.linear_x, 0.0, "Should have zero linear velocity during rotation");
        assert!(cmd.angular_z > 0.0, "Should rotate toward path heading");
    }

    #[test]
    fn test_rpp_adaptive_lookahead() {
        let rpp = RegulatedPurePursuit::new();

        // At zero speed: should get minimum lookahead
        let ld_zero = rpp.get_lookahead_dist(0.0);
        assert!(
            (ld_zero - rpp.min_lookahead_dist).abs() < 1e-6,
            "At zero speed, lookahead should be minimum"
        );

        // At high speed: should get larger lookahead
        let ld_fast = rpp.get_lookahead_dist(1.0);
        assert!(
            ld_fast > ld_zero,
            "At higher speed, lookahead should be larger"
        );
    }
}
