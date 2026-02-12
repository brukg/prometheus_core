//! Simple path smoother — iterative gradient-based smoothing.
//!
//! Shifts each waypoint toward the midpoint of its neighbors while keeping
//! the path in costmap-free space. Matches Nav2's `SimpleSmoother`.

use crate::navigation::costmap::{cost_values, Costmap};
use crate::navigation::traits::{Smoother, SmootherError};
use crate::navigation::types::*;
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};

/// Simple iterative path smoother.
pub struct SimpleSmoother {
    name: String,
    costmap: Option<Arc<RwLock<Costmap>>>,
    /// Maximum number of iterations.
    max_iterations: usize,
    /// Smoothing weight (how much to pull toward neighbor midpoint). Range [0, 1].
    w_smooth: f64,
    /// Data weight (how much to keep original position). Range [0, 1].
    w_data: f64,
    /// Convergence tolerance (stop when change is below this).
    tolerance: f64,
    /// Whether to refine path to avoid costmap obstacles after smoothing.
    do_refinement: bool,
}

impl SimpleSmoother {
    pub fn new() -> Self {
        Self {
            name: "SimpleSmoother".to_string(),
            costmap: None,
            max_iterations: 1000,
            w_smooth: 0.4,
            w_data: 0.5,
            tolerance: 1e-5,
            do_refinement: true,
        }
    }

    /// Smooth a path using iterative optimization.
    /// This modifies the internal waypoints (not the first and last) to minimize
    /// path roughness while staying close to the original path.
    fn smooth_internal(
        &self,
        poses: &[PoseStamped],
        max_duration: Duration,
    ) -> Result<Vec<PoseStamped>, SmootherError> {
        if poses.len() <= 2 {
            return Ok(poses.to_vec());
        }

        let start_time = Instant::now();

        // Extract x,y coordinates for smoothing
        let n = poses.len();
        let mut xs: Vec<f64> = poses.iter().map(|p| p.pose.x).collect();
        let mut ys: Vec<f64> = poses.iter().map(|p| p.pose.y).collect();

        let orig_xs = xs.clone();
        let orig_ys = ys.clone();

        for _iter in 0..self.max_iterations {
            if start_time.elapsed() > max_duration {
                break; // Time budget exhausted — return best result so far
            }

            let mut max_change = 0.0_f64;

            for i in 1..n - 1 {
                // Smoothing: pull toward neighbor midpoint
                let new_x = xs[i]
                    + self.w_data * (orig_xs[i] - xs[i])
                    + self.w_smooth * (xs[i - 1] + xs[i + 1] - 2.0 * xs[i]);
                let new_y = ys[i]
                    + self.w_data * (orig_ys[i] - ys[i])
                    + self.w_smooth * (ys[i - 1] + ys[i + 1] - 2.0 * ys[i]);

                let change_x = (new_x - xs[i]).abs();
                let change_y = (new_y - ys[i]).abs();

                xs[i] = new_x;
                ys[i] = new_y;

                max_change = max_change.max(change_x).max(change_y);
            }

            if max_change < self.tolerance {
                break; // Converged
            }
        }

        // Refinement: check smoothed path against costmap and pull away from obstacles
        if self.do_refinement {
            if let Some(costmap_arc) = &self.costmap {
                if let Ok(costmap) = costmap_arc.read() {
                    for i in 1..n - 1 {
                        let cost = costmap.get_cost(xs[i], ys[i]);
                        if cost >= cost_values::INSCRIBED_INFLATED_OBSTACLE {
                            // Point is in obstacle — revert to original
                            xs[i] = orig_xs[i];
                            ys[i] = orig_ys[i];
                        }
                    }
                }
            }
        }

        // Reconstruct poses
        let frame_id = &poses[0].frame_id;
        let mut smoothed: Vec<PoseStamped> = Vec::with_capacity(n);
        for i in 0..n {
            let yaw = if i + 1 < n {
                (ys[i + 1] - ys[i]).atan2(xs[i + 1] - xs[i])
            } else {
                poses[i].pose.yaw
            };
            smoothed.push(PoseStamped {
                pose: Pose2D::new(xs[i], ys[i], yaw),
                frame_id: frame_id.clone(),
                stamp: poses[i].stamp,
            });
        }

        Ok(smoothed)
    }
}

impl Smoother for SimpleSmoother {
    fn configure(&mut self, name: &str, costmap: Arc<RwLock<Costmap>>, params: &ParamMap) {
        self.name = name.to_string();
        self.costmap = Some(costmap);

        if let Some(pv) = params.get("max_iterations") {
            if let Some(v) = pv.as_int() {
                self.max_iterations = v.max(1) as usize;
            }
        }
        if let Some(pv) = params.get("w_smooth") {
            if let Some(v) = pv.as_float() {
                self.w_smooth = v.clamp(0.0, 1.0);
            }
        }
        if let Some(pv) = params.get("w_data") {
            if let Some(v) = pv.as_float() {
                self.w_data = v.clamp(0.0, 1.0);
            }
        }
        if let Some(pv) = params.get("tolerance") {
            if let Some(v) = pv.as_float() {
                self.tolerance = v;
            }
        }
        if let Some(pv) = params.get("do_refinement") {
            if let Some(v) = pv.as_bool() {
                self.do_refinement = v;
            }
        }
    }

    fn smooth(&self, path: &Path, max_duration: Duration) -> Result<Path, SmootherError> {
        let smoothed_poses = self.smooth_internal(&path.poses, max_duration)?;
        Ok(Path::from_poses(smoothed_poses, &path.frame_id))
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn cleanup(&mut self) {
        self.costmap = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_zigzag_path() -> Path {
        let poses = vec![
            PoseStamped::new(Pose2D::new(0.0, 0.0, 0.0), "map"),
            PoseStamped::new(Pose2D::new(1.0, 0.5, 0.0), "map"),
            PoseStamped::new(Pose2D::new(2.0, -0.3, 0.0), "map"),
            PoseStamped::new(Pose2D::new(3.0, 0.4, 0.0), "map"),
            PoseStamped::new(Pose2D::new(4.0, -0.2, 0.0), "map"),
            PoseStamped::new(Pose2D::new(5.0, 0.0, 0.0), "map"),
        ];
        Path::from_poses(poses, "map")
    }

    #[test]
    fn test_simple_smoother_reduces_zigzag() {
        let smoother = SimpleSmoother::new();
        let path = make_zigzag_path();

        let smoothed = smoother
            .smooth(&path, Duration::from_secs(1))
            .unwrap();

        // Endpoints should be preserved
        assert!((smoothed.poses[0].pose.x - 0.0).abs() < 1e-6);
        assert!((smoothed.poses[0].pose.y - 0.0).abs() < 1e-6);
        let last = smoothed.poses.last().unwrap();
        assert!((last.pose.x - 5.0).abs() < 1e-6);
        assert!((last.pose.y - 0.0).abs() < 1e-6);

        // Interior points should be smoother (less y-deviation)
        let original_deviation: f64 = path
            .poses
            .iter()
            .skip(1)
            .take(path.len() - 2)
            .map(|p| p.pose.y.abs())
            .sum();

        let smoothed_deviation: f64 = smoothed
            .poses
            .iter()
            .skip(1)
            .take(smoothed.len() - 2)
            .map(|p| p.pose.y.abs())
            .sum();

        assert!(
            smoothed_deviation < original_deviation,
            "Smoothing should reduce y-deviation: original={}, smoothed={}",
            original_deviation,
            smoothed_deviation
        );
    }

    #[test]
    fn test_simple_smoother_short_path() {
        let smoother = SimpleSmoother::new();
        let path = Path::from_poses(
            vec![
                PoseStamped::new(Pose2D::new(0.0, 0.0, 0.0), "map"),
                PoseStamped::new(Pose2D::new(1.0, 1.0, 0.0), "map"),
            ],
            "map",
        );

        let smoothed = smoother.smooth(&path, Duration::from_secs(1)).unwrap();
        assert_eq!(smoothed.len(), 2);
    }

    #[test]
    fn test_simple_smoother_preserves_endpoints() {
        let smoother = SimpleSmoother::new();
        let path = make_zigzag_path();

        let smoothed = smoother.smooth(&path, Duration::from_secs(1)).unwrap();

        assert_eq!(smoothed.len(), path.len());
        assert!((smoothed.poses[0].pose.x - path.poses[0].pose.x).abs() < 1e-10);
        assert!((smoothed.poses[0].pose.y - path.poses[0].pose.y).abs() < 1e-10);
        let last_s = smoothed.poses.last().unwrap();
        let last_o = path.poses.last().unwrap();
        assert!((last_s.pose.x - last_o.pose.x).abs() < 1e-10);
        assert!((last_s.pose.y - last_o.pose.y).abs() < 1e-10);
    }
}
