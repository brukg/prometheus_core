//! Prometheus Navigation Node — the main ROS2 navigation binary.
//!
//! This node subscribes to goal_pose, odom, and scan topics, plans paths using
//! the navigation stack, and publishes velocity commands on cmd_vel.
//!
//! Build: `colcon build --packages-select prometheus_core` (requires ROS2 workspace)
//! Run:   `ros2 run prometheus_core prometheus_navigation_node`
//!        `ros2 run prometheus_core prometheus_navigation_node --ros-args --params-file config/navigation_params.yaml`

// This entire binary requires a ROS2 workspace with rclrs and message packages.
// When building outside a ROS2 workspace (no `ros2` feature), this compiles to a
// stub that prints a helpful message.

#[cfg(feature = "ros2")]
mod ros2_node {
    use anyhow::{Error, Result};
    use prometheus_core::{
        control::ControlStack,
        navigation::costmap::WorldPoint,
        navigation::path_follower::pure_pursuit::PurePursuitFollower,
        navigation::path_follower::PathFollower,
        navigation::NavigationStack,
        perception::PerceptionStack,
        PrometheusCore,
    };
    use rclrs::{Context, Node, QOS_PROFILE_DEFAULT};
    use std::collections::HashMap;
    use std::sync::{Arc, Mutex};
    use std::thread;
    use std::time::Duration;

    use geometry_msgs::msg::{PoseStamped, Twist};
    use nav_msgs::msg::Odometry;
    use nav_msgs::msg::Path;
    use sensor_msgs::msg::LaserScan;

    pub struct PrometheusNavigationNode {
        core: Mutex<PrometheusCore>,
        node: Arc<Node>,
        cmd_vel_publisher: Arc<rclrs::Publisher<Twist>>,
        goal_subscription: Mutex<Option<Arc<rclrs::Subscription<PoseStamped>>>>,
        odom_subscription: Mutex<Option<Arc<rclrs::Subscription<Odometry>>>>,
        scan_subscription: Mutex<Option<Arc<rclrs::Subscription<LaserScan>>>>,
        running: Arc<Mutex<bool>>,
        current_path: Arc<Mutex<Option<Vec<(f64, f64)>>>>,
        current_goal: Arc<Mutex<Option<(f64, f64)>>>,
        current_pose: Arc<Mutex<(f64, f64, f64)>>,
        max_linear_velocity: f64,
        path_publisher: Arc<rclrs::Publisher<Path>>,
    }

    impl PrometheusNavigationNode {
        pub fn new(context: &Context, name: &str) -> Result<Arc<Self>, rclrs::RclrsError> {
            let node = Node::new(context, name)?;

            let mut core = PrometheusCore::new();

            let lookahead_distance = 0.8;
            let max_linear_velocity = 1.75;
            let cmd_vel_topic = "/prometheus/cmd_vel".to_string();
            let odom_topic = "/prometheus/odom".to_string();
            let goal_topic = "goal_pose".to_string();
            let scan_topic = "/prometheus/scan".to_string();

            println!(
                "Using parameters: lookahead_distance={}, max_linear_velocity={}",
                lookahead_distance, max_linear_velocity
            );
            println!(
                "Topics: cmd_vel={}, odom={}, goal={}, scan={}",
                cmd_vel_topic, odom_topic, goal_topic, scan_topic
            );

            let mut nav_stack = NavigationStack::with_path_follower(PurePursuitFollower::new());

            let mut params = HashMap::new();
            params.insert("lookahead_distance".to_string(), lookahead_distance);
            params.insert("max_linear_velocity".to_string(), max_linear_velocity);

            if let Err(e) = nav_stack.configure_path_follower(&params) {
                eprintln!("Failed to configure path follower: {}", e);
            }

            core.register(nav_stack);
            core.register(ControlStack::new());
            core.register(PerceptionStack::new());

            println!("Core initialized successfully!");

            let cmd_vel_publisher =
                node.create_publisher::<Twist>(&cmd_vel_topic, QOS_PROFILE_DEFAULT)?;
            let path_publisher =
                node.create_publisher::<Path>("/prometheus/planned_path", QOS_PROFILE_DEFAULT)?;

            let running = Arc::new(Mutex::new(true));

            let prometheus_navigation_node = Arc::new(PrometheusNavigationNode {
                core: Mutex::new(core),
                node,
                cmd_vel_publisher,
                goal_subscription: None.into(),
                odom_subscription: None.into(),
                scan_subscription: None.into(),
                running,
                current_path: Arc::new(Mutex::new(None)),
                current_goal: Arc::new(Mutex::new(None)),
                current_pose: Arc::new(Mutex::new((0.0, 0.0, 0.0))),
                max_linear_velocity,
                path_publisher,
            });

            // Set up goal subscription
            let node_clone = Arc::clone(&prometheus_navigation_node);
            let goal_subscription = prometheus_navigation_node
                .node
                .create_subscription::<PoseStamped, _>(
                    &goal_topic,
                    QOS_PROFILE_DEFAULT,
                    move |msg: PoseStamped| {
                        node_clone.goal_callback(msg);
                    },
                )?;
            *prometheus_navigation_node.goal_subscription.lock().unwrap() =
                Some(goal_subscription);

            // Set up odometry subscription
            let node_clone = Arc::clone(&prometheus_navigation_node);
            let odom_subscription = prometheus_navigation_node
                .node
                .create_subscription::<Odometry, _>(
                    &odom_topic,
                    QOS_PROFILE_DEFAULT,
                    move |msg: Odometry| {
                        node_clone.odom_callback(msg);
                    },
                )?;
            *prometheus_navigation_node.odom_subscription.lock().unwrap() =
                Some(odom_subscription);

            // Set up laser scan subscription
            let node_clone = Arc::clone(&prometheus_navigation_node);
            let scan_subscription = prometheus_navigation_node
                .node
                .create_subscription::<LaserScan, _>(
                    &scan_topic,
                    QOS_PROFILE_DEFAULT,
                    move |msg: LaserScan| {
                        node_clone.scan_callback(msg);
                    },
                )?;
            *prometheus_navigation_node.scan_subscription.lock().unwrap() =
                Some(scan_subscription);

            // Start a thread to periodically publish velocity commands
            let node_clone = Arc::clone(&prometheus_navigation_node);
            let running_clone = Arc::clone(&prometheus_navigation_node.running);

            thread::spawn(move || {
                while *running_clone.lock().unwrap() {
                    node_clone.timer_callback();
                    thread::sleep(Duration::from_millis(100)); // 10 Hz
                }
            });

            Ok(prometheus_navigation_node)
        }

        fn publish_path(&self, path: &[(f64, f64)]) {
            let mut path_msg = Path::default();
            path_msg.header.frame_id = "map".to_string();

            path_msg.poses = path
                .iter()
                .map(|(x, y)| {
                    let mut pose_stamped = PoseStamped::default();
                    pose_stamped.pose.position.x = *x;
                    pose_stamped.pose.position.y = *y;
                    pose_stamped.pose.orientation.w = 1.0;
                    pose_stamped
                })
                .collect();

            if let Err(e) = self.path_publisher.publish(&path_msg) {
                eprintln!("Failed to publish path: {}", e);
            } else {
                println!("Published path with {} points", path.len());
            }
        }

        fn goal_callback(&self, msg: PoseStamped) {
            println!(
                "Received new goal: x={}, y={}",
                msg.pose.position.x, msg.pose.position.y
            );

            let goal = (msg.pose.position.x, msg.pose.position.y);
            *self.current_goal.lock().unwrap() = Some(goal);

            let current_pose = *self.current_pose.lock().unwrap();

            let mut core = self.core.lock().unwrap();
            if let Some(nav_stack) = core.navigation_stack_mut() {
                let path = nav_stack.plan_path((current_pose.0, current_pose.1), goal);
                println!("Planned path with {} points", path.len());
                *self.current_path.lock().unwrap() = Some(path.clone());
                self.publish_path(&path);
            }
        }

        fn odom_callback(&self, msg: Odometry) {
            let x = msg.pose.pose.position.x;
            let y = msg.pose.pose.position.y;

            let qx = msg.pose.pose.orientation.x;
            let qy = msg.pose.pose.orientation.y;
            let qz = msg.pose.pose.orientation.z;
            let qw = msg.pose.pose.orientation.w;

            let theta =
                2.0 * (qw * qz + qx * qy).atan2(1.0 - 2.0 * (qy * qy + qz * qz));

            *self.current_pose.lock().unwrap() = (x, y, theta);

            println!(
                "Updated pose from odom: x={:.2}, y={:.2}, theta={:.2}",
                x, y, theta
            );
        }

        fn scan_callback(&self, msg: LaserScan) {
            let current_pose = *self.current_pose.lock().unwrap();

            let mut world_points = Vec::new();
            let angle_min = msg.angle_min;
            let angle_increment = msg.angle_increment;

            for (i, range) in msg.ranges.iter().enumerate() {
                if *range < msg.range_min || *range > msg.range_max {
                    continue;
                }

                let angle = angle_min + (angle_increment * i as f32);
                let x_robot = range * angle.cos();
                let y_robot = range * angle.sin();

                let x_world = current_pose.0
                    + (x_robot as f64 * current_pose.2.cos()
                        - y_robot as f64 * current_pose.2.sin());
                let y_world = current_pose.1
                    + (x_robot as f64 * current_pose.2.sin()
                        + y_robot as f64 * current_pose.2.cos());

                world_points.push(WorldPoint {
                    x: x_world,
                    y: y_world,
                });
            }

            let mut core = self.core.lock().unwrap();
            if let Some(nav_stack) = core.navigation_stack_mut() {
                if let Err(e) = nav_stack.update_local_map(&world_points, msg.range_max as f64)
                {
                    eprintln!("Failed to update local costmap with laser data: {}", e);
                } else {
                    println!(
                        "Updated local costmap with {} points from laser scan",
                        world_points.len()
                    );
                }
            }
        }

        fn timer_callback(&self) {
            let path_option = self.current_path.lock().unwrap().clone();

            if let Some(path) = path_option {
                let current_pose = *self.current_pose.lock().unwrap();
                let mut core = self.core.lock().unwrap();

                if let Some(nav_stack) = core.navigation_stack_mut() {
                    nav_stack.set_path(&path);
                    let pose = (current_pose.0, current_pose.1, current_pose.2);

                    if let Some(cmd) = nav_stack.compute_velocity_command(pose) {
                        let mut twist = Twist::default();
                        twist.linear.x = cmd.linear;
                        twist.angular.z = cmd.angular;

                        if let Err(e) = self.cmd_vel_publisher.publish(&twist) {
                            eprintln!("Failed to publish velocity command: {}", e);
                        }

                        println!(
                            "Pure Pursuit velocity: linear={:.2}, angular={:.2}",
                            cmd.linear, cmd.angular
                        );
                    } else {
                        println!(
                            "Pure Pursuit did not return a command, falling back to simple controller"
                        );
                        self.simple_controller(current_pose);
                    }

                    if let Some(goal) = *self.current_goal.lock().unwrap() {
                        let dx = goal.0 - current_pose.0;
                        let dy = goal.1 - current_pose.1;
                        let distance = (dx * dx + dy * dy).sqrt();

                        if distance < 0.1 {
                            println!("Goal reached!");
                            let mut twist = Twist::default();
                            twist.linear.x = 0.0;
                            twist.angular.z = 0.0;

                            if let Err(e) = self.cmd_vel_publisher.publish(&twist) {
                                eprintln!("Failed to publish velocity command: {}", e);
                            }
                            *self.current_path.lock().unwrap() = None;
                            *self.current_goal.lock().unwrap() = None;
                        }
                    }
                } else {
                    println!(
                        "Navigation stack not found, falling back to simple controller"
                    );
                    self.simple_controller(current_pose);
                }
            } else {
                let mut twist = Twist::default();
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;

                if let Err(e) = self.cmd_vel_publisher.publish(&twist) {
                    eprintln!("Failed to publish velocity command: {}", e);
                }
            }
        }

        fn simple_controller(&self, current_pose: (f64, f64, f64)) {
            if let Some(goal) = *self.current_goal.lock().unwrap() {
                let dx = goal.0 - current_pose.0;
                let dy = goal.1 - current_pose.1;
                let distance = (dx * dx + dy * dy).sqrt();
                let angle_to_goal = dy.atan2(dx);
                let angle_error = angle_to_goal - current_pose.2;

                let angle_error = if angle_error > std::f64::consts::PI {
                    angle_error - 2.0 * std::f64::consts::PI
                } else if angle_error < -std::f64::consts::PI {
                    angle_error + 2.0 * std::f64::consts::PI
                } else {
                    angle_error
                };

                let linear_velocity = if distance > 0.1 {
                    self.max_linear_velocity
                } else {
                    0.0
                };
                let angular_velocity = 0.5 * angle_error;

                println!(
                    "Simple controller: linear={:.2}, angular={:.2}",
                    linear_velocity, angular_velocity
                );

                let mut twist = Twist::default();
                twist.linear.x = linear_velocity;
                twist.angular.z = angular_velocity;

                if let Err(e) = self.cmd_vel_publisher.publish(&twist) {
                    eprintln!("Failed to publish velocity command: {}", e);
                }
            }
        }
    }

    impl Drop for PrometheusNavigationNode {
        fn drop(&mut self) {
            if let Ok(mut running) = self.running.lock() {
                *running = false;
            }
        }
    }

    pub fn run() -> Result<(), Error> {
        println!("Initializing Prometheus Navigation Node...");

        let context = Context::new(std::env::args())?;
        let _node =
            PrometheusNavigationNode::new(&context, "prometheus_navigation_node")?;

        println!("Prometheus Navigation Node initialized. Starting to spin...");
        println!(
            "To use with parameters: ros2 run prometheus_core prometheus_navigation_node \
             --ros-args --params-file /path/to/prometheus_core/config/navigation_params.yaml"
        );

        rclrs::spin(_node.node.clone()).map_err(|err| err.into())
    }
}

fn main() {
    #[cfg(feature = "ros2")]
    {
        if let Err(e) = ros2_node::run() {
            eprintln!("Navigation node error: {}", e);
            std::process::exit(1);
        }
    }

    #[cfg(not(feature = "ros2"))]
    {
        eprintln!("ERROR: prometheus_navigation_node requires the 'ros2' feature.");
        eprintln!();
        eprintln!("This binary needs a ROS2 workspace with rclrs and message packages.");
        eprintln!("Build with:  colcon build --packages-select prometheus_core");
        eprintln!();
        eprintln!("For a standalone demo without ROS2, run:");
        eprintln!("  cargo run --bin nav2_example");
        std::process::exit(1);
    }
}
