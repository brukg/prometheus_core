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

// Import the message types directly from the crates
use geometry_msgs::msg::{PoseStamped, Twist};
use nav_msgs::msg::Odometry;
use nav_msgs::msg::Path;
use sensor_msgs::msg::LaserScan;

struct PrometheusNavigationNode {
    core: Mutex<PrometheusCore>,
    node: Arc<Node>,
    cmd_vel_publisher: Arc<rclrs::Publisher<Twist>>,
    goal_subscription: Mutex<Option<Arc<rclrs::Subscription<PoseStamped>>>>,
    odom_subscription: Mutex<Option<Arc<rclrs::Subscription<Odometry>>>>,
    scan_subscription: Mutex<Option<Arc<rclrs::Subscription<LaserScan>>>>,
    running: Arc<Mutex<bool>>,
    current_path: Arc<Mutex<Option<Vec<(f64, f64)>>>>,
    current_goal: Arc<Mutex<Option<(f64, f64)>>>,
    current_pose: Arc<Mutex<(f64, f64, f64)>>, // (x, y, theta)
    max_linear_velocity: f64,
    path_publisher: Arc<rclrs::Publisher<Path>>,
}

impl PrometheusNavigationNode {
    pub fn new(context: &Context, name: &str) -> Result<Arc<Self>, rclrs::RclrsError> {
        // Create a node using the context
        let node = Node::new(context, name)?;

        // Create the Prometheus Core
        let mut core = PrometheusCore::new();

        // Default parameters
        let lookahead_distance = 0.8;
        let max_linear_velocity = 1.75;
        let cmd_vel_topic = "/prometheus/cmd_vel".to_string();
        let odom_topic = "/prometheus/odom".to_string();
        let goal_topic = "goal_pose".to_string();
        let scan_topic = "/prometheus/scan".to_string();

        // Print parameter values
        println!(
            "Using parameters: lookahead_distance={}, max_linear_velocity={}",
            lookahead_distance, max_linear_velocity
        );
        println!(
            "Topics: cmd_vel={}, odom={}, goal={}, scan={}",
            cmd_vel_topic, odom_topic, goal_topic, scan_topic
        );

        // Create a navigation stack with Pure Pursuit follower
        let mut nav_stack = NavigationStack::with_path_follower(PurePursuitFollower::new());

        // Configure the path follower with parameters
        let mut params = HashMap::new();
        params.insert("lookahead_distance".to_string(), lookahead_distance);
        params.insert("max_linear_velocity".to_string(), max_linear_velocity);

        if let Err(e) = nav_stack.configure_path_follower(&params) {
            eprintln!("Failed to configure path follower: {}", e);
        }

        // Register components
        core.register(nav_stack);
        core.register(ControlStack::new());
        core.register(PerceptionStack::new());

        // Initialize the core
        // if let Err(e) = core.init() {
        //     eprintln!("Failed to initialize core: {}", e);
        //     // Just return a generic RclrsError since we can't convert
        //     return Err(rclrs::RclrsError::RclError);
        // }

        println!("Core initialized successfully!");

        // Create publisher for velocity commands
        let cmd_vel_publisher =
            node.create_publisher::<Twist>(&cmd_vel_topic, QOS_PROFILE_DEFAULT)?;

        // Create publisher for the planned path
        let path_publisher =
            node.create_publisher::<Path>("/prometheus/planned_path", QOS_PROFILE_DEFAULT)?;

        // Create the node instance with a running flag
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
            current_pose: Arc::new(Mutex::new((0.0, 0.0, 0.0))), // Initial pose at origin
            max_linear_velocity,
            path_publisher,
        });

        // Set up goal subscription
        let prometheus_navigation_node_clone = Arc::clone(&prometheus_navigation_node);
        let goal_subscription = prometheus_navigation_node
            .node
            .create_subscription::<PoseStamped, _>(
                &goal_topic,
                QOS_PROFILE_DEFAULT,
                move |msg: PoseStamped| {
                    prometheus_navigation_node_clone.goal_callback(msg);
                },
            )?;

        *prometheus_navigation_node.goal_subscription.lock().unwrap() = Some(goal_subscription);

        // Set up odometry subscription
        let prometheus_navigation_node_clone = Arc::clone(&prometheus_navigation_node);
        let odom_subscription = prometheus_navigation_node
            .node
            .create_subscription::<Odometry, _>(
                &odom_topic,
                QOS_PROFILE_DEFAULT,
                move |msg: Odometry| {
                    prometheus_navigation_node_clone.odom_callback(msg);
                },
            )?;

        *prometheus_navigation_node.odom_subscription.lock().unwrap() = Some(odom_subscription);

        // Set up laser scan subscription
        let prometheus_navigation_node_clone = Arc::clone(&prometheus_navigation_node);
        let scan_subscription = prometheus_navigation_node
            .node
            .create_subscription::<LaserScan, _>(
                &scan_topic,
                QOS_PROFILE_DEFAULT,
                move |msg: LaserScan| {
                    prometheus_navigation_node_clone.scan_callback(msg);
                },
            )?;

        *prometheus_navigation_node.scan_subscription.lock().unwrap() = Some(scan_subscription);

        // Start a thread to periodically publish velocity commands
        let prometheus_navigation_node_clone = Arc::clone(&prometheus_navigation_node);
        let running_clone = Arc::clone(&prometheus_navigation_node.running);

        thread::spawn(move || {
            while *running_clone.lock().unwrap() {
                prometheus_navigation_node_clone.timer_callback();
                thread::sleep(Duration::from_millis(100)); // 10 Hz
            }
        });

        Ok(prometheus_navigation_node)
    }

    fn publish_path(&self, path: &[(f64, f64)]) {
        let mut path_msg = Path::default();
        path_msg.header.frame_id = "map".to_string();

        // Convert path points to PoseStamped messages
        path_msg.poses = path
            .iter()
            .map(|(x, y)| {
                let mut pose_stamped = PoseStamped::default();
                pose_stamped.pose.position.x = *x;
                pose_stamped.pose.position.y = *y;
                pose_stamped.pose.orientation.w = 1.0; // Default orientation
                pose_stamped
            })
            .collect();

        // Publish the path
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

        // Store the goal
        let goal = (msg.pose.position.x, msg.pose.position.y);
        *self.current_goal.lock().unwrap() = Some(goal);

        // Get current pose for path planning
        let current_pose = *self.current_pose.lock().unwrap();

        // Plan path using the navigation stack
        let mut core = self.core.lock().unwrap();
        if let Some(nav_stack) = core.navigation_stack_mut() {
            let path = nav_stack.plan_path((current_pose.0, current_pose.1), goal);
            println!("Planned path with {} points", path.len());

            // Store the path
            *self.current_path.lock().unwrap() = Some(path.clone());

            // Publish the path for visualization
            self.publish_path(&path);
        }
    }

    fn odom_callback(&self, msg: Odometry) {
        // Extract position
        let x = msg.pose.pose.position.x;
        let y = msg.pose.pose.position.y;

        // Extract orientation (quaternion) and convert to yaw (theta)
        let qx = msg.pose.pose.orientation.x;
        let qy = msg.pose.pose.orientation.y;
        let qz = msg.pose.pose.orientation.z;
        let qw = msg.pose.pose.orientation.w;

        // Convert quaternion to Euler angles (we only need yaw/theta)
        let theta = 2.0 * (qw * qz + qx * qy).atan2(1.0 - 2.0 * (qy * qy + qz * qz));

        // Update the current pose
        *self.current_pose.lock().unwrap() = (x, y, theta);

        // Debug output (can be removed in production)
        println!(
            "Updated pose from odom: x={:.2}, y={:.2}, theta={:.2}",
            x, y, theta
        );
    }

    fn scan_callback(&self, msg: LaserScan) {
        // Current robot pose in the map frame
        let current_pose = *self.current_pose.lock().unwrap();
        
        // Process scan data into world points
        let mut world_points = Vec::new();
        let angle_min = msg.angle_min;
        let angle_increment = msg.angle_increment;
        
        for (i, range) in msg.ranges.iter().enumerate() {
            // Skip invalid measurements
            if *range < msg.range_min || *range > msg.range_max {
                continue;
            }
            
            // Convert polar (laser) to cartesian coordinates (relative to robot)
            let angle = angle_min + (angle_increment * i as f32);
            let x_robot = range * angle.cos();
            let y_robot = range * angle.sin();
            
            // Transform to world coordinates using robot's pose
            let x_world = current_pose.0 + (x_robot as f64 * current_pose.2.cos() - y_robot as f64 * current_pose.2.sin());
            let y_world = current_pose.1 + (x_robot as f64 * current_pose.2.sin() + y_robot as f64 * current_pose.2.cos());
            
            world_points.push(WorldPoint { x: x_world, y: y_world });
        }
        
        // Update costmap in the navigation stack
        let mut core = self.core.lock().unwrap();
        if let Some(nav_stack) = core.navigation_stack_mut() {
            if let Err(e) = nav_stack.update_local_map(&world_points, msg.range_max as f64) {
                eprintln!("Failed to update local costmap with laser data: {}", e);
            } else {
                println!("Updated local costmap with {} points from laser scan", world_points.len());
            }
        }
    }

    fn timer_callback(&self) {
        // Get the current path
        let path_option = self.current_path.lock().unwrap().clone();

        if let Some(path) = path_option {
            // We have a path to follow
            let current_pose = *self.current_pose.lock().unwrap();

            // Get the core with the navigation stack
            let mut core = self.core.lock().unwrap();

            // Use the path follower to compute velocity commands
            if let Some(nav_stack) = core.navigation_stack_mut() {
                // Update the path in the navigation stack if needed
                nav_stack.set_path(&path);

                // Get the current pose as a 3D vector (x, y, theta)
                let pose = (current_pose.0, current_pose.1, current_pose.2);

                // Use the path follower to compute velocity commands
                if let Some(cmd) = nav_stack.compute_velocity_command(pose) {
                    // Create and publish the twist message
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

                // Check if we've reached the goal
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
                println!("Navigation stack not found, falling back to simple controller");
                self.simple_controller(current_pose);
            }
        } else {
            // No path to follow, publish zero velocity
            let mut twist = Twist::default();
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;

            if let Err(e) = self.cmd_vel_publisher.publish(&twist) {
                eprintln!("Failed to publish velocity command: {}", e);
            }
        }
    }

    // Simple controller as a fallback
    fn simple_controller(&self, current_pose: (f64, f64, f64)) {
        if let Some(goal) = *self.current_goal.lock().unwrap() {
            // Compute a simple velocity command to move toward the goal

            // Vector from current position to goal
            let dx = goal.0 - current_pose.0;
            let dy = goal.1 - current_pose.1;

            // Distance to goal
            let distance = (dx * dx + dy * dy).sqrt();

            // Angle to goal
            let angle_to_goal = dy.atan2(dx);

            // Angle error
            let angle_error = angle_to_goal - current_pose.2;

            // Normalize angle to [-pi, pi]
            let angle_error = if angle_error > std::f64::consts::PI {
                angle_error - 2.0 * std::f64::consts::PI
            } else if angle_error < -std::f64::consts::PI {
                angle_error + 2.0 * std::f64::consts::PI
            } else {
                angle_error
            };

            // Compute velocity commands
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

            // Create and publish the twist message
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
        // Set running to false when the node is dropped
        if let Ok(mut running) = self.running.lock() {
            *running = false;
        }
    }
}

fn main() -> Result<(), Error> {
    println!("Initializing Prometheus Navigation Node...");

    // Create the ROS 2 context
    // TODO: This will automatically read parameters from the parameter file
    // when launched with --ros-args --params-file /path/to/config/navigation_params.yaml
    let context = Context::new(std::env::args())?;

    // Create the Prometheus navigation node with the correct name
    let prometheus_navigation_node =
        PrometheusNavigationNode::new(&context, "prometheus_navigation_node")?;

    println!("Prometheus Navigation Node initialized. Starting to spin...");
    println!("To use with parameters: ros2 run prometheus_core prometheus_navigation_node --ros-args --params-file /path/to/prometheus_core/config/navigation_params.yaml");

    // Spin the node to process callbacks
    rclrs::spin(prometheus_navigation_node.node.clone()).map_err(|err| err.into())
}
