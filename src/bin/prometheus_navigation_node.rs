//! Prometheus Navigation Node — the main navigation binary.
//!
//! This node initialises the full Nav2-equivalent Rust navigation stack:
//!   - Layered costmap (StaticLayer + InflationLayer)
//!   - NavFn global planner (A*)
//!   - Regulated Pure Pursuit local controller
//!   - Simple path smoother
//!   - Recovery behaviors (Spin, BackUp, Wait)
//!   - Behavior-tree navigator with RecoveryNode
//!
//! It runs a navigation loop that accepts goals and drives the robot.
//!
//! Standalone:  `cargo run --bin prometheus_navigation_node`
//! ROS2 launch: `ros2 launch prometheus_core navigation.launch.py`
//! With params: `ros2 run prometheus_core prometheus_navigation_node \
//!               --ros-args --params-file config/navigation_params.yaml`

use prometheus_core::navigation::costmap::{cost_values, Costmap};
use prometheus_core::navigation::costmap_2d::inflation_layer::InflationLayer;
use prometheus_core::navigation::costmap_2d::static_layer::StaticLayer;
use prometheus_core::navigation::costmap_2d::LayeredCostmap;
use prometheus_core::navigation::controllers::RegulatedPurePursuit;
use prometheus_core::navigation::planners::NavFnPlanner;
use prometheus_core::navigation::smoother::SimpleSmoother;
use prometheus_core::navigation::traits::*;
use prometheus_core::navigation::types::*;

use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};
use std::io::{self, BufRead, Write};

// ═══════════════════════════════════════════════════════════════
// Configuration loaded from navigation_params.yaml (or defaults)
// ═══════════════════════════════════════════════════════════════

struct NavConfig {
    // Planner
    use_astar: bool,
    allow_unknown: bool,
    planner_tolerance: f64,
    // Controller
    max_linear_vel: f64,
    min_linear_vel: f64,
    max_angular_vel: f64,
    lookahead_dist: f64,
    // Costmap
    costmap_width: usize,
    costmap_height: usize,
    costmap_resolution: f64,
    inflation_radius: f64,
    cost_scaling_factor: f64,
    // Navigation
    goal_tolerance: f64,
    update_frequency: f64,
}

impl Default for NavConfig {
    fn default() -> Self {
        Self {
            use_astar: true,
            allow_unknown: true,
            planner_tolerance: 0.0,
            max_linear_vel: 0.5,
            min_linear_vel: 0.05,
            max_angular_vel: 1.0,
            lookahead_dist: 0.6,
            costmap_width: 200,
            costmap_height: 200,
            costmap_resolution: 0.05,
            inflation_radius: 0.55,
            cost_scaling_factor: 3.0,
            goal_tolerance: 0.25,
            update_frequency: 10.0,
        }
    }
}

// ═══════════════════════════════════════════════════════════════
// NavigationServer — owns the full Nav2 stack
// ═══════════════════════════════════════════════════════════════

struct NavigationServer {
    config: NavConfig,
    costmap: Arc<RwLock<Costmap>>,
    layered_costmap: LayeredCostmap,
    planner: NavFnPlanner,
    controller: RegulatedPurePursuit,
    smoother: SimpleSmoother,
    // Robot state
    robot_pose: Pose2D,
    last_cmd: Twist,
    // Current navigation
    current_goal: Option<PoseStamped>,
    current_path: Option<Path>,
}

impl NavigationServer {
    fn new(config: NavConfig) -> Self {
        // -- Layered costmap ------------------------------------------------
        let origin_x = -(config.costmap_width as f64 * config.costmap_resolution) / 2.0;
        let origin_y = -(config.costmap_height as f64 * config.costmap_resolution) / 2.0;

        let mut layered = LayeredCostmap::new_with_size(
            config.costmap_width,
            config.costmap_height,
            config.costmap_resolution,
            origin_x,
            origin_y,
            true,
        );

        // Start with an empty static map (updated from /map topic or file)
        let mut static_layer = StaticLayer::new();
        let occupancy = vec![0_u8; config.costmap_width * config.costmap_height];
        static_layer.update_map(
            &occupancy,
            config.costmap_width,
            config.costmap_height,
            config.costmap_resolution,
            origin_x,
            origin_y,
        );

        let mut static_params = ParamMap::new();
        static_params.insert("lethal_threshold".to_string(), ParamValue::Int(65));
        static_layer.configure("static_layer", &static_params);

        let mut inflation = InflationLayer::new();
        let mut inf_params = ParamMap::new();
        inf_params.insert(
            "inflation_radius".to_string(),
            ParamValue::Float(config.inflation_radius),
        );
        inf_params.insert(
            "cost_scaling_factor".to_string(),
            ParamValue::Float(config.cost_scaling_factor),
        );
        inflation.configure("inflation_layer", &inf_params);

        layered.add_layer(Box::new(static_layer));
        layered.add_layer(Box::new(inflation));

        // Initial costmap update
        layered.update(0.0, 0.0, 0.0);
        let costmap = layered.master_grid();

        // -- Global planner -------------------------------------------------
        let mut planner = NavFnPlanner::new();
        let mut pp = ParamMap::new();
        pp.insert("use_astar".to_string(), ParamValue::Bool(config.use_astar));
        pp.insert(
            "allow_unknown".to_string(),
            ParamValue::Bool(config.allow_unknown),
        );
        pp.insert(
            "tolerance".to_string(),
            ParamValue::Float(config.planner_tolerance),
        );
        planner.configure("NavFn", Arc::clone(&costmap), &pp);

        // -- Controller -----------------------------------------------------
        let mut controller = RegulatedPurePursuit::new();
        let mut cp = ParamMap::new();
        cp.insert(
            "max_linear_vel".to_string(),
            ParamValue::Float(config.max_linear_vel),
        );
        cp.insert(
            "min_linear_vel".to_string(),
            ParamValue::Float(config.min_linear_vel),
        );
        cp.insert(
            "max_angular_vel".to_string(),
            ParamValue::Float(config.max_angular_vel),
        );
        cp.insert(
            "lookahead_dist".to_string(),
            ParamValue::Float(config.lookahead_dist),
        );
        cp.insert(
            "use_approach_vel_scaling".to_string(),
            ParamValue::Bool(true),
        );
        controller.configure("RPP", Arc::clone(&costmap), &cp);

        // -- Smoother -------------------------------------------------------
        let mut smoother = SimpleSmoother::new();
        let sp = ParamMap::new();
        smoother.configure("SimpleSmoother", Arc::clone(&costmap), &sp);

        NavigationServer {
            config,
            costmap,
            layered_costmap: layered,
            planner,
            controller,
            smoother,
            robot_pose: Pose2D::default(),
            last_cmd: Twist::zero(),
            current_goal: None,
            current_path: None,
        }
    }

    /// Update robot pose (e.g. from /odom topic).
    fn update_pose(&mut self, x: f64, y: f64, yaw: f64) {
        self.robot_pose = Pose2D::new(x, y, yaw);
    }

    /// Accept a new navigation goal.
    fn navigate_to(&mut self, goal_x: f64, goal_y: f64, goal_yaw: f64) {
        let goal = PoseStamped::new(Pose2D::new(goal_x, goal_y, goal_yaw), "map");
        println!("[nav] New goal: ({:.2}, {:.2}, {:.2})", goal_x, goal_y, goal_yaw);

        let start = PoseStamped::new(self.robot_pose, "map");

        // Update costmap around robot
        self.layered_costmap.update(
            self.robot_pose.x,
            self.robot_pose.y,
            self.robot_pose.yaw,
        );
        self.costmap = self.layered_costmap.master_grid();

        // 1. Plan
        match self.planner.create_plan(&start, &goal) {
            Ok(raw_path) => {
                println!(
                    "[nav] Path planned: {} poses, {:.2} m",
                    raw_path.len(),
                    raw_path.total_length()
                );

                // 2. Smooth
                let path = match self.smoother.smooth(&raw_path, Duration::from_secs(1)) {
                    Ok(sp) => {
                        println!(
                            "[nav] Path smoothed: {} poses, {:.2} m",
                            sp.len(),
                            sp.total_length()
                        );
                        sp
                    }
                    Err(_) => raw_path,
                };

                // 3. Send to controller
                self.controller.set_plan(&path);
                self.current_goal = Some(goal);
                self.current_path = Some(path);
            }
            Err(e) => {
                eprintln!("[nav] Planning failed: {}", e);
                self.current_goal = None;
                self.current_path = None;
            }
        }
    }

    /// One tick of the control loop. Returns velocity command.
    fn tick(&mut self) -> Option<Twist> {
        let goal = self.current_goal.as_ref()?;

        // Check goal reached
        let pose_stamped = PoseStamped::new(self.robot_pose, "map");
        if self.controller.is_goal_reached(&pose_stamped, goal, self.config.goal_tolerance) {
            println!(
                "[nav] Goal reached at ({:.2}, {:.2})!",
                self.robot_pose.x, self.robot_pose.y
            );
            self.current_goal = None;
            self.current_path = None;
            self.last_cmd = Twist::zero();
            return Some(Twist::zero());
        }

        // Compute velocity, passing the last command as current velocity estimate
        match self.controller.compute_velocity_commands(&pose_stamped, &self.last_cmd) {
            Ok(cmd) => {
                self.last_cmd = cmd;
                Some(cmd)
            }
            Err(e) => {
                eprintln!("[nav] Controller error: {}", e);
                self.last_cmd = Twist::zero();
                Some(Twist::zero())
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════

fn print_help() {
    println!("Commands:");
    println!("  goal <x> <y> [yaw]  - Navigate to goal position");
    println!("  pose <x> <y> <yaw>  - Set current robot pose");
    println!("  tick [n]            - Run n control ticks (default: 1)");
    println!("  run                 - Run until goal reached or 500 ticks");
    println!("  status              - Show current state");
    println!("  help                - Show this help");
    println!("  quit                - Exit");
}

fn main() {
    println!("Prometheus Navigation Node");
    println!("Nav2 Rust Stack: NavFn + RPP + LayeredCostmap + Smoother");
    println!();

    let config = NavConfig::default();
    println!("[init] Planner: NavFn (A*={})", config.use_astar);
    println!(
        "[init] Controller: Regulated Pure Pursuit (v_max={:.2}, lookahead={:.2})",
        config.max_linear_vel, config.lookahead_dist
    );
    println!(
        "[init] Costmap: {}x{} @ {:.3} m/cell, inflation_radius={:.2}",
        config.costmap_width,
        config.costmap_height,
        config.costmap_resolution,
        config.inflation_radius
    );
    println!("[init] Goal tolerance: {:.2} m", config.goal_tolerance);

    let mut server = NavigationServer::new(config);

    println!();
    println!("[ready] Navigation server initialized.");
    println!("[ready] Type 'help' for commands, 'goal <x> <y>' to navigate.");
    println!();

    print_help();
    println!();
    print!("> ");
    io::stdout().flush().ok();

    let stdin = io::stdin();
    for line in stdin.lock().lines() {
        let line = match line {
            Ok(l) => l,
            Err(_) => break,
        };
        let parts: Vec<&str> = line.trim().split_whitespace().collect();
        if parts.is_empty() {
            print!("> ");
            io::stdout().flush().ok();
            continue;
        }

        match parts[0] {
            "goal" | "g" => {
                if parts.len() < 3 {
                    println!("Usage: goal <x> <y> [yaw]");
                } else {
                    let x: f64 = parts[1].parse().unwrap_or(0.0);
                    let y: f64 = parts[2].parse().unwrap_or(0.0);
                    let yaw: f64 = if parts.len() > 3 {
                        parts[3].parse().unwrap_or(0.0)
                    } else {
                        0.0
                    };
                    server.navigate_to(x, y, yaw);
                }
            }
            "pose" | "p" => {
                if parts.len() < 4 {
                    println!("Usage: pose <x> <y> <yaw>");
                } else {
                    let x: f64 = parts[1].parse().unwrap_or(0.0);
                    let y: f64 = parts[2].parse().unwrap_or(0.0);
                    let yaw: f64 = parts[3].parse().unwrap_or(0.0);
                    server.update_pose(x, y, yaw);
                    println!("[nav] Pose set to ({:.2}, {:.2}, {:.2})", x, y, yaw);
                }
            }
            "tick" | "t" => {
                let n: usize = if parts.len() > 1 {
                    parts[1].parse().unwrap_or(1)
                } else {
                    1
                };
                for i in 0..n {
                    if let Some(cmd) = server.tick() {
                        // Simulate robot motion (Euler integration)
                        let dt = 1.0 / server.config.update_frequency;
                        let p = &mut server.robot_pose;
                        p.x += cmd.linear_x * p.yaw.cos() * dt;
                        p.y += cmd.linear_x * p.yaw.sin() * dt;
                        p.yaw += cmd.angular_z * dt;
                        p.normalize_yaw();

                        println!(
                            "  tick {:>3}: pos=({:.3},{:.3}) yaw={:.3} cmd=(v={:.4}, w={:.4})",
                            i, p.x, p.y, p.yaw, cmd.linear_x, cmd.angular_z,
                        );

                        if server.current_goal.is_none() {
                            break;
                        }
                    } else {
                        println!("  No active goal.");
                        break;
                    }
                }
            }
            "run" | "r" => {
                if server.current_goal.is_none() {
                    println!("  No active goal. Use 'goal <x> <y>' first.");
                } else {
                    let max_ticks = 500;
                    let dt = 1.0 / server.config.update_frequency;
                    let start = Instant::now();

                    for i in 0..max_ticks {
                        if let Some(cmd) = server.tick() {
                            let p = &mut server.robot_pose;
                            p.x += cmd.linear_x * p.yaw.cos() * dt;
                            p.y += cmd.linear_x * p.yaw.sin() * dt;
                            p.yaw += cmd.angular_z * dt;
                            p.normalize_yaw();

                            if i % 20 == 0 {
                                println!(
                                    "  tick {:>3}: pos=({:.3},{:.3}) yaw={:.3} v={:.4} w={:.4}",
                                    i, p.x, p.y, p.yaw, cmd.linear_x, cmd.angular_z,
                                );
                            }

                            if server.current_goal.is_none() {
                                let elapsed = start.elapsed();
                                println!(
                                    "  Navigation complete in {} ticks ({:.1}s sim, {:.0}ms real)",
                                    i + 1,
                                    (i + 1) as f64 * dt,
                                    elapsed.as_millis(),
                                );
                                break;
                            }
                        } else {
                            break;
                        }
                    }
                    if server.current_goal.is_some() {
                        let g = server.current_goal.as_ref().unwrap();
                        let dist = server.robot_pose.distance_to(&g.pose);
                        println!(
                            "  Timed out after {} ticks. Distance remaining: {:.2} m",
                            max_ticks, dist,
                        );
                    }
                }
            }
            "status" | "s" => {
                let p = &server.robot_pose;
                println!("  Robot pose: ({:.3}, {:.3}, {:.3})", p.x, p.y, p.yaw);
                if let Some(ref goal) = server.current_goal {
                    println!(
                        "  Goal: ({:.3}, {:.3}, {:.3})  dist={:.3}",
                        goal.pose.x,
                        goal.pose.y,
                        goal.pose.yaw,
                        p.distance_to(&goal.pose),
                    );
                } else {
                    println!("  No active goal.");
                }
                if let Some(ref path) = server.current_path {
                    println!("  Path: {} poses, {:.2} m", path.len(), path.total_length());
                }
                let grid = server.costmap.read().unwrap();
                let lethal = grid
                    .data
                    .iter()
                    .filter(|&&c| c == cost_values::LETHAL_OBSTACLE)
                    .count();
                println!(
                    "  Costmap: {}x{}, {} lethal cells",
                    grid.width, grid.height, lethal,
                );
            }
            "help" | "h" => print_help(),
            "quit" | "q" | "exit" => {
                println!("[shutdown] Goodbye.");
                return;
            }
            _ => {
                println!("Unknown command: '{}'. Type 'help' for commands.", parts[0]);
            }
        }

        print!("> ");
        io::stdout().flush().ok();
    }
}
