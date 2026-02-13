//! Nav2 Rust Port — standalone example / demo binary.
//!
//! Demonstrates all Nav2-equivalent components without a ROS2 workspace:
//!   - Layered Costmap (StaticLayer + InflationLayer)
//!   - NavFn global planner (Dijkstra / A*)
//!   - Regulated Pure Pursuit (RPP) controller
//!   - Simple path smoother
//!   - Recovery behaviors (Spin, BackUp, Wait)
//!   - Behavior tree engine with Nav2-style RecoveryNode
//!
//! Run: `cargo run --bin nav2_example`

use prometheus_core::navigation::costmap::{cost_values, Costmap};
use prometheus_core::navigation::costmap_2d::inflation_layer::InflationLayer;
use prometheus_core::navigation::costmap_2d::static_layer::StaticLayer;
use prometheus_core::navigation::costmap_2d::LayeredCostmap;
use prometheus_core::navigation::planners::NavFnPlanner;
use prometheus_core::navigation::controllers::RegulatedPurePursuit;
use prometheus_core::navigation::smoother::SimpleSmoother;
use prometheus_core::navigation::nav_behaviors::{SpinBehavior, BackUpBehavior, WaitBehavior};
use prometheus_core::navigation::bt::{BtNode, BtStatus, Blackboard, BtAction};
use prometheus_core::navigation::traits::*;
use prometheus_core::navigation::types::*;

use std::sync::{Arc, RwLock};
use std::time::Duration;

// ─────────────────────────────────────────────────────────────
// Helper: build a costmap with walls and an inflation layer
// ─────────────────────────────────────────────────────────────

fn build_costmap() -> Arc<RwLock<Costmap>> {
    let width = 200;
    let height = 200;
    let resolution = 0.05; // 5 cm cells → 10 m × 10 m world
    let origin_x = 0.0;
    let origin_y = 0.0;

    // -- Create the layered costmap ----------------------------------------
    let mut layered = LayeredCostmap::new_with_size(
        width, height, resolution, origin_x, origin_y, true,
    );

    // Static layer: occupancy grid with walls
    let mut occupancy = vec![0_u8; width * height]; // all free

    // Wall across the middle (y = 100, x = 40..160) leaving gaps at the edges
    for x in 40..160 {
        occupancy[100 * width + x] = 100; // lethal
    }

    // Small box obstacle at (3.0, 3.0) world → grid (60, 60)
    for dy in 0..6 {
        for dx in 0..6 {
            let gx = 60 + dx;
            let gy = 60 + dy;
            occupancy[gy * width + gx] = 100;
        }
    }

    let mut static_layer = StaticLayer::new();
    static_layer.update_map(&occupancy, width, height, resolution, origin_x, origin_y);

    let mut static_params = ParamMap::new();
    static_params.insert("lethal_threshold".to_string(), ParamValue::Int(100));
    static_layer.configure("static_layer", &static_params);

    // Inflation layer
    let mut inflation = InflationLayer::new();
    let mut inf_params = ParamMap::new();
    inf_params.insert("inflation_radius".to_string(), ParamValue::Float(0.55));
    inf_params.insert("cost_scaling_factor".to_string(), ParamValue::Float(3.0));
    inflation.configure("inflation_layer", &inf_params);

    layered.add_layer(Box::new(static_layer));
    layered.add_layer(Box::new(inflation));

    // Run one update cycle
    layered.update(1.0, 1.0, 0.0);

    layered.master_grid()
}

fn print_section(title: &str) {
    println!();
    println!("══════════════════════════════════════════════════════════════");
    println!("  {}", title);
    println!("══════════════════════════════════════════════════════════════");
}

// ─────────────────────────────────────────────────────────────
// Demo 1: Layered Costmap
// ─────────────────────────────────────────────────────────────

fn demo_layered_costmap(costmap: &Arc<RwLock<Costmap>>) {
    print_section("Demo 1: Layered Costmap (StaticLayer + InflationLayer)");

    let grid = costmap.read().unwrap();
    println!("  Grid size: {}×{} cells", grid.width, grid.height);
    println!("  Resolution: {} m/cell", grid.resolution);
    println!(
        "  World extent: ({}, {}) to ({}, {})",
        grid.origin_x,
        grid.origin_y,
        grid.origin_x + grid.width as f64 * grid.resolution,
        grid.origin_y + grid.height as f64 * grid.resolution,
    );

    // Count cell types
    let lethal = grid.data.iter().filter(|&&c| c == cost_values::LETHAL_OBSTACLE).count();
    let inscribed = grid.data.iter().filter(|&&c| c == cost_values::INSCRIBED_INFLATED_OBSTACLE).count();
    let inflated = grid.data.iter().filter(|&&c| c > cost_values::NO_COST && c < cost_values::INSCRIBED_INFLATED_OBSTACLE).count();
    let free = grid.data.iter().filter(|&&c| c == cost_values::NO_COST).count();

    println!("  Cell statistics:");
    println!("    Lethal (254):    {:>6}", lethal);
    println!("    Inscribed (253): {:>6}", inscribed);
    println!("    Inflated (1-252):{:>6}", inflated);
    println!("    Free (0):        {:>6}", free);

    // Sample costs at specific world locations
    let test_points = [
        (1.0, 1.0, "open space"),
        (3.0, 3.0, "near box obstacle"),
        (4.0, 5.0, "on wall"),
        (0.5, 5.0, "gap beside wall"),
    ];
    println!("  Sample costs:");
    for (x, y, label) in &test_points {
        let cost = grid.get_cost(*x, *y);
        println!("    ({:.1}, {:.1}) {}: cost={}", x, y, label, cost);
    }
}

// ─────────────────────────────────────────────────────────────
// Demo 2: NavFn Global Planner (Dijkstra + A*)
// ─────────────────────────────────────────────────────────────

fn demo_navfn_planner(costmap: &Arc<RwLock<Costmap>>) {
    print_section("Demo 2: NavFn Global Planner");

    let start = PoseStamped::new(Pose2D::new(1.0, 1.0, 0.0), "map");
    let goal  = PoseStamped::new(Pose2D::new(1.0, 8.0, 0.0), "map");

    // --- Dijkstra ---
    let mut dijkstra = NavFnPlanner::new();
    let mut params = ParamMap::new();
    params.insert("use_astar".to_string(), ParamValue::Bool(false));
    params.insert("allow_unknown".to_string(), ParamValue::Bool(true));
    params.insert("tolerance".to_string(), ParamValue::Float(0.0));
    dijkstra.configure("NavFn_Dijkstra", Arc::clone(costmap), &params);

    println!("  Start: ({:.1}, {:.1})", start.pose.x, start.pose.y);
    println!("  Goal:  ({:.1}, {:.1})", goal.pose.x, goal.pose.y);

    match dijkstra.create_plan(&start, &goal) {
        Ok(path) => {
            println!("  [Dijkstra] Path found: {} poses, length={:.2} m",
                     path.len(), path.total_length());
            print_path_preview(&path);
        }
        Err(e) => println!("  [Dijkstra] Planning failed: {}", e),
    }

    // --- A* ---
    let mut astar = NavFnPlanner::new();
    let mut params = ParamMap::new();
    params.insert("use_astar".to_string(), ParamValue::Bool(true));
    params.insert("allow_unknown".to_string(), ParamValue::Bool(true));
    params.insert("tolerance".to_string(), ParamValue::Float(0.0));
    astar.configure("NavFn_AStar", Arc::clone(costmap), &params);

    match astar.create_plan(&start, &goal) {
        Ok(path) => {
            println!("  [A*]       Path found: {} poses, length={:.2} m",
                     path.len(), path.total_length());
            print_path_preview(&path);
        }
        Err(e) => println!("  [A*]       Planning failed: {}", e),
    }
}

fn print_path_preview(path: &Path) {
    let n = path.len();
    if n == 0 { return; }

    let show = 5.min(n);
    print!("             First {} points: ", show);
    for i in 0..show {
        let p = &path.poses[i].pose;
        print!("({:.2},{:.2}) ", p.x, p.y);
    }
    if n > show {
        print!("... ({} more)", n - show);
    }
    println!();
}

// ─────────────────────────────────────────────────────────────
// Demo 3: Regulated Pure Pursuit Controller
// ─────────────────────────────────────────────────────────────

fn demo_rpp_controller(costmap: &Arc<RwLock<Costmap>>) {
    print_section("Demo 3: Regulated Pure Pursuit Controller");

    let mut rpp = RegulatedPurePursuit::new();
    let mut params = ParamMap::new();
    params.insert("max_linear_vel".to_string(), ParamValue::Float(0.5));
    params.insert("lookahead_dist".to_string(), ParamValue::Float(0.6));
    params.insert("use_approach_vel_scaling".to_string(), ParamValue::Bool(true));
    rpp.configure("RPP", Arc::clone(costmap), &params);

    // Build a simple straight-ahead path
    let path = Path::from_poses(
        (0..20)
            .map(|i| PoseStamped::new(Pose2D::new(1.0, 0.5 + i as f64 * 0.3, 0.0), "map"))
            .collect(),
        "map",
    );
    rpp.set_plan(&path);

    // Simulate several ticks with the robot at the start
    println!("  Path: {} poses, length={:.2} m", path.len(), path.total_length());
    println!("  Simulating 5 controller ticks:");

    let mut pose = PoseStamped::new(Pose2D::new(1.0, 0.5, std::f64::consts::FRAC_PI_2), "map");
    let velocity = Twist::zero();

    for tick in 0..5 {
        match rpp.compute_velocity_commands(&pose, &velocity) {
            Ok(cmd) => {
                println!(
                    "    tick {}: pose=({:.2},{:.2},{:.2}) -> cmd: linear={:.3}, angular={:.3}",
                    tick, pose.pose.x, pose.pose.y, pose.pose.yaw,
                    cmd.linear_x, cmd.angular_z,
                );
                // Advance pose (simple Euler integration, dt=0.1s)
                let dt = 0.1;
                pose.pose.x += cmd.linear_x * pose.pose.yaw.cos() * dt;
                pose.pose.y += cmd.linear_x * pose.pose.yaw.sin() * dt;
                pose.pose.yaw += cmd.angular_z * dt;
            }
            Err(e) => println!("    tick {}: error — {}", tick, e),
        }
    }

    // Goal reached check
    let goal = path.poses.last().unwrap();
    let reached = rpp.is_goal_reached(&pose, goal, 0.25);
    println!("  Goal reached (tolerance 0.25 m): {}", reached);
}

// ─────────────────────────────────────────────────────────────
// Demo 4: Simple Path Smoother
// ─────────────────────────────────────────────────────────────

fn demo_smoother(costmap: &Arc<RwLock<Costmap>>) {
    print_section("Demo 4: Simple Path Smoother");

    let mut smoother = SimpleSmoother::new();
    let params = ParamMap::new();
    smoother.configure("SimpleSmoother", Arc::clone(costmap), &params);

    // A zigzag path that begs for smoothing
    let zigzag = Path::from_poses(
        vec![
            PoseStamped::new(Pose2D::new(0.5, 0.5, 0.0), "map"),
            PoseStamped::new(Pose2D::new(1.0, 1.0, 0.0), "map"),
            PoseStamped::new(Pose2D::new(1.5, 0.6, 0.0), "map"),
            PoseStamped::new(Pose2D::new(2.0, 1.2, 0.0), "map"),
            PoseStamped::new(Pose2D::new(2.5, 0.7, 0.0), "map"),
            PoseStamped::new(Pose2D::new(3.0, 1.0, 0.0), "map"),
        ],
        "map",
    );

    println!("  Original path ({} poses):", zigzag.len());
    for p in &zigzag.poses {
        println!("    ({:.2}, {:.2})", p.pose.x, p.pose.y);
    }

    match smoother.smooth(&zigzag, Duration::from_secs(1)) {
        Ok(smoothed) => {
            println!("  Smoothed path ({} poses):", smoothed.len());
            for p in &smoothed.poses {
                println!("    ({:.2}, {:.2})", p.pose.x, p.pose.y);
            }

            let orig_dev: f64 = zigzag.poses.iter().skip(1).take(zigzag.len() - 2)
                .map(|p| (p.pose.y - 0.85).abs())
                .sum();
            let smooth_dev: f64 = smoothed.poses.iter().skip(1).take(smoothed.len() - 2)
                .map(|p| (p.pose.y - 0.85).abs())
                .sum();
            println!("  Y-deviation from mean: original={:.3}, smoothed={:.3}", orig_dev, smooth_dev);
        }
        Err(e) => println!("  Smoothing failed: {}", e),
    }
}

// ─────────────────────────────────────────────────────────────
// Demo 5: Recovery Behaviors
// ─────────────────────────────────────────────────────────────

fn demo_behaviors() {
    print_section("Demo 5: Recovery Behaviors (Spin, BackUp, Wait)");

    // Spin
    let mut spin = SpinBehavior::new(1.0);
    let vel = spin.get_velocity();
    println!("  SpinBehavior(target_yaw=1.0):");
    println!("    velocity: linear={:.2}, angular={:.2}", vel.linear_x, vel.angular_z);
    let mut ticks = 0;
    loop {
        let status = spin.execute();
        ticks += 1;
        if status != BehaviorStatus::Running || ticks > 200 {
            println!("    Completed after {} ticks: {:?}", ticks, status);
            break;
        }
    }

    // BackUp
    let mut backup = BackUpBehavior::new(0.3);
    let vel = backup.get_velocity();
    println!("  BackUpBehavior(target_distance=0.3):");
    println!("    velocity: linear={:.2}, angular={:.2}", vel.linear_x, vel.angular_z);
    let mut ticks = 0;
    loop {
        let status = backup.execute();
        ticks += 1;
        if status != BehaviorStatus::Running || ticks > 200 {
            println!("    Completed after {} ticks: {:?}", ticks, status);
            break;
        }
    }

    // Wait
    let mut wait = WaitBehavior::new(Duration::from_millis(50));
    println!("  WaitBehavior(duration=50ms):");
    let s1 = wait.execute();
    println!("    tick 1: {:?}", s1);
    std::thread::sleep(Duration::from_millis(60));
    let s2 = wait.execute();
    println!("    tick 2 (after 60ms sleep): {:?}", s2);
}

// ─────────────────────────────────────────────────────────────
// Demo 6: Behavior Tree Engine
// ─────────────────────────────────────────────────────────────

/// A simple BT action that succeeds after N ticks.
struct CountdownAction {
    name: String,
    ticks_remaining: u32,
    initial: u32,
}

impl CountdownAction {
    fn new(name: &str, ticks: u32) -> Self {
        Self { name: name.to_string(), ticks_remaining: ticks, initial: ticks }
    }
}

impl BtAction for CountdownAction {
    fn tick(&mut self, _bb: &mut Blackboard) -> BtStatus {
        if self.ticks_remaining == 0 {
            self.ticks_remaining = self.initial; // reset for next use
            return BtStatus::Success;
        }
        self.ticks_remaining -= 1;
        BtStatus::Running
    }
    fn halt(&mut self) {
        self.ticks_remaining = self.initial;
    }
    fn name(&self) -> &str { &self.name }
}

/// A BT action that always fails (simulates a planner failure).
struct AlwaysFail { name: String }
impl AlwaysFail {
    fn new(name: &str) -> Self { Self { name: name.to_string() } }
}
impl BtAction for AlwaysFail {
    fn tick(&mut self, _bb: &mut Blackboard) -> BtStatus { BtStatus::Failure }
    fn halt(&mut self) {}
    fn name(&self) -> &str { &self.name }
}

fn demo_behavior_tree() {
    print_section("Demo 6: Behavior Tree Engine");

    // Build a Nav2-style tree:
    //   RecoveryNode(max_retries=2)
    //     main: Sequence [ ComputePath(2 ticks), FollowPath(3 ticks) ]
    //     recovery: SpinRecovery(1 tick)
    //
    // The sequence should succeed after 2+3 = 5 ticks of Running then Success.
    let mut bb = Blackboard::new();
    bb.set("goal_x", 5.0_f64);
    bb.set("goal_y", 8.0_f64);

    let compute = BtNode::Action(Box::new(CountdownAction::new("ComputePathToPose", 2)));
    let follow  = BtNode::Action(Box::new(CountdownAction::new("FollowPath", 3)));
    let main_seq = BtNode::Sequence(vec![compute, follow]);

    let spin_recovery = BtNode::Action(Box::new(CountdownAction::new("Spin", 1)));

    let mut tree = BtNode::RecoveryNode {
        main: Box::new(main_seq),
        recovery: Box::new(spin_recovery),
        max_retries: 2,
        current_retries: 0,
    };

    println!("  Tree: RecoveryNode {{ Sequence[ComputePath, FollowPath], Spin }}");
    println!("  Blackboard: goal_x={}, goal_y={}",
             bb.get::<f64>("goal_x").unwrap(),
             bb.get::<f64>("goal_y").unwrap());
    println!("  Ticking:");

    for tick in 0..10 {
        let status = tree.tick(&mut bb);
        println!("    tick {}: {:?}", tick, status);
        if status == BtStatus::Success || status == BtStatus::Failure {
            break;
        }
    }

    // Second demo: Fallback with failure + recovery
    println!();
    println!("  -- Fallback demo: [AlwaysFail, CountdownAction(1)] --");

    let fail_action = BtNode::Action(Box::new(AlwaysFail::new("BrokenPlanner")));
    let backup_plan = BtNode::Action(Box::new(CountdownAction::new("BackupPlanner", 1)));
    let mut fallback = BtNode::Fallback(vec![fail_action, backup_plan]);

    for tick in 0..5 {
        let status = fallback.tick(&mut bb);
        println!("    tick {}: {:?}", tick, status);
        if status == BtStatus::Success || status == BtStatus::Failure {
            break;
        }
    }
}

// ─────────────────────────────────────────────────────────────
// Demo 7: Full Navigation Pipeline (plan → smooth → control)
// ─────────────────────────────────────────────────────────────

fn demo_full_pipeline(costmap: &Arc<RwLock<Costmap>>) {
    print_section("Demo 7: Full Navigation Pipeline (Plan -> Smooth -> Control)");

    let start = PoseStamped::new(Pose2D::new(1.0, 1.0, 0.0), "map");
    let goal  = PoseStamped::new(Pose2D::new(1.0, 8.0, 0.0), "map");

    // 1. Plan
    let mut planner = NavFnPlanner::new();
    let mut pp = ParamMap::new();
    pp.insert("use_astar".to_string(), ParamValue::Bool(true));
    pp.insert("allow_unknown".to_string(), ParamValue::Bool(true));
    planner.configure("NavFn", Arc::clone(costmap), &pp);

    let raw_path = match planner.create_plan(&start, &goal) {
        Ok(p) => {
            println!("  1. Planned path: {} poses, {:.2} m", p.len(), p.total_length());
            p
        }
        Err(e) => {
            println!("  1. Planning failed: {} — aborting pipeline", e);
            return;
        }
    };

    // 2. Smooth
    let mut smoother = SimpleSmoother::new();
    let sp = ParamMap::new();
    smoother.configure("SimpleSmoother", Arc::clone(costmap), &sp);

    let smooth_path = match smoother.smooth(&raw_path, Duration::from_secs(1)) {
        Ok(p) => {
            println!("  2. Smoothed path: {} poses, {:.2} m", p.len(), p.total_length());
            p
        }
        Err(e) => {
            println!("  2. Smoothing failed: {} — using raw path", e);
            raw_path.clone()
        }
    };

    // 3. Control loop
    let mut controller = RegulatedPurePursuit::new();
    let mut cp = ParamMap::new();
    cp.insert("max_linear_vel".to_string(), ParamValue::Float(0.5));
    cp.insert("lookahead_dist".to_string(), ParamValue::Float(0.6));
    controller.configure("RPP", Arc::clone(costmap), &cp);
    controller.set_plan(&smooth_path);

    let goal_pose = smooth_path.poses.last().unwrap().clone();
    let mut pose = start.clone();
    pose.pose.yaw = std::f64::consts::FRAC_PI_2; // facing +Y
    let vel = Twist::zero();

    println!("  3. Running control loop (max 100 ticks, dt=0.1s):");
    let mut reached = false;
    for tick in 0..100 {
        match controller.compute_velocity_commands(&pose, &vel) {
            Ok(cmd) => {
                // Euler step
                let dt = 0.1;
                pose.pose.x += cmd.linear_x * pose.pose.yaw.cos() * dt;
                pose.pose.y += cmd.linear_x * pose.pose.yaw.sin() * dt;
                pose.pose.yaw += cmd.angular_z * dt;

                if tick % 10 == 0 {
                    println!(
                        "     tick {:>3}: pos=({:.2},{:.2}) yaw={:.2} cmd=(v={:.3}, w={:.3})",
                        tick, pose.pose.x, pose.pose.y, pose.pose.yaw,
                        cmd.linear_x, cmd.angular_z,
                    );
                }
            }
            Err(e) => {
                println!("     tick {:>3}: controller error — {}", tick, e);
                break;
            }
        }

        if controller.is_goal_reached(&pose, &goal_pose, 0.3) {
            println!("     -> Goal reached at tick {}! Final pos: ({:.2},{:.2})",
                     tick, pose.pose.x, pose.pose.y);
            reached = true;
            break;
        }
    }

    if !reached {
        let dist = pose.pose.distance_to(&goal_pose.pose);
        println!("     -> Did not reach goal in 100 ticks. Distance remaining: {:.2} m", dist);
    }
}

// ─────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────

fn main() {
    println!("┌─────────────────────────────────────────────────────────┐");
    println!("│  Prometheus Core — Nav2 Rust Port Demo                  │");
    println!("│  Standalone example (no ROS2 workspace required)        │");
    println!("└─────────────────────────────────────────────────────────┘");

    // Build a shared costmap used by most demos
    let costmap = build_costmap();

    demo_layered_costmap(&costmap);
    demo_navfn_planner(&costmap);
    demo_rpp_controller(&costmap);
    demo_smoother(&costmap);
    demo_behaviors();
    demo_behavior_tree();
    demo_full_pipeline(&costmap);

    print_section("All demos complete");
    println!();
    println!("  To run with ROS2:");
    println!("    colcon build --packages-select prometheus_core");
    println!("    ros2 run prometheus_core prometheus_navigation_node");
    println!("    ros2 run prometheus_core prometheus_navigation_node \\");
    println!("      --ros-args --params-file config/navigation_params.yaml");
    println!();
}
