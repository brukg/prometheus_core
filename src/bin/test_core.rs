use prometheus_core::navigation::path_follower::pure_pursuit::PurePursuitFollower;
use prometheus_core::navigation::path_follower::PathFollower;
use prometheus_core::{
    control::ControlStack, navigation::NavigationStack, perception::PerceptionStack, PrometheusCore,
};
use std::collections::HashMap;

fn main() {
    println!("Initializing Prometheus Core...");

    let mut core = PrometheusCore::new();

    // Create a navigation stack with Pure Pursuit follower
    let mut nav_stack = NavigationStack::with_path_follower(PurePursuitFollower::new());

    // Configure the path follower
    let mut params = HashMap::new();
    params.insert("lookahead_distance".to_string(), 0.8);
    params.insert("max_linear_velocity".to_string(), 0.3);

    if let Err(e) = nav_stack.configure_path_follower(&params) {
        println!("Failed to configure path follower: {}", e);
    }

    // Register components
    core.register(nav_stack);
    core.register(ControlStack::new());
    core.register(PerceptionStack::new());

    // Initialize the core
    match core.init() {
        Ok(_) => println!("Core initialized successfully!"),
        Err(e) => {
            println!("Failed to initialize core: {}", e);
            return;
        }
    }

    // Test navigation and control
    let current_pose = (0.0, 0.0, 0.0);
    let goal = (1.0, 1.0);

    println!("Planning path from {:?} to {:?}", current_pose, goal);

    // In a real application, you would get these components from the core
    // and use them to plan and execute paths
    let test_nav_stack = NavigationStack::with_path_follower(PurePursuitFollower::new());
    println!(
        "Using path follower: {}",
        test_nav_stack.path_follower_name()
    );

    let path = test_nav_stack.plan_path((current_pose.0, current_pose.1), goal);
    println!("Planned path: {:?}", path);

    let (linear_vel, angular_vel) = test_nav_stack.follow_path(current_pose, &path);
    println!(
        "Computed velocities: linear={}, angular={}",
        linear_vel, angular_vel
    );

    // Shutdown the core
    match core.shutdown() {
        Ok(_) => println!("Core shutdown successfully!"),
        Err(e) => println!("Failed to shutdown core: {}", e),
    }
}
