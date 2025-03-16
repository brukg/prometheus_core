Examples
========

Basic Example: Test Core
-----------------------

The package includes a simple test program that demonstrates the core functionality without ROS 2:

.. code-block:: bash

   cargo run --bin test_core

This example initializes the core components, plans a simple path, and computes velocity commands.

ROS 2 Node Example
----------------

Here's a complete example of using the Prometheus node with ROS 2:

1. Start the Prometheus node:

   .. code-block:: bash

      ros2 run prometheus_core prometheus_navigation_node

2. In another terminal, send a goal:

   .. code-block:: bash

      ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "
      header:
        stamp:
          sec: 0
          nanosec: 0
        frame_id: 'map'
      pose:
        position:
          x: 5.0
          y: 3.0
          z: 0.0
        orientation:
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0"

3. Monitor the velocity commands:

   .. code-block:: bash

      ros2 topic echo /cmd_vel

Custom Path Follower Example
--------------------------

You can create your own path follower by implementing the ``PathFollower`` trait:

.. code-block:: rust

   use prometheus_core::navigation::path_follower::PathFollower;
   use std::collections::HashMap;
   
   #[derive(Debug)]
   struct MyCustomFollower {
       max_speed: f64,
   }
   
   impl PathFollower for MyCustomFollower {
       fn new() -> Self {
           MyCustomFollower { max_speed: 0.5 }
       }
       
       fn compute_velocity(&self, current_pose: (f64, f64, f64), path: &[(f64, f64)]) -> (f64, f64) {
           // Your custom algorithm here
           (0.2, 0.1) // Example linear and angular velocity
       }
       
       fn name(&self) -> &str {
           "MyCustomFollower"
       }
       
       fn configure(&mut self, params: &HashMap<String, f64>) -> Result<(), String> {
           if let Some(&max_speed) = params.get("max_speed") {
               self.max_speed = max_speed;
           }
           Ok(())
       }
   }
   
   // Use it with the navigation stack
   let mut nav_stack = NavigationStack::with_path_follower(MyCustomFollower::new()); 