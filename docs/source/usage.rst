Usage
=====

Running the Prometheus Node
--------------------------

**Using the Launch File (Recommended)**

After building the package, you can launch the Prometheus node with:

.. code-block:: bash

   ros2 launch prometheus_core navigation.launch.py

This will start the navigation node with the default configuration from ``config/navigation_params.yaml``.

To use a custom parameter file:

.. code-block:: bash

   ros2 launch prometheus_core navigation.launch.py params_file:=/path/to/custom_params.yaml

**Direct Execution**

Alternatively, you can run the node directly:

.. code-block:: bash

   ros2 run prometheus_core prometheus_navigation_node

The node will initialize the core components (navigation, control, and perception stacks) and start listening for goal messages.

Publishing Goals
--------------

You can send a goal to the robot using the ``goal_pose`` topic:

.. code-block:: bash

   ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "
   header:
     stamp:
       sec: 0
       nanosec: 0
     frame_id: 'map'
   pose:
     position:
       x: 1.0
       y: 2.0
       z: 0.0
     orientation:
       x: 0.0
       y: 0.0
       z: 0.0
       w: 1.0"

Topics
------

**Subscribed Topics:**

- ``/prometheus/odom`` (nav_msgs/Odometry) - Robot odometry for localization
- ``/prometheus/scan`` (sensor_msgs/LaserScan) - Laser scan data for obstacle detection
- ``/goal_pose`` (geometry_msgs/PoseStamped) - Goal poses for navigation

**Published Topics:**

- ``/prometheus/cmd_vel`` (geometry_msgs/Twist) - Velocity commands for the robot
- ``/prometheus/planned_path`` (nav_msgs/Path) - Planned path for visualization

Monitoring Topics
-----------------

You can monitor the node's activity by echoing various topics:

.. code-block:: bash

   # Monitor velocity commands
   ros2 topic echo /prometheus/cmd_vel

   # Monitor the planned path
   ros2 topic echo /prometheus/planned_path

   # Monitor laser scans
   ros2 topic echo /prometheus/scan

   # Monitor odometry
   ros2 topic echo /prometheus/odom 

Running with Parameters
---------------------

You can customize the navigation node behavior using a parameter file:

.. code-block:: bash

   ros2 run prometheus_core prometheus_navigation_node --ros-args --params-file /path/to/prometheus_core/config/navigation_params.yaml

This allows you to change the path follower algorithm and its parameters without recompiling. 