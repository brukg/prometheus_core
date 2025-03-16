Usage
=====

Running the Prometheus Node
--------------------------

After building the package, you can run the Prometheus node with:

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

Monitoring Velocity Commands
--------------------------

The node publishes velocity commands on the ``cmd_vel`` topic. You can monitor these with:

.. code-block:: bash

   ros2 topic echo /cmd_vel 

Running with Parameters
---------------------

You can customize the navigation node behavior using a parameter file:

.. code-block:: bash

   ros2 run prometheus_core prometheus_navigation_node --ros-args --params-file /path/to/prometheus_core/config/navigation_params.yaml

This allows you to change the path follower algorithm and its parameters without recompiling. 