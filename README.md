# Prometheus Core

A modular robotics framework built in Rust for ROS 2, designed for autonomous navigation and control.

## Overview

Prometheus Core provides a flexible, component-based architecture for building robotic systems. It includes:

- Navigation stack with path planning and following
- Control stack for robot motion control
- Perception stack for sensor processing
- Lifecycle management for components

## Features

- Written in Rust for safety, performance, and reliability
- Integrates with ROS 2 for communication with other robotics software
- Modular design allows easy extension and customization
- Thread-safe components for concurrent operation
- Real-time costmap updates from laser scan data
- Obstacle inflation for safe navigation
- Configurable parameters via ROS 2 parameter system

## Path Planning and Navigation

The navigation stack features advanced path planning capabilities:

### Path Planning
- **Cubic Spline Path Planning**: Generates smooth, continuous paths
- Automatic intermediate point generation for long distances
- Continuous curvature profiles for optimal robot motion
- Fallback to simple linear paths for short distances

### Path Following
- Pure Pursuit algorithm for accurate trajectory tracking
- Configurable lookahead distance and velocity parameters
- Automatic velocity command generation
- Fallback controller for edge cases

### Visualization
- Real-time path visualization via ROS 2 topics
- Path publishing on `/prometheus/planned_path`
- Compatible with RViz Path display
- Includes orientation data for smooth transitions

## Installation

### Prerequisites

- ROS 2 (Jazzy or compatible)
- Rust 1.88+ (stable toolchain)
- Cargo
- [ros2_rust](https://github.com/ros2-rust/ros2_rust)

### Building from Source

1. Clone the repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository-url> prometheus_core
```

2. Install Rust dependencies and upgrade if needed:
```bash
rustup update stable
```

3. Build the package with colcon:
```bash
cd ~/ros2_ws
colcon build --packages-select prometheus_core
```

4. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Running the Navigation Node

**Option 1: Using the Launch File (Recommended)**

```bash
# Launch with default configuration
ros2 launch prometheus_core navigation.launch.py

# Launch with custom parameters
ros2 launch prometheus_core navigation.launch.py params_file:=/path/to/custom_params.yaml
```

**Option 2: Direct Execution**

```bash
# Run the navigation node directly
ros2 run prometheus_core prometheus_navigation_node

# With custom parameters
ros2 run prometheus_core prometheus_navigation_node --ros-args --params-file config/navigation_params.yaml
```

### Sending a Goal

```bash
# Send a goal pose using ROS 2 command line
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{ 
  header: { frame_id: 'map' }, 
  pose: { 
    position: { x: 1.0, y: 1.0, z: 0.0 }, 
    orientation: { w: 1.0 }
  }
}"

# Or use RViz's 2D Nav Goal tool
```

### Visualizing the Path

1. Launch RViz
2. Add a Path display
3. Set the topic to `/prometheus/planned_path`
4. Set the fixed frame to `map`

### Topics

**Subscribed Topics:**
- `/prometheus/odom` (nav_msgs/Odometry) - Robot odometry for localization
- `/prometheus/scan` (sensor_msgs/LaserScan) - Laser scan data for obstacle detection
- `/goal_pose` (geometry_msgs/PoseStamped) - Goal poses for navigation

**Published Topics:**
- `/prometheus/cmd_vel` (geometry_msgs/Twist) - Velocity commands for the robot
- `/prometheus/planned_path` (nav_msgs/Path) - Planned path for visualization

### Configuration

The navigation stack can be configured with parameters in `config/navigation_params.yaml`:
- `lookahead_distance` - Lookahead distance for Pure Pursuit controller
- `max_linear_velocity` - Maximum linear velocity for the robot
- `path_resolution` - Resolution for spline path generation
- Costmap parameters (inflation radius, cost scaling, etc.)

### Costmap and Obstacle Avoidance

The navigation stack includes:
- **Global Costmap**: For long-term obstacle representation
- **Local Costmap**: Updated from laser scans for dynamic obstacle avoidance
- **Obstacle Inflation**: Configurable safety margins around obstacles
- **Real-time Updates**: Laser scan data continuously updates the local costmap

### Monitoring

```bash
# Monitor velocity commands
ros2 topic echo /prometheus/cmd_vel

# Monitor planned path
ros2 topic echo /prometheus/planned_path

# Monitor laser scans
ros2 topic echo /prometheus/scan

# Monitor odometry
ros2 topic echo /prometheus/odom
```

## Documentation

For detailed documentation, build the Sphinx docs in the docs directory

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Acknowledgments

- The cubic spline path planning and Pure Pursuit algorithms are inspired by the RUST implementation from [RustRobotics](https://github.com/rsasaki0109/rust_robotics)
- Thanks to the ROS 2 and Rust communities for their excellent tools and documentation

## License

This project is licensed under the MIT License - see the LICENSE file for details.
