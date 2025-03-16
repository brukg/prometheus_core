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
- Rust (nightly toolchain recommended)
- Cargo
- ros2_rust

### Building from Source

1. Clone the repository into your ROS 2 workspace
2. Build the package with colcon
3. Source the workspace

## Usage

### Running the Navigation Node

```bash
# Run the navigation node
ros2 run prometheus_core prometheus_navigation_node
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

### Configuration

The navigation stack can be configured with:
- Lookahead distance for Pure Pursuit
- Maximum linear velocity
- Path resolution for spline generation

### Monitoring

```bash
# Monitor velocity commands
ros2 topic echo /prometheus/cmd_vel

# Monitor planned path
ros2 topic echo /prometheus/planned_path
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
