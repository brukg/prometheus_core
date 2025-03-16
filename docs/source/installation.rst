Installation
===========

Prerequisites
------------

* ROS 2 Jazzy (or compatible version)
* Rust (nightly toolchain recommended)
* Cargo
* [ros2_rust](https://github.com/ros2-rust/ros2_rust)

Installing from Source
---------------------

1. Clone the repository into your ROS 2 workspace:

   .. code-block:: bash

      cd ~/ros2_ws/src
      git clone https://github.com/brukg/prometheus_core.git

2. Build the package using colcon:

   .. code-block:: bash

      cd ~/ros2_ws
      colcon build --packages-select prometheus_core

3. Source the workspace:

   .. code-block:: bash

      source ~/ros2_ws/install/setup.bash 