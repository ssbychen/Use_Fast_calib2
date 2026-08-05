# use_fast_calib2 ROS2 Humble skeleton

Minimal ROS2 package demonstrating a header-only SVD solver (uses Eigen).

Prerequisites
- ROS2 Humble installed and sourced
- colcon build tool
- Eigen3 (Ubuntu: sudo apt install libeigen3-dev)

Build
1. Source your ROS2 Humble environment:
   source /opt/ros/humble/setup.bash

2. Put this package in a workspace (e.g., ~/ros2_ws/src/use_fast_calib2) and run:
   cd ~/ros2_ws
   colcon build --packages-select use_fast_calib2_ros2

Run
1. Source the workspace:
   source install/setup.bash

2. Run the demo node:
   ros2 run use_fast_calib2_ros2 svd_demo_node

Notes
- If colcon complains about Eigen, install libeigen3-dev and try again.
