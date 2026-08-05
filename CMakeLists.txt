cmake_minimum_required(VERSION 3.5)
project(use_fast_calib2_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_executable(svd_demo_node src/svd_demo_node.cpp)
target_include_directories(svd_demo_node PRIVATE include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(svd_demo_node rclcpp)

install(TARGETS
  svd_demo_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
