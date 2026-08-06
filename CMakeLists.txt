cmake_minimum_required(VERSION 3.14)
project(fast_calib2 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation features registration)
find_package(yaml-cpp REQUIRED)
find_package(GTest REQUIRED)

add_definitions(${PCL_DEFINITIONS})

set(PROJECT_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

function(configure_fast_calib_target target)
  target_include_directories(${target} PRIVATE ${PROJECT_INCLUDE_DIRS})
  target_link_libraries(${target} PRIVATE
    Eigen3::Eigen
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    yaml-cpp
  )
endfunction()

add_executable(fast_calib2 src/main.cpp)
configure_fast_calib_target(fast_calib2)

add_executable(multi_fast_calib src/multi_scene.cpp)
configure_fast_calib_target(multi_fast_calib)

add_executable(lidar_center_test test/lidar_center_test.cpp)
configure_fast_calib_target(lidar_center_test)

add_executable(common_lib_test test/common_lib_test.cpp)
target_include_directories(common_lib_test PRIVATE ${PROJECT_INCLUDE_DIRS})
target_link_libraries(common_lib_test PRIVATE
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
  GTest::gtest
  GTest::gtest_main
)
