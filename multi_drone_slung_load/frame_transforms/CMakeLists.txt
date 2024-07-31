cmake_minimum_required(VERSION 3.4)
project(frame_transforms LANGUAGES CXX)

find_package(pybind11 REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)

# Note have to include directories because not linked automatically by ament_cmake like normal c-ROS2 packages
include_directories(/usr/include/eigen3)
include_directories(/opt/ros/humble/include/geometry_msgs/)
include_directories(/opt/ros/humble/include/rosidl_runtime_cpp/)
include_directories(/opt/ros/humble/include/rosidl_runtime_c/)
include_directories(/opt/ros/humble/include/sensor_msgs/)
include_directories(/opt/ros/humble/include/std_msgs/)
include_directories(/opt/ros/humble/include/builtin_interfaces/)

pybind11_add_module(frame_transforms frame_transforms.cpp frame_transforms_bind.cpp frame_transforms.h)