cmake_minimum_required(VERSION 3.10.2)
project(stargazer_ros_tool)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Ensure C++17 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(NOT COMPILER_SUPPORTS_CXX17)
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

###################
## Find packages ##
###################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(stargazer_interfaces REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(stargazer REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

###########
## Build ##
###########
# Add include and library directories
include_directories(
    ${Eigen3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

# Add subdirectories
add_subdirectory(src/landmark_calibrator)
add_subdirectory(src/landmark_finder)
add_subdirectory(src/landmark_localizer)
add_subdirectory(src/landmark_visualizer)
add_subdirectory(src/pose_visualizer)
add_subdirectory(src/reprojection_visualizer)

#############
## Install ##
#############
# Install launch files, scripts, and other resources
install(DIRECTORY launch res param
  DESTINATION share/${PROJECT_NAME}/
)

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Package ##
#############
ament_package()

