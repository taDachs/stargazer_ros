generate_parameter_library(
    pose_visualizer_parameters
    pose_visualizer_parameters.yaml
)

add_executable(pose_visualizer
    pose_visualizer.cpp
)

# Add dependencies for the executable
ament_target_dependencies(pose_visualizer
    rclcpp
    visualization_msgs
    std_msgs
    sensor_msgs
    cv_bridge
    tf2
    tf2_geometry_msgs
    Eigen3
    Boost
    Ceres
    yaml-cpp
    stargazer_interfaces
    rosbag2_cpp
    stargazer
    image_transport
)

# Link necessary libraries
target_link_libraries(pose_visualizer
    ${OpenCV_LIBRARIES}
    yaml-cpp
    pose_visualizer_parameters
)

# Install the executable
install(TARGETS pose_visualizer
    DESTINATION lib/${PROJECT_NAME}
)
