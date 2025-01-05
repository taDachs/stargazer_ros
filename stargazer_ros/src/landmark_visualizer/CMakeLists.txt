generate_parameter_library(
    landmark_visualizer_parameters
    landmark_visualizer_parameters.yaml
)

add_executable(landmark_visualizer
    landmark_visualizer.cpp
)

# Add dependencies for the executable
ament_target_dependencies(landmark_visualizer
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
target_link_libraries(landmark_visualizer
    ${OpenCV_LIBRARIES}
    yaml-cpp
    landmark_visualizer_parameters
)

# Install the executable
install(TARGETS landmark_visualizer
    DESTINATION lib/${PROJECT_NAME}
)
