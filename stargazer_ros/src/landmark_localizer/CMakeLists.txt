generate_parameter_library(
    landmark_localizer_parameters
    landmark_localizer_parameters.yaml
)

add_executable(landmark_localizer
    LandmarkLocalizerInterface.cpp
    LandmarkLocalizerInterface.h
    landmark_localizer_node.cpp
)

# Add dependencies for the executable
ament_target_dependencies(landmark_localizer
    rclcpp
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
target_link_libraries(landmark_localizer
    ${OpenCV_LIBRARIES}
    yaml-cpp
    landmark_localizer_parameters
)

# Install the executable
install(TARGETS landmark_localizer
    DESTINATION lib/${PROJECT_NAME}
)
