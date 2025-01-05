generate_parameter_library(
    landmark_finder_parameters
    landmark_finder_parameters.yaml
)

add_executable(landmark_finder
    LandmarkFinderInterface.cpp
    LandmarkFinderInterface.h
    landmark_finder_node.cpp
)

# Add dependencies for the executable
ament_target_dependencies(landmark_finder
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
target_link_libraries(landmark_finder
    ${OpenCV_LIBRARIES}
    yaml-cpp
    landmark_finder_parameters
)

# Install the executable
install(TARGETS landmark_finder
    DESTINATION lib/${PROJECT_NAME}
)
