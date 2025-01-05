generate_parameter_library(
    reprojection_visualizer_parameters
    reprojection_visualizer_parameters.yaml
)

add_executable(reprojection_visualizer
    reprojection_visualizer_node.cpp
    ReprojectionVisualizer.cpp
    ReprojectionVisualizer.h
)

# Add dependencies for the executable
ament_target_dependencies(reprojection_visualizer
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
target_link_libraries(reprojection_visualizer
    ${OpenCV_LIBRARIES}
    yaml-cpp
    reprojection_visualizer_parameters
)

# Install the executable
install(TARGETS reprojection_visualizer
    DESTINATION lib/${PROJECT_NAME}
)
