generate_parameter_library(
    landmark_calibrator_parameters
    landmark_calibrator_parameters.yaml
)

add_executable(landmark_calibrator
    LandmarkCalibratorInterface.cpp
    LandmarkCalibratorInterface.h
    landmark_calibrator_node.cpp
)

# Add dependencies for the executable
ament_target_dependencies(landmark_calibrator
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
)

# Link necessary libraries
target_link_libraries(landmark_calibrator
    ${OpenCV_LIBRARIES}
    yaml-cpp
    landmark_calibrator_parameters
)

# Install the executable
install(TARGETS landmark_calibrator
    DESTINATION lib/${PROJECT_NAME}
)
