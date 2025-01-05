#include "LandmarkCalibratorInterface.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("landmark_calibrator_node");

  stargazer_ros_tool::LandmarkCalibratorInterface interface(node);

  // ros::spin(); // We don't need that here
  return EXIT_SUCCESS;
}
