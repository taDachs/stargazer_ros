#include "LandmarkLocalizerInterface.h"

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("landmark_localizer_node");

  stargazer_ros_tool::LandmarkLocalizerInterface interface(node);

  rclcpp::spin(node);
  return EXIT_SUCCESS;
}
