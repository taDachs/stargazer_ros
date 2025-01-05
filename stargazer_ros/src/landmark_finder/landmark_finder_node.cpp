#include <rclcpp/executors.hpp>
#include "LandmarkFinderInterface.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("landmark_finder_node");
  stargazer_ros_tool::LandmarkFinderInterface interface(node);
  rclcpp::spin(node);
  return EXIT_SUCCESS;
}
