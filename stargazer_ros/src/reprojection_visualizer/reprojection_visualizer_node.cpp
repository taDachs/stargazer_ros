#include "ReprojectionVisualizer.h"

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("reprojection_visualizer_node");

  stargazer_ros_tool::ReprojectionVisualizer reprojectionVisualizer(node);

  return EXIT_SUCCESS;
}
