#pragma once

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <stargazer/DebugVisualizer.h>
#include <stargazer/LandmarkFinder.h>

#include <landmark_finder_parameters.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <stargazer_interfaces/msg/landmark_array.hpp>

namespace stargazer_ros_tool {

class LandmarkFinderInterface {

 public:
  LandmarkFinderInterface(rclcpp::Node::SharedPtr node);

 private:
  void imgCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  image_transport::Subscriber img_sub;
  image_transport::ImageTransport img_trans;
  rclcpp::Publisher<stargazer_interfaces::msg::LandmarkArray>::SharedPtr lm_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_points,
      debug_pub_clusters, debug_pub_hypotheses, debug_pub_landmarks;
  std::shared_ptr<landmark_finder::ParamListener> param_listener;
  stargazer::DebugVisualizer debugVisualizer_;
  std::unique_ptr<stargazer::LandmarkFinder> landmarkFinder;
  void updateFinderConfig();
};
}  // namespace stargazer_ros_tool
