//
// Created by bandera on 09.06.16.
//

#pragma once

// ROS includes
#include <tf2_ros/transform_broadcaster.h>

// Msg formats
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/subscription.hpp>
#include <stargazer_interfaces/msg/landmark_array.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <landmark_localizer_parameters.hpp>
#include "stargazer/DebugVisualizer.h"
#include "stargazer/Localizer.h"
#include "stargazer/StargazerImgTypes.h"
#include "stargazer/StargazerTypes.h"

namespace stargazer_ros_tool {

class LandmarkLocalizerInterface {

 public:
  LandmarkLocalizerInterface(rclcpp::Node::SharedPtr);

 private:
  // Subscriber
  rclcpp::Subscription<stargazer_interfaces::msg::LandmarkArray>::SharedPtr lm_sub;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub;

  rclcpp::Logger logger;
  std::shared_ptr<landmark_localizer::ParamListener> param_listener;
  stargazer::DebugVisualizer debugVisualizer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_reprojection;

  std::unique_ptr<stargazer::Localizer> localizer_;

  rclcpp::Time last_timestamp_;

  void landmarkCallback(const stargazer_interfaces::msg::LandmarkArray::ConstSharedPtr& msg);
};

}  // namespace stargazer_ros_tool
