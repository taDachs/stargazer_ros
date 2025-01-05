//
// Created by bandera on 10.06.16.
//

#pragma once

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <memory>
#include <reprojection_visualizer_parameters.hpp>
#include "stargazer/DebugVisualizer.h"
#include <stargazer_interfaces/msg/landmark_array.hpp>

namespace stargazer_ros_tool {

class ReprojectionVisualizer {
 public:
  ReprojectionVisualizer(rclcpp::Node::SharedPtr node);

 private:
  stargazer::landmark_map_t landmarks;
  stargazer::camera_params_t camera_intrinsics = {{0., 0., 0., 0.}};

  std::shared_ptr<reprojection_visualizer::ParamListener> param_listener;

  std::unique_ptr<stargazer::DebugVisualizer> debugVisualizer_;

  void synchronizerCallback(const stargazer_interfaces::msg::LandmarkArray::ConstSharedPtr& lm_msg,
                            const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg,
                            const sensor_msgs::msg::Image::ConstSharedPtr& img_msg);
};

}  // namespace stargazer_ros_tool
