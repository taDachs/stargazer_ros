#pragma once

#include <ros/node_handle.h>
#include <string>

namespace stargazer_ros_tool {

struct LandmarkLocalizerInterfaceParameters {

  void fromNodeHandle(const ros::NodeHandle&);

  std::string cam_config;
  std::string map_config;
  std::string map_frame;
  std::string camera_frame;
  std::string landmark_topic;
  std::string pose_topic;
  bool debug_mode;
  bool estimate_2d_pose;
};
}  // namespace stargazer_ros_tool
