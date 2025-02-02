#pragma once

#include <ros/node_handle.h>
#include <string>

namespace stargazer_ros_tool {

struct ReprojectionVisualizerParameters {

  static ReprojectionVisualizerParameters& getInstance();

  void fromNodeHandle(const ros::NodeHandle&);

  std::string cam_config;
  std::string map_config;
  std::string bag_file;
  std::string landmark_topic;
  std::string img_topic;
  std::string pose_topic;
  int waitTime;

 private:
  ReprojectionVisualizerParameters();
};

}  // namespace stargazer_ros_tool
