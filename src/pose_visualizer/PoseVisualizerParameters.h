#pragma once

#include <ros/node_handle.h>
#include <string>

namespace stargazer_ros_tool {

struct PoseVisualizerParameters {

  static PoseVisualizerParameters& getInstance();

  void fromNodeHandle(const ros::NodeHandle&);

  std::string bag_file;
  std::string cam_config;
  std::string map_config;
  std::string map_frame;
  std::string camera_frame;
  std::string landmark_topic;
  std::string pose_pub_topic;
  double rate;

 private:
  PoseVisualizerParameters();
};

}  // namespace stargazer_ros_tool
