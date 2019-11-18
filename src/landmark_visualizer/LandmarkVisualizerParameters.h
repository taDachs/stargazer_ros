#pragma once

#include <ros/node_handle.h>
#include <string>

namespace stargazer_ros_tool {

struct LandmarkVisualizerParameters {

  static LandmarkVisualizerParameters& getInstance();

  void fromNodeHandle(const ros::NodeHandle&);

  std::string map_config;

  std::string landmark_topic;
  std::string map_frame_id;
  double rate;

 private:
  LandmarkVisualizerParameters();
};

}  // namespace stargazer_ros_tool
