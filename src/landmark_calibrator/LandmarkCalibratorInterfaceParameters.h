#pragma once

#include <ros/node_handle.h>
#include <string>

namespace stargazer_ros_tool {

struct LandmarkCalibratorInterfaceParameters {

  static LandmarkCalibratorInterfaceParameters& getInstance();

  void fromNodeHandle(const ros::NodeHandle&);

  std::string cam_cfg_file_in;
  std::string cam_cfg_file_out;
  std::string map_cfg_file_in;
  std::string map_cfg_file_out;
  std::string bag_file;
  std::string landmark_topic;
  std::string pose_topic;

 private:
  LandmarkCalibratorInterfaceParameters();
};

}  // namespace stargazer_ros_tool
