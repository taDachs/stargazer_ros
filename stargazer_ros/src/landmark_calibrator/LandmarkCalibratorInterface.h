//
// Created by bandera on 10.06.16.
//

#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <stargazer_interfaces/msg/landmark.hpp>
#include "stargazer/LandmarkCalibrator.h"
#include "stargazer/StargazerImgTypes.h"
#include <stargazer_interfaces/msg/landmark_array.hpp>
#include <landmark_calibrator_parameters.hpp>
#include <rclcpp/node.hpp>

namespace stargazer_ros_tool {

class LandmarkCalibratorInterface {
 public:
  LandmarkCalibratorInterface(rclcpp::Node::SharedPtr);
  ~LandmarkCalibratorInterface();

 private:
  std::unique_ptr<stargazer::LandmarkCalibrator> bundleAdjuster;
  std::vector<stargazer::Pose> observed_poses;
  std::vector<std::vector<stargazer::ImgLandmark>> observed_landmarks;
  std::vector<rclcpp::Time> observed_timestamps;
  std::string pose_frame;
  rosbag2_cpp::Writer bag_out;
  rclcpp::Logger logger;

  std::shared_ptr<landmark_calibrator::ParamListener> param_listener;

  void load_data();
  void write_data();
  void optimize();
  void synchronizerCallback(const stargazer_interfaces::msg::LandmarkArray::ConstSharedPtr& lm_msg,
                            const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg);
};

}  // namespace stargazer_ros_tool
