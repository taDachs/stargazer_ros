//
// Created by bandera on 09.06.16.
//

#include "LandmarkLocalizerInterface.h"

#include <cv_bridge/cv_bridge.h>

#include "../StargazerConversionMethods.h"
#include "stargazer/CeresLocalizer.h"

using namespace stargazer_ros_tool;

LandmarkLocalizerInterface::LandmarkLocalizerInterface(rclcpp::Node::SharedPtr node)
    : logger{node->get_logger()} {


  param_listener = std::make_shared<landmark_localizer::ParamListener>(node);
  auto params = param_listener->get_params();

  // Set parameters
  last_timestamp_ = node->get_clock()->now();
  debugVisualizer_.SetWaitTime(1);

  // Setup and set values in dynamic reconfigure server

  localizer_ = std::make_unique<stargazer::CeresLocalizer>(
      params.cam_config, params.map_config, params.estimate_2d_pose);

  // Initialize publisher
  pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(params.pose_topic, 1);
  lm_sub = node->create_subscription<stargazer_interfaces::msg::LandmarkArray>(
      params.landmark_topic,
      1,
      std::bind(&LandmarkLocalizerInterface::landmarkCallback, this, std::placeholders::_1));

  if (!params.debug_mode)
    return;

  debug_pub_reprojection =
      node->create_publisher<sensor_msgs::msg::Image>("debug_reprojection", 1);
}

void LandmarkLocalizerInterface::landmarkCallback(
    const stargazer_interfaces::msg::LandmarkArray::ConstSharedPtr& msg) {
  auto params = param_listener->get_params();

  rclcpp::Time this_timestamp = msg->header.stamp;
  double dt = (this_timestamp - last_timestamp_).seconds();
  rclcpp::Time last_timestamp = this_timestamp;

  std::vector<stargazer::ImgLandmark> detected_landmarks = convert2ImgLandmarks(*msg);

  // Localize
  localizer_->UpdatePose(detected_landmarks, dt);
  stargazer::Pose pose = localizer_->getPose();

  // Publish tf pose
  geometry_msgs::msg::TransformStamped map2camTransform;
  pose2tf(pose, map2camTransform);
  map2camTransform.header.stamp = msg->header.stamp;
  map2camTransform.header.frame_id = params.map_frame;
  map2camTransform.child_frame_id = params.camera_frame;
  tf_pub->sendTransform(map2camTransform);

  geometry_msgs::msg::PoseStamped poseStamped;
  poseStamped.header.frame_id = params.map_frame;
  poseStamped.header.stamp = msg->header.stamp;
  poseStamped.pose = pose2gmPose(pose);
  pose_pub->publish(poseStamped);

  const ceres::Solver::Summary& summary =
      dynamic_cast<stargazer::CeresLocalizer*>(localizer_.get())->getSummary();
  RCLCPP_DEBUG_STREAM(logger,
                      "Number of iterations: " << summary.iterations.size() << " Time needed: "
                                               << summary.total_time_in_seconds);
  if (summary.termination_type != ceres::TerminationType::CONVERGENCE) {
    RCLCPP_WARN_STREAM(logger,
                       "Solver did not converge! "
                           << ceres::TerminationTypeToString(summary.termination_type));
    RCLCPP_WARN_STREAM(logger, summary.FullReport());
  }

  //  Visualize
  if (params.debug_mode) {
    cv_bridge::CvImage out_msg;
    out_msg.header = msg->header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;

    out_msg.image = cv::Mat::zeros(1024, 1360, CV_8UC3);  // TODO is an hardcoded image resolution valid?
    out_msg.image.setTo(cv::Scalar(255, 255, 255));
    out_msg.image = debugVisualizer_.DrawLandmarks(out_msg.image, detected_landmarks);
    out_msg.image = debugVisualizer_.DrawLandmarks(
        out_msg.image, localizer_->getLandmarks(), localizer_->getIntrinsics(), pose);
    debug_pub_reprojection->publish(*out_msg.toImageMsg());
  }
}
