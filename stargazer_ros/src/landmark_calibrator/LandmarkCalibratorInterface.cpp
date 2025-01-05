//
// Created by bandera on 10.06.16.
//

#include "LandmarkCalibratorInterface.h"

// ROS messages
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/serialization.hpp>

// Local Helpers
#include <tf2/transform_datatypes.h>
#include <boost/foreach.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include "../StargazerConversionMethods.h"
#include "stargazer/StargazerConfig.h"
#define foreach BOOST_FOREACH

using namespace stargazer_ros_tool;

/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M> {
 public:
  void newMessage(const std::shared_ptr<M const>& msg) {
    this->signalMessage(msg);
  }
};

LandmarkCalibratorInterface::LandmarkCalibratorInterface(rclcpp::Node::SharedPtr node_handle)
    : logger{node_handle->get_logger()}  {

  param_listener = std::make_shared<landmark_calibrator::ParamListener>(node_handle);
  auto params = param_listener->get_params();

  // Set parameters
  bag_out.open(params.bag_file + "_optimized.bag");
  bundleAdjuster = std::make_unique<stargazer::LandmarkCalibrator>(
      params.cam_cfg_file_in, params.map_cfg_file_in);
  load_data();

  // Init logging for ceres
  google::InitGoogleLogging("LandmarkCalibrator");
  FLAGS_logtostderr = 1;

  // Optimize
  optimize();

  // Write data
  write_data();
}

LandmarkCalibratorInterface::~LandmarkCalibratorInterface() { bag_out.close(); }

void LandmarkCalibratorInterface::synchronizerCallback(
    const stargazer_interfaces::msg::LandmarkArray::ConstSharedPtr& lm_msg,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg) {
  auto params = param_listener->get_params();

  std::vector<stargazer::ImgLandmark> img_lms = convert2ImgLandmarks(*lm_msg);

  // Test for wrong number of detected LEDs
  size_t old_size = img_lms.size();
  img_lms.erase(
      std::remove_if(img_lms.begin(),
                     img_lms.end(),
                     [&](stargazer::ImgLandmark& lm) {
                       return (lm.corners.size() + lm.idPoints.size() !=
                               bundleAdjuster->getLandmarks().at(lm.nID).points.size());
                     }),
      img_lms.end());
  size_t new_size = img_lms.size();
  if (old_size != new_size)
    RCLCPP_INFO_STREAM(logger, "Removed " << old_size - new_size << " landmarks because of wrong number of points.");
  if (!img_lms.empty()) {
    observed_timestamps.push_back(lm_msg->header.stamp);
    observed_landmarks.push_back(img_lms);
    observed_poses.push_back(gmPose2pose(pose_msg->pose));
    pose_frame = pose_msg->header.frame_id;

    bag_out.write(*pose_msg, params.pose_topic, pose_msg->header.stamp);
    bag_out.write(*lm_msg, params.landmark_topic, lm_msg->header.stamp);

  } else {
    RCLCPP_WARN_STREAM(logger, "Received empty landmarks message.");
  }
}

void LandmarkCalibratorInterface::load_data() {
  auto params = param_listener->get_params();
  RCLCPP_INFO_STREAM(logger, "Reading bag file...");
  rosbag2_cpp::Reader bag;
  bag.open(params.bag_file);

  std::vector<std::string> topics;
  topics.push_back(std::string(params.landmark_topic));
  topics.push_back(std::string(params.pose_topic));

  // Set up fake subscribers to capture images
  BagSubscriber<stargazer_interfaces::msg::LandmarkArray> lm_sub;
  BagSubscriber<geometry_msgs::msg::PoseStamped> pose_sub;

  // Use time synchronizer to make sure we get properly synchronized images
  message_filters::TimeSynchronizer<stargazer_interfaces::msg::LandmarkArray, geometry_msgs::msg::PoseStamped> sync(
      lm_sub, pose_sub, 25);
  using namespace std::placeholders;
  sync.registerCallback(std::bind(
      &LandmarkCalibratorInterface::synchronizerCallback, this, _1, _2));

  rclcpp::Serialization<stargazer_interfaces::msg::LandmarkArray> lm_serializer;
  rclcpp::Serialization<geometry_msgs::msg::PoseStamped> pose_serializer;

  while (bag.has_next()) {
    rosbag2_storage::SerializedBagMessageSharedPtr msg = bag.read_next();

    if (msg->topic_name == params.landmark_topic) {
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      auto lm_msg = std::make_shared<stargazer_interfaces::msg::LandmarkArray>();
      lm_serializer.deserialize_message(&serialized_msg, lm_msg.get());
      lm_sub.newMessage(lm_msg);
    } else if (msg->topic_name == params.pose_topic) {
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
      pose_serializer.deserialize_message(&serialized_msg, pose_msg.get());
      pose_sub.newMessage(pose_msg);
    }
  }

  bag.close();

  std::cout << "CameraParameters: " << bundleAdjuster->getIntrinsics().size() << std::endl;
  std::cout << "Landmarks: " << bundleAdjuster->getLandmarks().size() << std::endl;
  std::cout << "Observations(Images): " << observed_landmarks.size() << std::endl;
  std::cout << "Observations(Poses): " << observed_poses.size() << std::endl;
}

void LandmarkCalibratorInterface::write_data() {
  auto params = param_listener->get_params();
  stargazer::writeCamConfig(params.cam_cfg_file_out, bundleAdjuster->getIntrinsics());
  stargazer::writeMapConfig(params.map_cfg_file_out, bundleAdjuster->getLandmarks());

  RCLCPP_INFO_STREAM(logger, "Writing bag file..." << params.bag_file << "_optimized.bag");
  for (size_t i = 0; i < observed_timestamps.size(); i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = observed_timestamps[i];
    pose.header.frame_id = pose_frame;
    pose.pose = pose2gmPose(bundleAdjuster->getPoses()[i]);
    bag_out.write(pose, params.pose_topic + "_optimized", observed_timestamps[i]);
  }
}

void LandmarkCalibratorInterface::optimize() {
  auto params = param_listener->get_params();
  // Start work by setting up problem
  bundleAdjuster->AddReprojectionResidualBlocks(observed_poses, observed_landmarks);
  // xy origin is landmark in southwest corner mrt maschinenhalle 0x0190
  // x axis points approximatly east, to landmark near of entry of maschinenhalle 0x2084
  bundleAdjuster->SetLandmarksOriginAndXAxis(0x0190, 0x2084);
  if (params.constant_intrinsics) {
    bundleAdjuster->SetIntrinsicsConstant();
  }
  bundleAdjuster->Optimize();
}
