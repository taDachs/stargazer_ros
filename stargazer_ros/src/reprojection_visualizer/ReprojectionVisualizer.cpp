//
// Created by bandera on 10.06.16.
//

#include "ReprojectionVisualizer.h"

// ROS includes
#include <memory>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>

// ROS messages
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// Local Helpers
#include <tf2/transform_datatypes.h>

#include "../StargazerConversionMethods.h"
#include "stargazer/CoordinateTransformations.h"
#include "stargazer/StargazerConfig.h"

using namespace stargazer_ros_tool;
using namespace stargazer;

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

ReprojectionVisualizer::ReprojectionVisualizer(rclcpp::Node::SharedPtr node)
    {
  param_listener = std::make_shared<reprojection_visualizer::ParamListener>(node);
  auto params = param_listener->get_params();

  // Set parameters
  readCamConfig(params.cam_config, camera_intrinsics);
  readMapConfig(params.map_config, landmarks);

  // Convert landmark points to worldcoordinates once.
  for (auto& el : landmarks) {
    for (auto& pt : el.second.points) {
      double x, y, z;
      transformLandMarkToWorld(pt[(int)POINT::X],
                               pt[(int)POINT::Y],
                               el.second.pose.position.data(),
                               el.second.pose.orientation.data(),
                               &x,
                               &y,
                               &z);
      pt[(int)POINT::X] = x;
      pt[(int)POINT::Y] = y;
      pt[(int)POINT::Z] = z;
    }
  }

  debugVisualizer_ = std::make_unique<DebugVisualizer>();
  debugVisualizer_->SetWaitTime(params.wait_time);

  /*
   * Read Bag
   */
  rosbag2_cpp::Reader bag;
  bag.open(params.bag_file);

  // Set up fake subscribers to capture images
  BagSubscriber<stargazer_interfaces::msg::LandmarkArray> lm_sub;
  BagSubscriber<geometry_msgs::msg::PoseStamped> pose_sub;
  BagSubscriber<sensor_msgs::msg::Image> img_sub;

  // Use time synchronizer to make sure we get properly synchronized images
  message_filters::TimeSynchronizer<stargazer_interfaces::msg::LandmarkArray, geometry_msgs::msg::PoseStamped, sensor_msgs::msg::Image> sync(
      lm_sub, pose_sub, img_sub, 25);
  using namespace std::placeholders;
  sync.registerCallback(std::bind(
      &ReprojectionVisualizer::synchronizerCallback, this, _1, _2, _3));

  rclcpp::Serialization<stargazer_interfaces::msg::LandmarkArray> lm_serializer;
  rclcpp::Serialization<geometry_msgs::msg::PoseStamped> pose_serializer;
  rclcpp::Serialization<sensor_msgs::msg::Image> img_serializer;

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
    } else if (msg->topic_name == params.img_topic) {
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
      img_serializer.deserialize_message(&serialized_msg, img_msg.get());
      img_sub.newMessage(img_msg);
    }

    rclcpp::spin_some(node);
    if (!rclcpp::ok())
      break;
  }

  bag.close();
}

void ReprojectionVisualizer::synchronizerCallback(
    const stargazer_interfaces::msg::LandmarkArray::ConstSharedPtr& lm_msg,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) {

  cv_bridge::CvImagePtr cvPtr =
      cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);

  std::vector<stargazer::ImgLandmark> img_lms = convert2ImgLandmarks(*lm_msg);
  debugVisualizer_->DrawLandmarks(
      cvPtr->image, landmarks, camera_intrinsics, gmPose2pose(pose_msg->pose));
  debugVisualizer_->DrawLandmarks(cvPtr->image, img_lms);
  debugVisualizer_->ShowImage(cvPtr->image);
}
