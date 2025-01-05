//
// Created by bandera on 20.03.16.
//
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <boost/foreach.hpp>
#include "../StargazerConversionMethods.h"
#include "stargazer/CeresLocalizer.h"
#include <stargazer_interfaces/msg/landmark_array.hpp>
#include <pose_visualizer_parameters.hpp>

using namespace stargazer;
using namespace stargazer_ros_tool;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pose_visualizer");

  pose_visualizer::ParamListener param_listener(node);
  auto params = param_listener.get_params();

  auto path_pub =
      node->create_publisher<geometry_msgs::msg::PoseArray>(params.pose_pub_topic, 1);
  std::unique_ptr<stargazer::CeresLocalizer> localizer =
      std::make_unique<stargazer::CeresLocalizer>(params.cam_config, params.map_config);

  geometry_msgs::msg::PoseArray pose_array;

  rosbag2_cpp::Reader bag;
  bag.open(params.bag_file);
  rclcpp::Serialization<stargazer_interfaces::msg::LandmarkArray> lm_serializer;

  std::vector<std::string> topics;
  topics.push_back(std::string(params.landmark_topic));

  while (bag.has_next()) {
    rosbag2_storage::SerializedBagMessageSharedPtr msg = bag.read_next();

    if (msg->topic_name == params.landmark_topic) {
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      auto img_landmark_msg = std::make_shared<stargazer_interfaces::msg::LandmarkArray>();
      lm_serializer.deserialize_message(&serialized_msg, img_landmark_msg.get());
        std::vector<stargazer::ImgLandmark> img_landmarks =
            convert2ImgLandmarks(*img_landmark_msg);
        localizer->UpdatePose(img_landmarks, 0.0);
        geometry_msgs::msg::PoseStamped camera_pose;
        camera_pose.pose = pose2gmPose(localizer->getPose());
        pose_array.poses.push_back(camera_pose.pose);
    }
  }
  bag.close();
  pose_array.header.frame_id = params.map_frame;

  rclcpp::Rate r(params.rate);
  while (rclcpp::ok()) {
    rclcpp::Time timestamp = node->get_clock()->now();

    pose_array.header.stamp = timestamp;

    path_pub->publish(pose_array);

    rclcpp::spin_some(node);
    r.sleep();
  }

  return 0;
}
