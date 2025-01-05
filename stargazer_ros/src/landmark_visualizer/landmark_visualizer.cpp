//
// Created by bandera on 20.03.16.
//
#include <tf2_ros/transform_broadcaster.h>

#include "../StargazerConversionMethods.h"
#include "ceres/rotation.h"
#include "stargazer/StargazerConfig.h"
#include "stargazer/DebugVisualizer.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include <std_msgs/msg/color_rgba.hpp>
#include <landmark_visualizer_parameters.hpp>

using namespace stargazer;
using namespace stargazer_ros_tool;

static std_msgs::msg::ColorRGBA Color(const float r, const float g, const float b, const float a = 1.) {
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

static std_msgs::msg::ColorRGBA NONE = Color(0., 0., 0., 0.);
static std_msgs::msg::ColorRGBA WHITE = Color(1., 1., 1.);
static std_msgs::msg::ColorRGBA BLACK = Color(0., 0., 0.);
static std_msgs::msg::ColorRGBA YELLOW = Color(1., 1., 0.);

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("static_landmark_visualizer");

  /* Read in data */
  landmark_map_t landmarks;
  landmark_visualizer::ParamListener param_listener(node);
  auto params = param_listener.get_params();

  readMapConfig(params.map_config, landmarks);

  auto lm_pub =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(params.landmark_topic, 1);
  tf2_ros::TransformBroadcaster transformBroadcaster(node);

  // Prepare data
  visualization_msgs::msg::MarkerArray lm_msg;
  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  // middle of landmarks (origin is at a corner)
  const double middle = stargazer::Landmark::kGridDistance *
                        (stargazer::Landmark::kGridCount - 1) / 2.;
  // approximate landmark size (circuit board)
  const double board_size =
      stargazer::Landmark::kGridDistance * (stargazer::Landmark::kGridCount);
  // sphere size (diameter, based on grid distance so they don't touch each other)
  const double led_diameter = stargazer::Landmark::kGridDistance / 3.;

  for (auto& el : landmarks) {
    stargazer::Landmark& lm = el.second;
    std::string frame_id = "lm" + std::to_string(lm.id);

    // TF
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = params.map_frame_id;
    transform.child_frame_id = frame_id;
    pose2tf(lm.pose, transform);
    transforms.push_back(transform);

    // Marker message with common parameters
    visualization_msgs::msg::Marker common_marker;
    common_marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    common_marker.header.frame_id = frame_id;
    common_marker.id = lm.id;
    common_marker.action = visualization_msgs::msg::Marker::ADD;
    common_marker.pose.orientation.w = 1;

    // Landmark
    visualization_msgs::msg::Marker board_marker(common_marker);
    board_marker.ns = "Boards";
    board_marker.pose.position.x = middle;
    board_marker.pose.position.y = middle;
    board_marker.pose.position.z = -(board_size * 0.01 + led_diameter) * 0.5;
    board_marker.type = visualization_msgs::msg::Marker::CUBE;
    board_marker.scale.x = board_size;
    board_marker.scale.y = board_size;
    board_marker.scale.z = board_size * 0.01;
    board_marker.color = BLACK;
    lm_msg.markers.push_back(board_marker);

    // LEDS
    visualization_msgs::msg::Marker led_marker(common_marker);
    led_marker.ns = "LEDs";
    led_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    led_marker.scale.x = led_diameter;
    led_marker.scale.y = led_diameter;
    led_marker.scale.z = led_diameter;
    led_marker.color = YELLOW;
    for (auto& led : lm.points) {
      geometry_msgs::msg::Point pt;
      pt.x = std::get<(int)POINT::X>(led);
      pt.y = std::get<(int)POINT::Y>(led);
      pt.z = 0;
      led_marker.points.push_back(pt);
    }
    lm_msg.markers.push_back(led_marker);

    // Text
    visualization_msgs::msg::Marker text_marker(common_marker);
    text_marker.ns = "IDs";
    text_marker.text = stargazer::getHexIDstring(lm.id);
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.scale.z = 0.1;
    text_marker.color = YELLOW;
    text_marker.pose.position.x = middle;
    text_marker.pose.position.y = middle;
    text_marker.pose.position.z = -board_size * 0.2;
    text_marker.pose.orientation.w = 1;
    lm_msg.markers.push_back(text_marker);
  }

  // Start loop
  rclcpp::Rate r(params.rate);
  while (rclcpp::ok()) {
    rclcpp::Time timestamp = node->get_clock()->now();

    for (auto& transform : transforms) {
      transform.header.stamp = timestamp;
      transformBroadcaster.sendTransform(transform);
    }

    for (auto& marker : lm_msg.markers) {
      marker.header.stamp = timestamp;
    }
    lm_pub->publish(lm_msg);

    rclcpp::spin_some(node);
    r.sleep();
  }

  return 0;
}
