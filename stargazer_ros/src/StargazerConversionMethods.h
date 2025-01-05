//
// Created by bandera on 10.06.16.
//

#pragma once
#include <ceres/rotation.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "stargazer/StargazerImgTypes.h"
#include "stargazer/StargazerTypes.h"
#include <stargazer_interfaces/msg/landmark.hpp>
#include <stargazer_interfaces/msg/landmark_array.hpp>

namespace stargazer_ros_tool {

inline stargazer::Landmark convert2Landmark(const stargazer_interfaces::msg::Landmark& lm_in) {

  stargazer::Landmark lm_out(lm_in.id);
  lm_out.points.clear();
  lm_out.points.reserve(lm_in.corner_points.size() + lm_in.id_points.size());

  for (auto& el : lm_in.corner_points) {
    stargazer::Point pt = {(double)el.u, (double)el.v, 0};
    lm_out.points.push_back(pt);
  }
  for (auto& el : lm_in.id_points) {
    stargazer::Point pt = {(double)el.u, (double)el.v, 0};
    lm_out.points.push_back(pt);
  }

  return lm_out;
};

inline stargazer::ImgLandmark convert2ImgLandmark(const stargazer_interfaces::msg::Landmark& lm_in) {

  stargazer::ImgLandmark lm_out;
  lm_out.nID = lm_in.id;

  // corners
  std::transform(lm_in.corner_points.begin(),
                 lm_in.corner_points.end(),
                 lm_out.corners.begin(),
                 [](auto& el) {
                   cv::Point pt;
                   pt.x = el.u;
                   pt.y = el.v;
                   return pt;
                 });

  lm_out.idPoints.reserve(lm_in.id_points.size());
  for (auto& el : lm_in.id_points) {
    cv::Point pt;
    pt.x = el.u;
    pt.y = el.v;
    lm_out.idPoints.push_back(pt);
  }

  return lm_out;
};

inline std::vector<stargazer::Landmark> convert2Landmarks(const stargazer_interfaces::msg::LandmarkArray& lms_in) {
  std::vector<stargazer::Landmark> lms_out;
  lms_out.reserve(lms_in.landmarks.size());

  for (auto& lm_in : lms_in.landmarks) {
    lms_out.push_back(convert2Landmark(lm_in));
  }

  return lms_out;
}

inline std::vector<stargazer::ImgLandmark> convert2ImgLandmarks(
    const stargazer_interfaces::msg::LandmarkArray& lms_in) {
  std::vector<stargazer::ImgLandmark> lms_out;
  lms_out.reserve(lms_in.landmarks.size());

  for (auto& lm_in : lms_in.landmarks) {
    lms_out.push_back(convert2ImgLandmark(lm_in));
  }

  return lms_out;
}

inline stargazer_interfaces::msg::LandmarkPoint convert2LandmarkPoint(const cv::Point& pt) {
  stargazer_interfaces::msg::LandmarkPoint lmpt;
  lmpt.u = static_cast<stargazer_interfaces::msg::LandmarkPoint::_u_type>(pt.x);
  lmpt.v = static_cast<stargazer_interfaces::msg::LandmarkPoint::_v_type>(pt.y);
  return lmpt;
}

inline stargazer_interfaces::msg::LandmarkArray convert2LandmarkMsg(
    const std::vector<stargazer::ImgLandmark>& lm_in, std_msgs::msg::Header header = std_msgs::msg::Header()) {

  stargazer_interfaces::msg::LandmarkArray landmarksMessage;
  landmarksMessage.landmarks.reserve(lm_in.size());
  landmarksMessage.header = header;

  for (auto& lm : lm_in) {
    stargazer_interfaces::msg::Landmark landmark;
    landmark.header = header;
    landmark.id = lm.nID;

    // corners
    std::transform(lm.corners.begin(),
                   lm.corners.end(),
                   landmark.corner_points.begin(),
                   [](auto& pt) { return convert2LandmarkPoint(pt); });

    for (auto& pt : lm.idPoints) {
      landmark.id_points.push_back(convert2LandmarkPoint(pt));
    }

    landmarksMessage.landmarks.push_back(landmark);
  }

  return landmarksMessage;
}

inline void pose2tf(const stargazer::Pose pose_in, geometry_msgs::msg::TransformStamped& transform) {
  using namespace stargazer;
  transform.transform.translation.x = pose_in.position[(int)POINT::X];
  transform.transform.translation.y = pose_in.position[(int)POINT::Y];
  transform.transform.translation.z = pose_in.position[(int)POINT::Z];
  transform.transform.rotation.w = pose_in.orientation[(int)QUAT::W];
  transform.transform.rotation.x = pose_in.orientation[(int)QUAT::X];
  transform.transform.rotation.y = pose_in.orientation[(int)QUAT::Y];
  transform.transform.rotation.z = pose_in.orientation[(int)QUAT::Z];
  return;
}

inline geometry_msgs::msg::Pose pose2gmPose(const stargazer::Pose& pose_in) {
  using namespace stargazer;
  geometry_msgs::msg::Pose pose_out;
  pose_out.position.x = pose_in.position[(int)POINT::X];
  pose_out.position.y = pose_in.position[(int)POINT::Y];
  pose_out.position.z = pose_in.position[(int)POINT::Z];
  pose_out.orientation.w = pose_in.orientation[(int)QUAT::W];
  pose_out.orientation.x = pose_in.orientation[(int)QUAT::X];
  pose_out.orientation.y = pose_in.orientation[(int)QUAT::Y];
  pose_out.orientation.z = pose_in.orientation[(int)QUAT::Z];
  return pose_out;
}

inline stargazer::Pose gmPose2pose(const geometry_msgs::msg::Pose& pose_in) {
  using namespace stargazer;
  stargazer::Pose pose_out;
  pose_out.position[(int)POINT::X] = pose_in.position.x;
  pose_out.position[(int)POINT::Y] = pose_in.position.y;
  pose_out.position[(int)POINT::Z] = pose_in.position.z;
  pose_out.orientation[(int)QUAT::W] = pose_in.orientation.w;
  pose_out.orientation[(int)QUAT::X] = pose_in.orientation.x;
  pose_out.orientation[(int)QUAT::Y] = pose_in.orientation.y;
  pose_out.orientation[(int)QUAT::Z] = pose_in.orientation.z;
  return pose_out;
}

}  // namespace stargazer_ros_tool
