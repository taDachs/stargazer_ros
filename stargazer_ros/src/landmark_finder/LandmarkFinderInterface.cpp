#include "LandmarkFinderInterface.h"

#include <opencv2/features2d.hpp>

#include "../StargazerConversionMethods.h"

using namespace stargazer_ros_tool;

LandmarkFinderInterface::LandmarkFinderInterface(rclcpp::Node::SharedPtr nh)
    : img_trans{nh} {

  param_listener = std::make_shared<landmark_finder::ParamListener>(nh);
  auto params = param_listener->get_params();
  landmarkFinder = std::make_unique<stargazer::LandmarkFinder>(params.map_config);
  lm_pub = nh->create_publisher<stargazer_interfaces::msg::LandmarkArray>(
      params.landmark_topic, 1);
  img_sub = img_trans.subscribe(
      params.undistorted_image_topic, 1, &LandmarkFinderInterface::imgCallback, this);
  debugVisualizer_.SetWaitTime(10);

  if (params.landmark_finder.debug_mode) {
    debug_pub_points =
        nh->create_publisher<sensor_msgs::msg::Image>("debug_1_points", 1);
    debug_pub_clusters =
        nh->create_publisher<sensor_msgs::msg::Image>("debug_2_clusters", 1);
    debug_pub_hypotheses =
        nh->create_publisher<sensor_msgs::msg::Image>("debug_3_hypotheses", 1);
    debug_pub_landmarks =
        nh->create_publisher<sensor_msgs::msg::Image>("debug_4_landmarks", 1);
  }
}

void LandmarkFinderInterface::imgCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  auto params = param_listener->get_params();
  updateFinderConfig();

  cv_bridge::CvImagePtr cvPtr =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

  std::vector<stargazer::ImgLandmark> detected_img_landmarks;
  landmarkFinder->DetectLandmarks(cvPtr->image, detected_img_landmarks);

  // Convert
  stargazer_interfaces::msg::LandmarkArray landmarksMessage =
      convert2LandmarkMsg(detected_img_landmarks, msg->header);
  lm_pub->publish(landmarksMessage);

  //  Visualize
  if (params.landmark_finder.debug_mode) {
    // Invert input image
    cv::bitwise_not(landmarkFinder->grayImage_, landmarkFinder->grayImage_);

    // Draw detections
    cv_bridge::CvImage out_msg;
    out_msg.header = msg->header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;

    out_msg.image = debugVisualizer_.DrawPoints(landmarkFinder->grayImage_,
                                                landmarkFinder->points_);
    debug_pub_points->publish(*out_msg.toImageMsg());

    out_msg.image = debugVisualizer_.DrawClusters(
        landmarkFinder->grayImage_, landmarkFinder->clusteredPoints_);
    debug_pub_clusters->publish(*out_msg.toImageMsg());

    out_msg.image = debugVisualizer_.DrawLandmarkHypotheses(
        landmarkFinder->grayImage_, landmarkFinder->landmarkHypotheses_);
    debug_pub_hypotheses->publish(*out_msg.toImageMsg());

    out_msg.image = debugVisualizer_.DrawLandmarks(landmarkFinder->grayImage_,
                                                   detected_img_landmarks);
    debug_pub_landmarks->publish(*out_msg.toImageMsg());
  }
}

void LandmarkFinderInterface::updateFinderConfig() {
  auto params = param_listener->get_params();

  // Point Clustering
  landmarkFinder->maxRadiusForCluster = params.landmark_finder.max_radius_for_cluster;
  landmarkFinder->maxPointsPerLandmark =
      static_cast<uint16_t>(params.landmark_finder.max_points_per_landmark);
  landmarkFinder->minPointsPerLandmark =
      static_cast<uint16_t>(params.landmark_finder.min_points_per_landmark);

  // Corner Detection
  landmarkFinder->maxCornerHypotheses = params.corner_detection.max_corner_hypotheses;
  landmarkFinder->cornerHypothesesCutoff = params.corner_detection.corner_hypotheses_cutoff;
  landmarkFinder->fwLengthTriangle = params.corner_detection.fw_length_triangle;
  landmarkFinder->fwCrossProduct = params.corner_detection.fw_cross_product;
  landmarkFinder->cornerAngleTolerance = params.corner_detection.corner_angle_tolerance;
  landmarkFinder->pointInsideTolerance = params.corner_detection.point_inside_tolerance;

  landmarkFinder->blobParams.filterByArea = params.blob_detection.blob_filter_by_area;
  landmarkFinder->blobParams.filterByCircularity =
      params.blob_detection.blob_filter_by_circularity;
  landmarkFinder->blobParams.filterByConvexity = params.blob_detection.blob_filter_by_convexity;
  landmarkFinder->blobParams.filterByInertia = params.blob_detection.blob_filter_by_inertia;
  landmarkFinder->blobParams.maxArea = params.blob_detection.blob_max_area;
  landmarkFinder->blobParams.maxCircularity = params.blob_detection.blob_max_circularity;
  landmarkFinder->blobParams.maxConvexity = params.blob_detection.blob_max_convexity;
  landmarkFinder->blobParams.maxInertiaRatio = params.blob_detection.blob_max_inertia_ratio;
  landmarkFinder->blobParams.maxThreshold = params.blob_detection.blob_max_threshold;
  landmarkFinder->blobParams.minArea = params.blob_detection.blob_min_area;
  landmarkFinder->blobParams.minCircularity = params.blob_detection.blob_min_circularity;
  landmarkFinder->blobParams.minConvexity = params.blob_detection.blob_min_convexity;
  landmarkFinder->blobParams.minDistBetweenBlobs =
      params.blob_detection.blob_min_dist_between_blobs;
  landmarkFinder->blobParams.minInertiaRatio = params.blob_detection.blob_min_inertia_ratio;
  landmarkFinder->blobParams.minRepeatability = params.blob_detection.blob_min_repeatability;
  landmarkFinder->blobParams.minThreshold = params.blob_detection.blob_min_threshold;
  landmarkFinder->blobParams.thresholdStep = params.blob_detection.blob_threshold_step;
}
