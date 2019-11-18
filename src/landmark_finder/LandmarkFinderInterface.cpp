#include "LandmarkFinderInterface.h"

#include "../StargazerConversionMethods.h"
#include "../ros_utils.h"
#include <opencv2/features2d.hpp>

using namespace stargazer_ros_tool;

LandmarkFinderInterface::LandmarkFinderInterface(ros::NodeHandle nh_public,
                                                 ros::NodeHandle nh_private)
        : img_trans{nh_public}, server{nh_private} {

    params_.fromNodeHandle(nh_private);
    landmarkFinder = std::make_unique<stargazer::LandmarkFinder>(params_.map_config);
    server.setCallback(boost::bind(&LandmarkFinderInterface::reconfigureCallback, this, _1, _2));
    lm_pub = nh_private.advertise<stargazer_ros_tool::LandmarkArray>(params_.landmark_topic, 1);
    img_sub = img_trans.subscribe(
        params_.undistorted_image_topic, 1, &LandmarkFinderInterface::imgCallback, this);
    debugVisualizer_.SetWaitTime(10);

    if (params_.cfg.debug_mode)
        showNodeInfo();
}

void LandmarkFinderInterface::imgCallback(const sensor_msgs::ImageConstPtr& msg) {

    cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

    std::vector<stargazer::ImgLandmark> detected_img_landmarks;
    landmarkFinder->DetectLandmarks(cvPtr->image, detected_img_landmarks);

    // Convert
    stargazer_ros_tool::LandmarkArray landmarksMessage =
        convert2LandmarkMsg(detected_img_landmarks, msg->header);
    lm_pub.publish(landmarksMessage);

    //  Visualize
    if (!params_.cfg.debug_mode)
        return;

    // Invert input image
    cv::bitwise_not(landmarkFinder->grayImage_, landmarkFinder->grayImage_);

    // Draw detections
    auto point_img = debugVisualizer_.DrawPoints(landmarkFinder->grayImage_,
                                                 landmarkFinder->clusteredPixels_);
    auto cluster_img = debugVisualizer_.DrawClusters(landmarkFinder->grayImage_,
                                                     landmarkFinder->clusteredPoints_);
    auto hypotheses_img = debugVisualizer_.DrawLandmarkHypotheses(landmarkFinder->grayImage_,
                                                                  landmarkFinder->landmarkHypotheses_);
    auto landmarks_img = debugVisualizer_.DrawLandmarks(landmarkFinder->grayImage_,
                                                        detected_img_landmarks);

    // Show images
    debugVisualizer_.ShowImage(landmarkFinder->grayImage_, "0 Gray Image");
    debugVisualizer_.ShowImage(point_img, "1 Points");
    debugVisualizer_.ShowImage(cluster_img, "2 Clusters");
    debugVisualizer_.ShowImage(hypotheses_img, "3 Hypotheses");
    debugVisualizer_.ShowImage(landmarks_img, "4 Landmarks");
}

void LandmarkFinderInterface::reconfigureCallback(LandmarkFinderConfig& config,
                                                  const uint32_t& level) {

    params_.fromConfig(config, level);

    // Point Clustering
    landmarkFinder->maxRadiusForCluster = params_.cfg.maxRadiusForCluster;
    landmarkFinder->maxPointsPerLandmark = static_cast<uint16_t>(params_.cfg.maxPointsPerLandmark);
    landmarkFinder->minPointsPerLandmark = static_cast<uint16_t>(params_.cfg.minPointsPerLandmark);

    // Corner Detection
    landmarkFinder->maxCornerHypotheses = params_.cfg.maxCornerHypotheses;
    landmarkFinder->cornerHypothesesCutoff = params_.cfg.cornerHypothesesCutoff;
    landmarkFinder->fwLengthTriangle = params_.cfg.fwLengthTriangle;
    landmarkFinder->fwCrossProduct = params_.cfg.fwCrossProduct;
    landmarkFinder->cornerAngleTolerance = params_.cfg.cornerAngleTolerance;
    landmarkFinder->pointInsideTolerance = params_.cfg.pointInsideTolerance;

    landmarkFinder->blobParams.filterByArea = params_.cfg.blobFilterByArea;
    landmarkFinder->blobParams.filterByCircularity = params_.cfg.blobFilterByCircularity;
    landmarkFinder->blobParams.filterByConvexity = params_.cfg.blobFilterByConvexity;
    landmarkFinder->blobParams.filterByInertia = params_.cfg.blobFilterByInertia;
    landmarkFinder->blobParams.maxArea = params_.cfg.blobMaxArea;
    landmarkFinder->blobParams.maxCircularity = params_.cfg.blobMaxCircularity;
    landmarkFinder->blobParams.maxConvexity = params_.cfg.blobMaxConvexity;
    landmarkFinder->blobParams.maxInertiaRatio = params_.cfg.blobMaxInertiaRatio;
    landmarkFinder->blobParams.maxThreshold = params_.cfg.blobMaxThreshold;
    landmarkFinder->blobParams.minArea = params_.cfg.blobMinArea;
    landmarkFinder->blobParams.minCircularity = params_.cfg.blobMinCircularity;
    landmarkFinder->blobParams.minConvexity = params_.cfg.blobMinConvexity;
    landmarkFinder->blobParams.minDistBetweenBlobs = params_.cfg.blobMinDistBetweenBlobs;
    landmarkFinder->blobParams.minInertiaRatio = params_.cfg.blobMinInertiaRatio;
    landmarkFinder->blobParams.minRepeatability = params_.cfg.blobMinRepeatability;
    landmarkFinder->blobParams.minThreshold = params_.cfg.blobMinThreshold;
    landmarkFinder->blobParams.thresholdStep = params_.cfg.blobThresholdStep;
}
