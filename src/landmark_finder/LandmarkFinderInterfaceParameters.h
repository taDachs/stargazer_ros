#pragma once

#include <string>
#include <ros/node_handle.h>
#include <stargazer_ros_tool/LandmarkFinderConfig.h>

namespace stargazer_ros_tool {

struct LandmarkFinderInterfaceParameters {

    void fromNodeHandle(const ros::NodeHandle&);
    void fromConfig(const LandmarkFinderConfig&, const uint32_t& = 0);

    std::string map_config;
    std::string landmark_topic;
    std::string undistorted_image_topic;

    LandmarkFinderConfig cfg;
};
}
