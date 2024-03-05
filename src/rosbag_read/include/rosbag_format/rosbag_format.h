//
// Created by cyppt on 24-1-23.
//

#ifndef CAM_READ_ROSBAG_FORMAT_H
#define CAM_READ_ROSBAG_FORMAT_H

#include "../common/common.h"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <cv_bridge/cv_bridge.h>

#include "jsoncpp/json/json.h"

class rosbag_format {
public:
    rosbag_format();

    ~rosbag_format() = default;

    void format_rosbag();

private:
    std::string img_path_json;
    std::string save_root;
    std::vector<std::string> camera_topics;

    std::vector<std::vector<std::string>> img_path_vec;

    /// @brief timesamp vec
    std::vector<std::vector<time_t>> timestamp_vec;
};


#endif //CAM_READ_ROSBAG_FORMAT_H
