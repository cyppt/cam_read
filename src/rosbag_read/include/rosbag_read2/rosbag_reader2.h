//
// Created by cyppt on 24-1-19.
//

#ifndef CAM_READ_ROSBAG_READER2_H
#define CAM_READ_ROSBAG_READER2_H

#include "../common/common.h"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.8/pcl/io/pcd_io.h>



#include "jsoncpp/json/json.h"
#include "rosbag_read/INSStdMsg.h"

class rosbag_reader2 {
public:
    rosbag_reader2();

    ~rosbag_reader2() = default;

    void read_rosbag();

    bool fomat_bag_flag();

private:

    std::vector<ros::Time> save_all_img(std::vector<rosbag::View *> view, int view_size, int total_frame_num);

    void save_one_group_img(std::vector<rosbag::View *> view, int view_size, int total_frame_num);

    void save_new_freq_bag(std::vector<rosbag::View *> view, int view_size, int total_frame_num);

    std::vector<ros::Time> save_all_img_with_ins(std::vector<rosbag::View *> view, int view_size, int total_frame_num, rosbag::View *ins_view);

    void save_pcd(rosbag::View * view, int total_frame_num);

    void save_pcd(rosbag::View * view, int total_frame_num, std::vector<ros::Time> img_time);

    std::string bag_path;
    std::string save_root;
    std::string ins_topic;
    std::string lidar_topic;
    std::vector<std::string> camera_topics;
    std::vector<std::string> cam_save_path;
    std::string lidar_save_path;
    std::string cam_name[6];
    bool save_all;
    bool bag_format;
    bool read_ins_flag;
    bool read_lidar_flag;
    bool save_json_flag;
    int save_one_index;
    int debug_mode;
    int save_interval;
    ros::NodeHandle nh;
};


#endif //CAM_READ_ROSBAG_READER2_H
