//
// Created by cyppt on 24-1-19.
//

#include "../include/rosbag_read2/rosbag_reader2.h"
#include "../include/rosbag_format/rosbag_format.h"


void SigIntHandler(int sig);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosbag_reader2");
    ros::NodeHandle nh;
    rosbag_reader2 reader;
    signal(SIGINT, SigIntHandler);
    rosbag_format format;

    std::cout << CYAN << "start to read rosbag" << RESET << std::endl;
    reader.read_rosbag();
    std::cout << MAGENTA << "read rosbag done" << RESET << std::endl;

    if (reader.fomat_bag_falg()) {
        std::cout << CYAN << "start to format rosbag" << RESET << std::endl;
        format.format_rosbag();
        std::cout << MAGENTA << "format rosbag done" << RESET << std::endl;
    }
    return 0;
}

/**
 * @brief SIGINT handler when Ctrl+C is pressed print log and stop ros node
*/
void SigIntHandler(int sig)
{
    std::cout << std::endl << "Ctrl+C pressed." << std::endl;
    // 停止ROS节点
    ros::shutdown();
    exit(0);
}