//
// Created by cyppt on 24-1-23.
//

#include "../include/rosbag_format/rosbag_format.h"

rosbag_format::rosbag_format() {
    YAML::Node config;
    std::cout << YELLOW << "yaml path: " << BAG_FORMAT_YAML << RESET << std::endl;
    config = YAML::LoadFile(BAG_FORMAT_YAML);

    save_root = config["save_root"].as<std::string>();
    camera_topics = config["camera_topics"].as<std::vector<std::string>>();
    img_path_json = config["img_path_json"].as<std::string>();

    if (DEBUG_LOAD_DATA) {
        std::cout << YELLOW << "-----------------debug mode-------------------" << std::endl;
        std::cout << CYAN << "check the config file" << std::endl;
        std::cout << CYAN << "save_root: " << save_root << std::endl;
        std::cout << CYAN << "camera_topics: " << std::endl;
        for (int i = 0; i < camera_topics.size(); ++i) {
            std::cout << BLUE << "camera " << i + 1 << ": " << camera_topics[i] << std::endl;
        }
        std::cout << MAGENTA << "img_path_json: " << img_path_json << std::endl;
        // reset the color
        std::cout << RESET << std::endl;
    }

    std::vector<std::vector<std::string>> img_path_vec_temp(camera_topics.size());
    std::vector<std::vector<time_t>> timestamp_vec_temp(camera_topics.size());

    // read the img_path_json
    std::ifstream ifs(img_path_json);
    Json::Reader reader;
    Json::Value root;
    reader.parse(ifs, root);
    Json::Value key_frames = root["key_frames"];
    for (auto key_frame : key_frames) {
        for (int i = 0; i < key_frame.size(); ++i) {
            Json::Value cam = key_frame[i];
            auto temp_name = cam["img_name"];
            // "img_name" : "camera0_2023-4-20-16-3-15_1681977795.jpg",
            // 按照 _ 分割
            auto temp_time_stamp_str = split(temp_name.asString(), '_')[2];
            time_t temp_time_stamp = std::stoi(split(temp_time_stamp_str, '.')[0]);

            std::string temp_path = cam["img_path"].asString();
            img_path_vec_temp[i].push_back(temp_path);
            timestamp_vec_temp[i].push_back(temp_time_stamp);
        }
    }

    img_path_vec = img_path_vec_temp;
    timestamp_vec = timestamp_vec_temp;

    // free
    config.reset();
}


/**
 * @brief format the rosbag from img file
 */
void rosbag_format::format_rosbag() {
    rosbag::Bag bag;
    bag.open(save_root + "/format.bag", rosbag::bagmode::Write);

    int total_messages = camera_topics.size() * img_path_vec[0].size();
    int current_message = 0;
    int barWidth = 70;

    for (int i = 0; i < camera_topics.size(); ++i) {
        std::string cam_topic = camera_topics[i];
        std::vector<std::string> img_path_vec_temp = img_path_vec[i];
        std::vector<time_t> timestamp_vec_temp = timestamp_vec[i];

        for (int j = 0; j < img_path_vec_temp.size(); ++j) {

            float progress = static_cast<float>(current_message) / total_messages;
            std::cout << GREEN << "[";
            int pos = barWidth * progress;
            for (int i = 0; i < barWidth; ++i) {
                if (i < pos) std::cout << "=";
                else if (i == pos) std::cout << ">";
                else std::cout << " ";
            }
            std::cout << "] " << int(progress * 100.0) << "%\r" << RESET;
            std::cout.flush();  // Manually flush the buffer

            std::string img_path = img_path_vec_temp[j];
            time_t timestamp = timestamp_vec_temp[j];

            cv::Mat img = cv::imread(img_path);
            sensor_msgs::CompressedImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toCompressedImageMsg();
            msg->header.stamp = ros::Time(timestamp);
            bag.write(cam_topic, ros::Time(timestamp), msg);
            current_message++;
        }
    }

    std::cout << GREEN << "[";
    for (int i = 0; i < barWidth; ++i) {
        std::cout << "=";
    }
    std::cout << "] " << 100 << "%\r" << RESET;
    std::cout.flush();
    std::cout << std::endl;

    bag.close();
}
