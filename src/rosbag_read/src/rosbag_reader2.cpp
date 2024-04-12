//
// Created by cyppt on 24-1-19.
//

#include "../include/rosbag_read2/rosbag_reader2.h"


/**
 * @brief read the config file in yaml format
 */
rosbag_reader2::rosbag_reader2() {
    YAML::Node config;
    std::cout << YELLOW << "yaml path: " << YAML_PATH << RESET << std::endl;
    config = YAML::LoadFile(YAML_PATH);

    bag_path = config["bag_path"].as<std::string>();
    save_root = config["save_root"].as<std::string>();
    ins_topic = config["ins_topic"].as<std::string>();
    camera_topics = config["camera_topics"].as<std::vector<std::string>>();
    cam_save_path = config["cam_save_path"].as<std::vector<std::string>>();
    std::vector<std::string> cam_name_temp = config["cam_name"].as<std::vector<std::string>>();
    save_all = config["save_all"].as<bool>();
    bag_format = config["bag_format"].as<bool>();
    save_interval = config["save_interval"].as<int>();
    save_one_index = config["save_one_index"].as<int>();
    debug_mode = config["debug_mode"].as<int>();
    read_ins_flag = config["read_ins_flag"].as<bool>();
    save_json_flag = config["save_json_flag"].as<bool>();
    for (auto &&cam:cam_save_path){
        if (save_all) cam = save_root + cam;
        else cam = save_root + "/";
    }

    for (int i = 0; i < 6; ++i) {
        cam_name[i] = cam_name_temp[i];
    }

    std::cout << CYAN << "bag_path: " << bag_path << std::endl;
    std::cout << CYAN << "save_root: " << save_root << std::endl;
    std::cout << RESET << std::endl;
    if(debug_mode == DEBUG_LOAD_DATA)
    {
        std::cout << YELLOW << "-----------------debug mode-------------------" << std::endl;
        std::cout << CYAN << "check the config file" << std::endl;
        std::cout << CYAN << "bag_path: " << bag_path << std::endl;
        std::cout << CYAN << "save_root: " << save_root << std::endl;
        std::cout << CYAN << "ins_topic: " << ins_topic << std::endl;
        std::cout << CYAN << "camera_topics: " << std::endl;
        for (int i = 0; i < camera_topics.size(); ++i) {
            std::cout << BLUE << "camera " << i + 1 << ": " << camera_topics[i] << std::endl;
        }
        std::cout << "cam_save_path: " << std::endl;
        for (int i = 0; i < cam_save_path.size(); ++i) {
            std::cout << BLUE << "camera " << i + 1 << ": " << cam_save_path[i] << std::endl;
        }
        std::cout << "cam_name: " << std::endl;
        for (int i = 0; i < 6; ++i) {
            std::cout << MAGENTA << "camera " << i + 1 << ": " << cam_name[i] << std::endl;
        }
        std::cout << GREEN << "save_all: " << save_all << std::endl;
        std::cout << GREEN << "save_one_index: " << save_one_index << std::endl;
        // reset the color
        std::cout << RESET << std::endl;
    }

    // free
    config.reset();
}


/**
 * @brief read the rosbag file
 */
void rosbag_reader2::read_rosbag() {
    // Open the rosbag file
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);

    // check and create save path
    // root
    if (access(save_root.c_str(), 0) == -1) {
        std::cout << "save root not exist, create it" << std::endl;
        mkdir(save_root.c_str(), 0777);
    }

    // check cam path
    for (int i = 0; i < 6; ++i) {
        if (access(cam_save_path[i].c_str(), 0) == -1) {
            std::cout << "camera " << i + 1 << " path not exist, create it" << std::endl;
            mkdir(cam_save_path[i].c_str(), 0777);
        }
    }

    // Create a view for all camera topics
    std::vector<rosbag::View *> view(camera_topics.size());
    for (int i = 0; i < camera_topics.size(); ++i) {
        view[i] = new rosbag::View(bag, rosbag::TopicQuery(camera_topics[i]));
    }

    // Check a view for ins topic
    rosbag::View* ins_view = nullptr;
    if (read_ins_flag) {
        ins_view = new rosbag::View(bag, rosbag::TopicQuery(ins_topic));
        if (debug_mode == DEBUG_LOAD_DATA) {
            // view ins data
            std::cout << YELLOW << "-----------------debug mode-------------------" << std::endl;
            rosbag_read::INSStdMsg::ConstPtr ins_msg = ins_view->begin()->instantiate<rosbag_read::INSStdMsg>();
            std::cout << "ins -> imu" << ins_msg->imu << std::endl;
            std::cout << "ins -> nav" << ins_msg->nav_sat_fix << std::endl;
            std::cout << "ins -> header" << ins_msg->header << std::endl;
        }
    }

    std::cout << GREEN << "Load rosbag file success" << RESET << std::endl;

    int total_frame_num = view[0]->size();
    for (int i = 0; i < camera_topics.size(); ++i) {
        if (total_frame_num > view[i]->size()) {
            total_frame_num = view[i]->size();
        }
    }

    std::cout << YELLOW << "total frame num: " << total_frame_num  << RESET << std::endl;

    if (save_all){
        if (read_ins_flag) {
            save_all_img_with_ins(view, camera_topics.size(), total_frame_num, ins_view);
        } else{
            save_all_img(view, camera_topics.size(), total_frame_num);
        }
    } else{
        save_one_group_img(view, camera_topics.size(), total_frame_num);
    }

    bag.close();
    std::cout << GREEN << "Save all images success" << RESET << std::endl;

    // free
    for (int i = 0; i < camera_topics.size(); ++i) {
        delete view[i];
    }
}

void rosbag_reader2::save_all_img(std::vector<rosbag::View *> view, int view_size, int total_frame_num) {
    int total_messages = total_frame_num * view_size;  // Total number of messages
    int current_message = 0;
    std::vector<std::vector<std::string>> img_name_vec(view_size);\
    Json::Value root;

    int barWidth = 70;
    for (int j = 0; j < view_size; ++j) {
        ros::Time last_time = view[j]->begin()->getTime() - ros::Duration(save_interval);
        for (const rosbag::MessageInstance &msg: *view[j]) {
            float progress = static_cast<float>(current_message) / total_messages;
            std::cout << MAGENTA << "[";
            int pos = barWidth * progress;
            for (int i = 0; i < barWidth; ++i) {
                if (i < pos) std::cout << "=";
                else if (i == pos) std::cout << ">";
                else std::cout << " ";
            }
            std::cout << "] " << int(progress * 100.0) << "%\r" << RESET;
            std::cout.flush();  // Manually flush the buffer

            ++current_message;

            sensor_msgs::CompressedImage::ConstPtr img_msg = msg.instantiate<sensor_msgs::CompressedImage>();
            ros::Time current_time = img_msg->header.stamp;
            if((current_time - last_time).toSec() >= save_interval){
                last_time = current_time;
                // Access image data
                std::vector<uint8_t> img_data = img_msg->data;

                // Display the image
                cv::Mat image = cv::imdecode(img_data, cv::IMREAD_COLOR);

                // Save the image
                // timestamp to datetime
                std::time_t time_stamp = msg.getTime().toNSec() / 1000000000;
                std::tm *tm_ptr = std::localtime(&time_stamp);
                std::string data_time = std::to_string(tm_ptr->tm_year + 1900) + "-" +
                                        std::to_string(tm_ptr->tm_mon + 1) + "-" +
                                        std::to_string(tm_ptr->tm_mday) + "-" +
                                        std::to_string(tm_ptr->tm_hour) + "-" +
                                        std::to_string(tm_ptr->tm_min) + "-" +
                                        std::to_string(tm_ptr->tm_sec);
                // Save the image
                std::string img_name = cam_name[j] + "_" + data_time + "_" + std::to_string(time_stamp) + ".jpg";
                cv::imwrite(cam_save_path[j] + img_name, image);
                img_name_vec[j].push_back(img_name);
            } // end if
        } // end for msg
    } // end for view_size
    std::cout << MAGENTA << "[";
    for (int i = 0; i < barWidth; ++i) {
        std::cout << "=";
    }
    std::cout << "] " << 100 << "%\r" << RESET;
    std::cout.flush();
    std::cout << std::endl;  // Move to the next line after the loop is done

    if (save_json_flag) {
        root = format_file_name_json(img_name_vec, save_root, cam_name);
        std::ofstream ofs(save_root + "/img_name.json");
        Json::StyledWriter styledWriter;
        ofs << styledWriter.write(root);
        ofs.close();
    }
}

void rosbag_reader2::save_one_group_img(std::vector<rosbag::View *> view, int view_size, int total_frame_num) {
    save_one_index = save_one_index % total_frame_num;
    for (int j = 0; j < view_size; ++j) {
        int i = 0;
        for (const rosbag::MessageInstance &msg: *view[j]) {
            if(i == save_one_index) {
                sensor_msgs::CompressedImage::ConstPtr img_msg = msg.instantiate<sensor_msgs::CompressedImage>();
                // Access image data
                std::vector<uint8_t> img_data = img_msg->data;

                // Display the image
                cv::Mat image = cv::imdecode(img_data, cv::IMREAD_COLOR);

                // Save the image
                // timestamp to datetime
                std::time_t time_stamp = msg.getTime().toNSec() / 1000000000;
                std::tm *tm_ptr = std::localtime(&time_stamp);
                std::string data_time = std::to_string(tm_ptr->tm_year + 1900) + "-" +
                                        std::to_string(tm_ptr->tm_mon + 1) + "-" +
                                        std::to_string(tm_ptr->tm_mday) + "-" +
                                        std::to_string(tm_ptr->tm_hour) + "-" +
                                        std::to_string(tm_ptr->tm_min) + "-" +
                                        std::to_string(tm_ptr->tm_sec);
                // Save the image
                std::string img_name = cam_name[j] + "_" + data_time + "_" + std::to_string(time_stamp) + ".jpg";
                cv::imwrite(cam_save_path[j] + img_name, image);
                break;
            } // end if
            i++;
        } // end for msg
    } // end for view_size
}

bool rosbag_reader2::fomat_bag_falg() {
    return bag_format;
}

void rosbag_reader2::save_all_img_with_ins(std::vector<rosbag::View *> view, int view_size, int total_frame_num,
                                           rosbag::View *ins_view) {
    int total_messages = total_frame_num * view_size;  // Total number of messages
    int current_message = 0;
    std::vector<std::vector<std::string>> img_name_vec(view_size);
    std::vector<std::vector<double>> ins_rotations_vec; // quaternion x y z w
    std::vector<std::vector<double>> ins_nav_vec; // latitude, longitude, altitude
    Json::Value root;

    rosbag::View::iterator it = ins_view->begin();

    int barWidth = 70;
    for (int j = 0; j < view_size; ++j) {
        ros::Time last_time = view[j]->begin()->getTime() - ros::Duration(save_interval);
        for (const rosbag::MessageInstance &msg: *view[j]) {
            float progress = static_cast<float>(current_message) / total_messages;
            std::cout << MAGENTA << "[";
            int pos = barWidth * progress;
            for (int i = 0; i < barWidth; ++i) {
                if (i < pos) std::cout << "=";
                else if (i == pos) std::cout << ">";
                else std::cout << " ";
            }
            std::cout << "] " << int(progress * 100.0) << "%\r" << RESET;
            std::cout.flush();  // Manually flush the buffer

            ++current_message;

            sensor_msgs::CompressedImage::ConstPtr img_msg = msg.instantiate<sensor_msgs::CompressedImage>();
            ros::Time current_time = img_msg->header.stamp;

            if((current_time - last_time).toSec() >= save_interval){
                last_time = current_time;

                // find the ins data
                rosbag_read::INSStdMsg::ConstPtr ins_msg = it->instantiate<rosbag_read::INSStdMsg>();
                rosbag_read::INSStdMsg::ConstPtr ins_msg_last = ins_msg;
                while (ins_msg->header.stamp < current_time){
                    ins_msg_last = ins_msg;
                    it++;
                    ins_msg = it->instantiate<rosbag_read::INSStdMsg>();
                }
                ros::Time ins_time_last = ins_msg_last->header.stamp;
                ros::Time ins_time = ins_msg->header.stamp;
                // choice the nearest ins data
                double ins_time_diff = (current_time - ins_time_last).toSec() / (ins_time - current_time).toSec();
                if (ins_time_diff < 1) ins_msg = ins_msg_last;
                ins_rotations_vec.push_back({ins_msg->imu.orientation.x, ins_msg->imu.orientation.y, ins_msg->imu.orientation.z, ins_msg->imu.orientation.w});
                ins_nav_vec.push_back({ins_msg->nav_sat_fix.latitude, ins_msg->nav_sat_fix.longitude, ins_msg->nav_sat_fix.altitude});

                // Access image data
                std::vector<uint8_t> img_data = img_msg->data;

                // Display the image
                cv::Mat image = cv::imdecode(img_data, cv::IMREAD_COLOR);

                // Save the image
                // timestamp to datetime
                std::time_t time_stamp = msg.getTime().toNSec() / 1000000000;
                std::tm *tm_ptr = std::localtime(&time_stamp);
                std::string data_time = std::to_string(tm_ptr->tm_year + 1900) + "-" +
                                        std::to_string(tm_ptr->tm_mon + 1) + "-" +
                                        std::to_string(tm_ptr->tm_mday) + "-" +
                                        std::to_string(tm_ptr->tm_hour) + "-" +
                                        std::to_string(tm_ptr->tm_min) + "-" +
                                        std::to_string(tm_ptr->tm_sec);
                // Save the image
                std::string img_name = cam_name[j] + "_" + data_time + "_" + std::to_string(time_stamp) + ".jpg";
                cv::imwrite(cam_save_path[j] + img_name, image);
                img_name_vec[j].push_back(img_name);
            } // end if
        } // end for msg
    } // end for view_size
    std::cout << MAGENTA << "[";
    for (int i = 0; i < barWidth; ++i) {
        std::cout << "=";
    }
    std::cout << "] " << 100 << "%\r" << RESET;
    std::cout.flush();
    std::cout << std::endl;  // Move to the next line after the loop is done

    if (save_json_flag) {
        root = format_file_name_with_ins_json(img_name_vec, save_root, cam_name, ins_rotations_vec, ins_nav_vec);
        std::ofstream ofs(save_root + "/img_name.json");
        Json::StyledWriter styledWriter;
        ofs << styledWriter.write(root);
        ofs.close();
    }
}




