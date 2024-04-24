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
    lidar_topic = config["lidar_topic"].as<std::string>();
    cam_save_path = config["cam_save_path"].as<std::vector<std::string>>();
    lidar_save_path = config["lidar_save_path"].as<std::string>();
    std::vector<std::string> cam_name_temp = config["cam_name"].as<std::vector<std::string>>();
    save_all = config["save_all"].as<bool>();
    bag_format = config["bag_format"].as<bool>();
    save_interval = config["save_interval"].as<int>();
    save_one_index = config["save_one_index"].as<int>();
    debug_mode = config["debug_mode"].as<int>();
    read_ins_flag = config["read_ins_flag"].as<bool>();
    read_lidar_flag = config["read_lidar_flag"].as<bool>();
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

    std::vector<ros::Time> cam_timestamp_vec;

    if (save_all){
        if (read_ins_flag) {
            cam_timestamp_vec = save_all_img_with_ins(view, camera_topics.size(), total_frame_num, ins_view);
        } else{
            cam_timestamp_vec = save_all_img(view, camera_topics.size(), total_frame_num);
        }
    } else{
        save_one_group_img(view, camera_topics.size(), total_frame_num);
    }

    rosbag::View* lidar_view = nullptr;
    int lidar_frame_num = 0;
    if (read_lidar_flag){
        lidar_view = new rosbag::View(bag, rosbag::TopicQuery(lidar_topic));
        std::cout << GREEN << "Begin to save lidar data" << RESET << std::endl;
        if (debug_mode == DEBUG_LOAD_DATA) {
            // view lidar data
            std::cout << YELLOW << "-----------------debug mode-------------------" << std::endl;
            sensor_msgs::PointCloud2::ConstPtr lidar_msg = lidar_view->begin()->instantiate<sensor_msgs::PointCloud2>();
            std::cout << "lidar -> header" << lidar_msg->header << std::endl;
        }
        lidar_frame_num = lidar_view->size();

        std::string lidar_save = save_root + lidar_save_path;
        if (access(lidar_save.c_str(), 0) == -1) {
            std::cout << "lidar path not exist, create it" << std::endl;
            mkdir(lidar_save.c_str(), 0777);
        }

        save_pcd(lidar_view, lidar_frame_num, cam_timestamp_vec);
    }

    bag.close();
    std::cout << GREEN << "Save all images success" << RESET << std::endl;

    // free
    for (int i = 0; i < camera_topics.size(); ++i) {
        delete view[i];
    }
}

/**
 * @brief save all images
 * @param view vector of rosbag::View
 * @param view_size num of camera topics
 * @param total_frame_num total frame num
 */
std::vector<ros::Time> rosbag_reader2::save_all_img(std::vector<rosbag::View *> view, int view_size, int total_frame_num) {
    int total_messages = total_frame_num * view_size;  // Total number of messages
    int current_message = 0;
    std::vector<std::vector<std::string>> img_name_vec(view_size);
    Json::Value root;
    std::vector<ros::Time> cam_timestamp_vec;

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
                if (j == 0){
                    // save the timestamp
                    cam_timestamp_vec.push_back(current_time);
                }
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

    return cam_timestamp_vec;
}

/**
 * @brief save one group images
 * @param view vector of rosbag::View
 * @param view_size num of camera topics
 * @param total_frame_num total frame num
 */
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

/**
 * @brief return the bag format flag
 * @return bag_format
 */
bool rosbag_reader2::fomat_bag_falg() {
    return bag_format;
}


/**
 * @brief save all images with ins data
 * @param view vector of rosbag::View
 * @param view_size num of camera topics
 * @param total_frame_num total frame num
 * @param ins_view rosbag::View of ins data
 */
std::vector<ros::Time> rosbag_reader2::save_all_img_with_ins(std::vector<rosbag::View *> view, int view_size, int total_frame_num,
                                           rosbag::View *ins_view) {
    int total_messages = total_frame_num * view_size;  // Total number of messages
    int current_message = 0;
    std::vector<std::vector<std::string>> img_name_vec(view_size);
    std::vector<std::vector<double>> ins_rotations_vec; // quaternion x y z w
    std::vector<std::vector<double>> ins_nav_vec; // latitude, longitude, altitude
    Json::Value root;
    std::vector<ros::Time> cam_timestamp_vec;

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
                if (j == 0){
                    // save the timestamp
                    cam_timestamp_vec.push_back(current_time);
                }

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

    return cam_timestamp_vec;
}

/**
 * @brief save pcd file
 * @param view vector of rosbag::View
 * @param total_frame_num total frame num
 */
void rosbag_reader2::save_pcd(rosbag::View * view, int total_frame_num) {
    int total_messages = total_frame_num;  // Total number
    int current_message = 0;
    int barWidth = 70;

    ros::Time last_time = view->begin()->getTime() - ros::Duration(save_interval);

    for (const rosbag::MessageInstance &msg: *view) {
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

        sensor_msgs::PointCloud2::ConstPtr lidar_msg = msg.instantiate<sensor_msgs::PointCloud2>();

        ros::Time current_time = lidar_msg->header.stamp;
        if((current_time - last_time).toSec() >= save_interval){
            last_time = current_time;
            // save pcd file
            std::string pcd_name = "lidar_" + std::to_string(current_time.toNSec() / 1000000000) + ".pcd";
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*lidar_msg, *cloud);
            pcl::io::savePCDFileBinary(save_root + lidar_save_path + pcd_name, *cloud);
        } // end if
    } // end for msg

    std::cout << MAGENTA << "[";
    for (int i = 0; i < barWidth; ++i) {
        std::cout << "=";
    }
    std::cout << "] " << 100 << "%\r" << RESET;
    std::cout.flush();
    std::cout << std::endl;  // Move to the next line after the loop is done
}

void rosbag_reader2::save_pcd(rosbag::View *view, int total_frame_num, std::vector<ros::Time> img_time) {
    int total_messages = total_frame_num;  // Total number
    int current_message = 0;
    int barWidth = 70;
    int img_index = 0;

    std::cout << "total lidar frame num: " << total_frame_num << std::endl;
    std::cout << "total img frame num: " << img_time.size() << std::endl;

    double last_time_diff = 0x3f3f;
    sensor_msgs::PointCloud2::ConstPtr last_lidar_msg = view->begin()->instantiate<sensor_msgs::PointCloud2>();

    for (const rosbag::MessageInstance &msg: *view) {
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

        sensor_msgs::PointCloud2::ConstPtr lidar_msg = msg.instantiate<sensor_msgs::PointCloud2>();

        ros::Time current_time = lidar_msg->header.stamp;
        ros::Time img_time_current = img_time[img_index];
        double time_diff = (current_time - img_time_current).toSec();
        time_diff = time_diff > 0 ? time_diff : -time_diff;
        if (time_diff <= last_time_diff){
            last_time_diff = time_diff;
            last_lidar_msg = lidar_msg;
        } else{
            // save pcd file
            std::string pcd_name = "lidar_" + std::to_string(last_lidar_msg->header.stamp.toNSec() / 1000000000) + ".pcd";
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*last_lidar_msg, *cloud);
            pcl::io::savePCDFileBinary(save_root + lidar_save_path + pcd_name, *cloud);
            last_time_diff = 0x3f3f;
            img_index++;
        } // end if

    } // end for msg

    std::cout << MAGENTA << "[";
    for (int i = 0; i < barWidth; ++i) {
        std::cout << "=";
    }
    std::cout << "] " << 100 << "%\r" << RESET;
    std::cout.flush();
    std::cout << std:: endl;  // Move to the next line after the loop is done
}




