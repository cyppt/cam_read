//
// Created by cyppt on 24-1-15.
//
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>

std::string bag_path = "/media/cyppt/新加卷/dataset/six_cam/route14_2021-12-29-15-54-06.bag";
// Define the camera save_path
std::string save_root = "/media/cyppt/新加卷/dataset/six_cam/route14/";
// Define the camera topics
std::vector<std::string> camera_topics = {
        "/camera1/compressed",
        "/camera2/compressed",
        "/camera3/compressed",
        "/camera4/compressed",
        "/camera5/compressed",
        "/camera6/compressed"
};

std::vector<std::string> save_path = {
        save_root + "camera1_front/",
        save_root + "camera2_front_right/",
        save_root + "camera3_back_right/",
        save_root + "camera4_back/",
        save_root + "camera5_back_left/",
        save_root + "camera6_front_left/"
};

std::string  cam_name[] = {"camera1_front", "camera2_front_right", "camera3_back_right",
                           "camera4_back", "camera5_back_left", "camera6_front_left"};

bool save_all = false;
int save_one_index = 1442;

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "rosbag_reader");
    ros::NodeHandle nh;

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
        if (access(save_path[i].c_str(), 0) == -1) {
            std::cout << "camera " << i + 1 << " path not exist, create it" << std::endl;
            mkdir(save_path[i].c_str(), 0777);
        }
    }

    // Create a view for all camera topics
    rosbag::View view(bag, rosbag::TopicQuery(camera_topics));

    std::cout << "Load rosbag file success" << std::endl;

    // Initialize OpenCV windows for each camera
    std::vector<cv::Mat> images(6);

    // Iterate through the messages
    int total_messages = view.size();  // Total number of messages
    int current_message = 0;           // Counter for the current message

//    for (int i = 0; i < 6; ++i) {
//        cv::namedWindow("Camera " + std::to_string(i + 1), cv::WINDOW_NORMAL);
//    }

    if(save_all) { // save all images
        // Iterate through the messages
        for (const rosbag::MessageInstance &msg: view) {
            // Display the progress bar
            float progress = static_cast<float>(current_message) / total_messages;
            int barWidth = 70;
            std::cout << "[";
            int pos = barWidth * progress;
            for (int i = 0; i < barWidth; ++i) {
                if (i < pos) std::cout << "=";
                else if (i == pos) std::cout << ">";
                else std::cout << " ";
            }
            std::cout << "] " << int(progress * 100.0) << "%\r" << std::flush;

            ++current_message;

            if (msg.getDataType() == "sensor_msgs/CompressedImage") {
                sensor_msgs::CompressedImage::ConstPtr img_msg = msg.instantiate<sensor_msgs::CompressedImage>();
                int camera_index = std::stoi(msg.getTopic().substr(7, 1)) - 1; // Extract camera index from topic

                // Access image data
                std::vector<uint8_t> img_data = img_msg->data;

                // Display the image
                cv::Mat image = cv::imdecode(img_data, cv::IMREAD_COLOR);
                images[camera_index] = image;

//            cv::imshow("Camera " + std::to_string(camera_index + 1), image);

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
                std::string img_name = cam_name[camera_index] + "_" + data_time + "_" + std::to_string(time_stamp) + ".jpg";
                cv::imwrite(save_path[camera_index] + img_name, image);
            }

//        // Break the loop if any window is closed
//        if (cv::waitKey(1) == 27) {
//            break;
//        }
        }

        std::cout << std::endl;  // Move to the next line after the loop is done
    }
    else{   // save one group of six images
        int six_cam_flag[6] = {0};
        std::vector<std::vector<cv::Mat>> six_cam_images;
        save_one_index = save_one_index % total_messages;
        std::cout << "save one image, index: " << save_one_index << std::endl;
        int i = 0;
        for(const rosbag::MessageInstance &msg: view){
            if(i == save_one_index) {
                sensor_msgs::CompressedImage::ConstPtr img_msg = msg.instantiate<sensor_msgs::CompressedImage>();
                int camera_index = std::stoi(msg.getTopic().substr(7, 1)) - 1; // Extract camera index from topic

                // Access image data
                std::vector<uint8_t> img_data = img_msg->data;

                // Display the image
                cv::Mat image = cv::imdecode(img_data, cv::IMREAD_COLOR);
                images[camera_index] = image;
                six_cam_flag[camera_index] = 1;
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
                std::string img_name = cam_name[camera_index] + "_" + data_time + "_" + std::to_string(time_stamp) + ".jpg";
                cv::imwrite(save_root + img_name, image);
                int sum_flag = 0;
                for (int j : six_cam_flag) {
                    sum_flag += j;
                }
                if(sum_flag == 6) break;
            }
            else i++;
        }
    }
    // Close OpenCV windows
//    for (int i = 0; i < 6; ++i) {
//        cv::destroyWindow("Camera " + std::to_string(i + 1));
//    }

    // Close the rosbag file
    bag.close();

    return 0;
}



