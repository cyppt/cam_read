#ifndef __COMMON_FUN_H__
#define __COMMON_FUN_H__

#include <iostream>
#include <string>
#include <cstring> 
#include <fstream>  // 文件读取
#include <sstream>
#include <boost/filesystem.hpp>
#include <cmath>
#include <thread>
#include <chrono>
#include <vector>
#include <csignal> // 捕获ctrl + c 信号

#include "parameter.h"
#include "jsoncpp/json/json.h" // json
#include "yaml-cpp/yaml.h" // yaml


Json::Value format_file_name_json(const std::vector<std::vector<std::string>>& img_name_vec, const std::string& save_root, const std::string* cam_name);

Json::Value format_file_name_with_ins_json(const std::vector<std::vector<std::string>> &img_name_vec, const std::string &save_root,
                               const std::string *cam_name, std::vector<std::vector<double>> ins_rotations_vec, std::vector<std::vector<double>> ins_nav_vec);

std::vector<std::string> split(const std::string& s, char delimiter);

#endif // __COMMON_FUN_H__