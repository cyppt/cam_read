#include "common.h"

/**
 * @brief read the strint and convert to json
 */
Json::Value format_file_name_json(const std::vector<std::vector<std::string>>& img_name_vec, const std::string& save_root, const std::string* cam_name){
    Json::Value root;
    Json::Value key_frames;

    for (int i = 0; i < img_name_vec[0].size(); ++i) {
        Json::Value key_frame;
        for (int j = 0; j < img_name_vec.size(); ++j) {
            Json::Value cam;
            cam["img_name"] = img_name_vec[j][i];
            cam["img_path"] = save_root + "/" + cam_name[j] + "/" + img_name_vec[j][i];
            key_frame.append(cam);
        }
        key_frames.append(key_frame);
    }

    root["key_frames"] = key_frames;

    return root;
}

std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);

    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }

    return tokens;
}

Json::Value
format_file_name_with_ins_json(const std::vector<std::vector<std::string>> &img_name_vec, const std::string &save_root,
                               const std::string *cam_name, std::vector<std::vector<double>> ins_rotations_vec, std::vector<std::vector<double>> ins_nav_vec) {
    Json::Value root;
    Json::Value key_frames;

    for (int i = 0; i < img_name_vec[0].size(); ++i) {
        Json::Value key_frame;
        for (int j = 0; j < img_name_vec.size(); ++j) {
            Json::Value cam;
            cam["img_name"] = img_name_vec[j][i];
            cam["img_path"] = save_root + "/" + cam_name[j] + "/" + img_name_vec[j][i];
            cam["orientation"]["x"] = ins_rotations_vec[i][0];
            cam["orientation"]["y"] = ins_rotations_vec[i][1];
            cam["orientation"]["z"] = ins_rotations_vec[i][2];
            cam["orientation"]["w"] = ins_rotations_vec[i][3];
            cam["nav"]["latitude"] = ins_nav_vec[i][0];
            cam["nav"]["longitude"] = ins_nav_vec[i][1];
            cam["nav"]["altitude"] = ins_nav_vec[i][2];
            key_frame.append(cam);
        }
        key_frames.append(key_frame);
    }

    root["key_frames"] = key_frames;

    return root;
}
