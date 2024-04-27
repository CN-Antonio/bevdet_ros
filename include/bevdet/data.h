#ifndef __DATA_H__
#define __DATA_H__

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include "bevdet/common.h"

struct camParams{
    camParams() = default;
    camParams(const YAML::Node &config, int n, std::vector<std::string> &cams_name);

    int N_img;  // 6

    Eigen::Quaternion<float> ego2global_rot;
    Eigen::Translation3f ego2global_trans;

    Eigen::Quaternion<float> lidar2ego_rot;
    Eigen::Translation3f lidar2ego_trans;
    // 
    std::vector<Eigen::Matrix3f> cams_intrin;
    std::vector<Eigen::Quaternion<float>> cams2ego_rot;
    std::vector<Eigen::Translation3f> cams2ego_trans;
    //
    std::vector<std::string> imgs_file;

    unsigned long long timestamp;
    std::string scene_token;
    
    // to delete
};

struct camsData{
    camsData() = default;
    camsData(const camParams &_param) : param(_param), imgs_dev(nullptr){};
    camParams param;
    uchar* imgs_dev;
};

Eigen::Translation3f fromYamlTrans(YAML::Node x);
Eigen::Quaternion<float> fromYamlQuater(YAML::Node x);
Eigen::Matrix3f fromYamlMatrix3f(YAML::Node x);

#endif // __DATA_H__
