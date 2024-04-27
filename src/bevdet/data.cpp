#include <iostream>

#include "bevdet/data.h"

camParams::camParams(const YAML::Node &config, int n, std::vector<std::string> &cams_name) :
                                                                    N_img(n){
    
    if((size_t)n != cams_name.size())
    {
        std::cerr << "Error! Need " << n << " camera param, bug given " << cams_name.size() << " camera names!" << std::endl;
    }
    ego2global_rot = fromYamlQuater(config["ego2global_rotation"]);
    ego2global_trans = fromYamlTrans(config["ego2global_translation"]);

    lidar2ego_rot = fromYamlQuater(config["lidar2ego_rotation"]);
    lidar2ego_trans = fromYamlTrans(config["lidar2ego_translation"]);

    timestamp = config["timestamp"].as<unsigned long long>();
    scene_token = config["scene_token"].as<std::string>();

    imgs_file.clear();

    cams_intrin.clear();
    cams2ego_rot.clear();
    cams2ego_trans.clear();
    
    for(std::string name : cams_name)
    {
        imgs_file.push_back("." + config["cams"][name]["data_path"].as<std::string>());

        //
        cams_intrin.push_back(fromYamlMatrix3f(config["cams"][name]["cam_intrinsic"]));
        cams2ego_rot.push_back(fromYamlQuater(config["cams"][name]["sensor2ego_rotation"]));
        cams2ego_trans.push_back(fromYamlTrans(config["cams"][name]["sensor2ego_translation"]));
        //

    }
}

Eigen::Translation3f fromYamlTrans(YAML::Node x){
    std::vector<float> trans = x.as<std::vector<float>>();
    return Eigen::Translation3f(trans[0], trans[1], trans[2]);
}

Eigen::Quaternion<float> fromYamlQuater(YAML::Node x){
    std::vector<float> quater = x.as<std::vector<float>>();
    return Eigen::Quaternion<float>(quater[0], quater[1], quater[2], quater[3]);
}

Eigen::Matrix3f fromYamlMatrix3f(YAML::Node x){
    std::vector<std::vector<float>> m = x.as<std::vector<std::vector<float>>>();
    Eigen::Matrix3f mat;
    for(size_t i = 0; i < m.size(); i++){
        for(size_t j = 0; j < m[0].size(); j++){
            mat(i, j) = m[i][j];
        }
    }
    return mat;
}
