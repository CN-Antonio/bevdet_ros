#ifndef __BEVDET_H__
#define __BEVDET_H__

#include <string>
#include <iostream>

#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

class BEVDet
{
public:
    BEVDet();
    BEVDet(int num);
    BEVDet(const std::string &config_file, int n_img,      
            std::vector<Eigen::Matrix3f> _cams_intrin, 
            std::vector<Eigen::Quaternion<float>> _cams2ego_rot, 
            std::vector<Eigen::Translation3f> _cams2ego_trans,
            const std::string &engine_file,
            YAML::Node &config);
    ~BEVDet();
protected:
    int DoInfer();

    std::string config_file;    // yaml filename
    YAML::Node config_;

    size_t img_N_;
    int img_w_; 
    int img_h_;

    // 模型配置文件路径 
    std::string model_config_;
    
    // 权重文件路径 图像部分 bev部分
    std::string imgstage_file_;
    std::string bevstage_file_;
   
    // 相机的内外配置参数
    YAML::Node camconfig_; 
    
    // 结果保存文件
    std::string output_lidarbox_;


private:
    YAML::Node config;
    std::string model_config;
    std::string engine_file;
};

#endif // __BEVDET_H__
