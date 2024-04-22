#include "bevdet/bevdet.h"

BEVDet::BEVDet()
{
    std::cout<< "construct bevdet -no-params" <<std::endl;

}
BEVDet::BEVDet(int num)
{
    std::cout<< "construct bevdet" <<std::endl;
}
BEVDet::BEVDet(const std::string &config_file, int n_img,      
            std::vector<Eigen::Matrix3f> _cams_intrin, 
            std::vector<Eigen::Quaternion<float>> _cams2ego_rot, 
            std::vector<Eigen::Translation3f> _cams2ego_trans,
            const std::string &engine_file,
            YAML::Node &config)
{
    std::cout<< "construct bevdet" <<std::endl;
}

int BEVDet::DoInfer()
{
    std::cout<< "Do Infer" <<std::endl;
}

BEVDet::~BEVDet()
{}
