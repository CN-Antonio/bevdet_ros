#ifndef __BEVDET_H__
#define __BEVDET_H__

#include <string>
#include <iostream>

#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/common/transforms.h>

#include <cuda_runtime.h>
#include "NvInfer.h"

#include "bevdet/data.h"
#include "bevdet/preprocess.h"
#include "bevdet/postprocess.h"

typedef pcl::PointXYZI PointT;

class Logger : public nvinfer1::ILogger {
public:
    explicit Logger(Severity severity = Severity::kWARNING) : reportable_severity(severity){}

    void log(Severity severity, const char *msg) noexcept override {
        // suppress messages with severity enum value greater than the reportable
        if (severity > reportable_severity) return;
        switch (severity) {
        case Severity::kINTERNAL_ERROR:
            std::cerr << "INTERNAL_ERROR: ";
            break;
        case Severity::kERROR:
            std::cerr << "ERROR: ";
            break;
        case Severity::kWARNING:
            std::cerr << "WARNING: ";
            break;
        case Severity::kINFO:
            std::cerr << "INFO: ";
            break;
        default:
            std::cerr << "UNKNOWN: ";
            break;
        }
        std::cerr << msg << std::endl;
        }

    Severity reportable_severity;
};

struct adjFrame{

    adjFrame(){}
    adjFrame(int _n,
             int _map_size, 
             int _bev_channel) : 
             n(_n), 
             map_size(_map_size), 
             bev_channel(_bev_channel),
             scenes_token(_n),
             ego2global_rot(_n),
             ego2global_trans(_n) {
        CHECK_CUDA(cudaMalloc((void**)&adj_buffer, _n * _map_size * _bev_channel * sizeof(float)));
    }  
    const std::string& lastScenesToken() const{
        return scenes_token[last];
    }

    void reset(){
        last = -1;
        buffer_num = 0;
    }

    void saveFrameBuffer(const float* curr_buffer, const std::string &curr_token, 
                                            const Eigen::Quaternion<float> &_ego2global_rot,
                                            const Eigen::Translation3f &_ego2global_trans){
        last = (last + 1) % n;
        CHECK_CUDA(cudaMemcpy(adj_buffer + last * map_size * bev_channel, curr_buffer,
                        map_size * bev_channel * sizeof(float), cudaMemcpyDeviceToDevice));
        scenes_token[last] = curr_token;
        ego2global_rot[last] = _ego2global_rot;
        ego2global_trans[last] = _ego2global_trans;
        buffer_num = std::min(buffer_num + 1, n);
    }
    const float* getFrameBuffer(int idx){
        idx = (-idx + last + n) % n;
        return adj_buffer + idx * map_size * bev_channel;
    }
    void getEgo2Global(int idx, Eigen::Quaternion<float> &adj_ego2global_rot, 
                                                Eigen::Translation3f &adj_ego2global_trans){
        idx = (-idx + last + n) % n;
        adj_ego2global_rot = ego2global_rot[idx];
        adj_ego2global_trans = ego2global_trans[idx];
    }

    ~adjFrame(){
        CHECK_CUDA(cudaFree(adj_buffer));
    }

    int n;
    int map_size;
    int bev_channel;

    int last;
    int buffer_num;

    std::vector<std::string> scenes_token;
    std::vector<Eigen::Quaternion<float>> ego2global_rot;
    std::vector<Eigen::Translation3f> ego2global_trans;

    float* adj_buffer;
};

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
    // Init
    void InitParams(const std::string &config_file);
    void InitViewTransformer();
    void MallocDeviceMemory();
    int DeserializeTRTEngine(const std::string &engine_file, nvinfer1::ICudaEngine **engine_ptr);
    int InitEngine(const std::string &imgstage_file, const std::string &bevstage_file);
    // Infer
    void InitDepth(const std::vector<Eigen::Quaternion<float>> &curr_cams2ego_rot,
                   const std::vector<Eigen::Translation3f> &curr_cams2ego_trans,
                   const std::vector<Eigen::Matrix3f> &cur_cams_intrin);
    int DoInfer();
    int DoInfer(const camsData &cam_data);
    int DoInfer(const camsData &cam_data,  
                std::vector<Box> &out_detections, 
                float &cost_time,
                int idx=-1);

    // std::string config_file;    // yaml filename
    YAML::Node config_;

    size_t img_N_;
    int img_w_; 
    int img_h_;

    // 模型配置文件路径 
    std::string model_config_;
    
    // 权重文件路径 图像部分 bev部分
    std::string imgstage_file;
    std::string bevstage_file;
   
    // 相机的内外配置参数
    YAML::Node camconfig_;

    YAML::Node sample_;

    std::vector<std::string> imgs_file_;
    std::vector<std::string> imgs_name_;

    camsData sampleData_;
    uchar* imgs_dev_ = nullptr; // stitched, memory_gpu
    
    // 结果保存文件
    std::string output_lidarbox_;

    // temp protected
    std::vector<Eigen::Matrix3f> cams_intrin;
    std::vector<Eigen::Quaternion<float>> cams2ego_rot;
    std::vector<Eigen::Translation3f> cams2ego_trans;


private:
    // YAML::Node config;
    // std::string model_config;
    // std::string engine_file;

    // 
    triplet mean;
    triplet std;
    bool use_depth;
    bool use_adj;
    int adj_num;
    Sampler pre_sample;
    // data_config
    int N_img;
    int src_img_h;
    int src_img_w;
    int input_img_h;
    int input_img_w;
    int crop_h;
    int crop_w;
    // 
    float resize_radio;
    int feat_h;
    int feat_w;
    // grid_config
    float depth_start;
    float depth_end;
    float depth_step;
    int depth_num;

    float x_start;
    float x_end;
    float x_step;
    int xgrid_num;

    float y_start;
    float y_end;
    float y_step;
    int ygrid_num;

    float z_start;
    float z_end;
    float z_step;
    int zgrid_num;
    // model
    int down_sample;
    int bev_h;
    int bev_w;
    int bevpool_channel;
    // test_cfg
    int class_num;
    float score_thresh;
    float nms_overlap_thresh;
    int nms_pre_maxnum;
    int nms_post_maxnum;
    std::vector<float> nms_rescale_factor;
    std::vector<int> class_num_pre_task;
    std::map<std::string, int> out_num_task_head;

    Eigen::Translation3f post_trans;
    Eigen::Matrix3f post_rot;

    uchar* src_imgs_dev;

    void** imgstage_buffer;
    void** bevstage_buffer;

    std::map<std::string, int> imgbuffer_map;
    std::map<std::string, int> bevbuffer_map;

    int valid_feat_num;
    int unique_bev_num;

    int* ranks_bev_dev;
    int* ranks_depth_dev;
    int* ranks_feat_dev;
    int* interval_starts_dev;
    int* interval_lengths_dev;

    // be private
    // std::vector<Eigen::Matrix3f> cams_intrin;
    // std::vector<Eigen::Quaternion<float>> cams2ego_rot;
    // std::vector<Eigen::Translation3f> cams2ego_trans;


    // 
    Logger g_logger;

    nvinfer1::ICudaEngine* imgstage_engine;
    nvinfer1::ICudaEngine* bevstage_engine;

    nvinfer1::IExecutionContext* imgstage_context;
    nvinfer1::IExecutionContext* bevstage_context;

    std::unique_ptr<PostprocessGPU> postprocess_ptr;
    std::unique_ptr<adjFrame> adj_frame_ptr;
};

__inline__ size_t dataTypeToSize(nvinfer1::DataType dataType);

#endif // __BEVDET_H__
