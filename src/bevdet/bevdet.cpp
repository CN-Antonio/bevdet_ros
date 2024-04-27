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

void BEVDet::InitParams(const std::string &config_file)
{
    YAML::Node model_config = YAML::LoadFile(config_file);
    // mean
    mean.x = model_config["mean"][0].as<float>();
    mean.y = model_config["mean"][1].as<float>();
    mean.z = model_config["mean"][2].as<float>();
    // std
    std.x = model_config["std"][0].as<float>();
    std.y = model_config["std"][1].as<float>();
    std.z = model_config["std"][2].as<float>();
    //
    use_depth = model_config["use_depth"].as<bool>();
    use_adj = model_config["use_adj"].as<bool>();

    if(model_config["sampling"].as<std::string>() == "bicubic"){
        pre_sample = Sampler::bicubic;
    }
    else{
        pre_sample = Sampler::nearest;
    }
    // data_config
    N_img = model_config["data_config"]["Ncams"].as<int>();
    src_img_h = model_config["data_config"]["src_size"][0].as<int>();
    src_img_w = model_config["data_config"]["src_size"][1].as<int>();
    input_img_h = model_config["data_config"]["input_size"][0].as<int>();
    input_img_w = model_config["data_config"]["input_size"][1].as<int>();
    crop_h = model_config["data_config"]["crop"][0].as<int>();
    crop_w = model_config["data_config"]["crop"][1].as<int>();
    // grid_config
    depth_start = model_config["grid_config"]["depth"][0].as<float>();
    depth_end = model_config["grid_config"]["depth"][1].as<float>();
    depth_step = model_config["grid_config"]["depth"][2].as<float>();
    depth_num = (depth_end - depth_start) / depth_step;

    x_start = model_config["grid_config"]["x"][0].as<float>();
    x_end = model_config["grid_config"]["x"][1].as<float>();
    x_step = model_config["grid_config"]["x"][2].as<float>();
    xgrid_num = (x_end - x_start) / x_step;

    y_start = model_config["grid_config"]["y"][0].as<float>();
    y_end = model_config["grid_config"]["y"][1].as<float>();
    y_step = model_config["grid_config"]["y"][2].as<float>();
    ygrid_num = (y_end - y_start) / y_step;

    z_start = model_config["grid_config"]["z"][0].as<float>();
    z_end = model_config["grid_config"]["z"][1].as<float>();
    z_step = model_config["grid_config"]["z"][2].as<float>();
    zgrid_num = (z_end - z_start) / z_step;
    // model
    down_sample = model_config["model"]["down_sample"].as<int>();
    bev_h = ygrid_num;
    bev_w = xgrid_num;
    bevpool_channel = model_config["model"]["bevpool_channels"].as<int>();
    // test_cfg
    nms_pre_maxnum = model_config["test_cfg"]["max_per_img"].as<int>();
    nms_post_maxnum = model_config["test_cfg"]["post_max_size"].as<int>();
    score_thresh = model_config["test_cfg"]["score_threshold"].as<float>();
    nms_overlap_thresh = model_config["test_cfg"]["nms_thr"][0].as<float>();

    std::vector<std::vector<float>> nms_factor_temp = model_config["test_cfg"]
                            ["nms_rescale_factor"].as<std::vector<std::vector<float>>>();
    nms_rescale_factor.clear();
    for(auto task_factors : nms_factor_temp){
        for(float factor : task_factors){
            nms_rescale_factor.push_back(factor);
        }
    }
    
    std::vector<std::vector<std::string>> class_name_pre_task;
    class_num = 0;
    YAML::Node tasks = model_config["model"]["tasks"];
    class_num_pre_task = std::vector<int>();
    for(auto it : tasks){
        int num = it["num_class"].as<int>();
        class_num_pre_task.push_back(num);
        class_num += num;
        class_name_pre_task.push_back(it["class_names"].as<std::vector<std::string>>());
    }

    YAML::Node common_head_channel = model_config["model"]["common_head"]["channels"];
    YAML::Node common_head_name = model_config["model"]["common_head"]["names"];
    for(size_t i = 0; i< common_head_channel.size(); i++){
        out_num_task_head[common_head_name[i].as<std::string>()] = 
                                                        common_head_channel[i].as<int>();
    }

    resize_radio = (float)input_img_w / src_img_w;
    feat_h = input_img_h / down_sample;
    feat_w = input_img_w / down_sample;
    

    post_rot << resize_radio, 0, 0,
                0, resize_radio, 0,
                0, 0, 1;
    post_trans.translation() << -crop_w, -crop_h, 0;

    adj_num = 0;
    if(use_adj){
        adj_num = model_config["adj_num"].as<int>();
        adj_frame_ptr.reset(new adjFrame(adj_num, bev_h * bev_w, bevpool_channel));
    }


    postprocess_ptr.reset(new PostprocessGPU(class_num, score_thresh, nms_overlap_thresh,
                                            nms_pre_maxnum, nms_post_maxnum, down_sample,
                                            bev_h, bev_w, x_step, y_step, x_start,
                                            y_start, class_num_pre_task, nms_rescale_factor));
}

void BEVDet::InitViewTransformer()
{
    int num_points = N_img * depth_num * feat_h * feat_w;
    Eigen::Vector3f* frustum = new Eigen::Vector3f[num_points];

    for(int i = 0; i < N_img; i++){
        for(int d_ = 0; d_ < depth_num; d_++){
            for(int h_ = 0; h_ < feat_h; h_++){
                for(int w_ = 0; w_ < feat_w; w_++){
                    int offset = i * depth_num * feat_h * feat_w + d_ * feat_h * feat_w
                                                                 + h_ * feat_w + w_;
                    (frustum + offset)->x() = (float)w_ * (input_img_w - 1) / (feat_w - 1);
                    (frustum + offset)->y() = (float)h_ * (input_img_h - 1) / (feat_h - 1);
                    (frustum + offset)->z() = (float)d_ * depth_step + depth_start;

                    // eliminate post transformation
                    *(frustum + offset) -= post_trans.translation();
                    *(frustum + offset) = post_rot.inverse() * *(frustum + offset);
                    // 
                    (frustum + offset)->x() *= (frustum + offset)->z();
                    (frustum + offset)->y() *= (frustum + offset)->z();
                    // img to ego -> rot -> trans
                    *(frustum + offset) = cams2ego_rot[i] * cams_intrin[i].inverse()
                                    * *(frustum + offset) + cams2ego_trans[i].translation();

                    // voxelization
                    *(frustum + offset) -= Eigen::Vector3f(x_start, y_start, z_start);
                    (frustum + offset)->x() = (int)((frustum + offset)->x() / x_step);
                    (frustum + offset)->y() = (int)((frustum + offset)->y() / y_step);
                    (frustum + offset)->z() = (int)((frustum + offset)->z() / z_step);
                }
            }
        }
    }

    int* _ranks_depth = new int[num_points];
    int* _ranks_feat = new int[num_points];

    for(int i = 0; i < num_points; i++){
        _ranks_depth[i] = i;
    }
    for(int i = 0; i < N_img; i++){
        for(int d_ = 0; d_ < depth_num; d_++){
            for(int u = 0; u < feat_h * feat_w; u++){
                int offset = i * (depth_num * feat_h * feat_w) + d_ * (feat_h * feat_w) + u;
                _ranks_feat[offset] = i * feat_h * feat_w + u;
            }
        }
    }

    std::vector<int> kept;
    for(int i = 0; i < num_points; i++){
        if((int)(frustum + i)->x() >= 0 && (int)(frustum + i)->x() < xgrid_num &&
           (int)(frustum + i)->y() >= 0 && (int)(frustum + i)->y() < ygrid_num &&
           (int)(frustum + i)->z() >= 0 && (int)(frustum + i)->z() < zgrid_num){
            kept.push_back(i);
        }
    }

    valid_feat_num = kept.size();
    int* ranks_depth_host = new int[valid_feat_num];
    int* ranks_feat_host = new int[valid_feat_num];
    int* ranks_bev_host = new int[valid_feat_num];
    int* order = new int[valid_feat_num];

    for(int i = 0; i < valid_feat_num; i++){
        Eigen::Vector3f &p = frustum[kept[i]];
        ranks_bev_host[i] = (int)p.z() * xgrid_num * ygrid_num + 
                            (int)p.y() * xgrid_num + (int)p.x();
        order[i] = i;
    }

    thrust::sort_by_key(ranks_bev_host, ranks_bev_host + valid_feat_num, order);
    for(int i = 0; i < valid_feat_num; i++){
        ranks_depth_host[i] = _ranks_depth[kept[order[i]]];
        ranks_feat_host[i] = _ranks_feat[kept[order[i]]];
    }

    delete[] _ranks_depth;
    delete[] _ranks_feat;
    delete[] frustum;
    delete[] order;

    std::vector<int> interval_starts_host;
    std::vector<int> interval_lengths_host;

    interval_starts_host.push_back(0);
    int len = 1;
    for(int i = 1; i < valid_feat_num; i++){
        if(ranks_bev_host[i] != ranks_bev_host[i - 1]){
            interval_starts_host.push_back(i);
            interval_lengths_host.push_back(len);
            len=1;
        }
        else{
            len++;
        }
    }
    
    interval_lengths_host.push_back(len);
    unique_bev_num = interval_lengths_host.size();

    CHECK_CUDA(cudaMalloc((void**)&ranks_bev_dev, valid_feat_num * sizeof(int)));
    CHECK_CUDA(cudaMalloc((void**)&ranks_depth_dev, valid_feat_num * sizeof(int)));
    CHECK_CUDA(cudaMalloc((void**)&ranks_feat_dev, valid_feat_num * sizeof(int)));
    CHECK_CUDA(cudaMalloc((void**)&interval_starts_dev, unique_bev_num * sizeof(int)));
    CHECK_CUDA(cudaMalloc((void**)&interval_lengths_dev, unique_bev_num * sizeof(int)));

    CHECK_CUDA(cudaMemcpy(ranks_bev_dev, ranks_bev_host, valid_feat_num * sizeof(int), 
                                                                    cudaMemcpyHostToDevice));
    CHECK_CUDA(cudaMemcpy(ranks_depth_dev, ranks_depth_host, valid_feat_num * sizeof(int), 
                                                                    cudaMemcpyHostToDevice));
    CHECK_CUDA(cudaMemcpy(ranks_feat_dev, ranks_feat_host, valid_feat_num * sizeof(int), 
                                                                    cudaMemcpyHostToDevice));
    CHECK_CUDA(cudaMemcpy(interval_starts_dev, interval_starts_host.data(), 
                                        unique_bev_num * sizeof(int), cudaMemcpyHostToDevice));
    CHECK_CUDA(cudaMemcpy(interval_lengths_dev, interval_lengths_host.data(), 
                                        unique_bev_num * sizeof(int), cudaMemcpyHostToDevice));

    // printf("Num_points : %d\n", num_points);
    // printf("valid_feat_num : %d\n", valid_feat_num);
    // printf("unique_bev_num : %d\n", unique_bev_num);
    // printf("valid rate : %.3lf\n", (float)valid_feat_num / num_points);

    delete[] ranks_bev_host;
    delete[] ranks_depth_host;
    delete[] ranks_feat_host;
}

int BEVDet::DeserializeTRTEngine(const std::string &engine_file, nvinfer1::ICudaEngine **engine_ptr)
{
    int verbosity = static_cast<int>(nvinfer1::ILogger::Severity::kWARNING);
    std::stringstream engine_stream;
    engine_stream.seekg(0, engine_stream.beg);

    std::ifstream file(engine_file);
    engine_stream << file.rdbuf();
    file.close();

    nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(g_logger);
    if (runtime == nullptr) 
    {
        // std::string msg("Failed to build runtime parser!");
        // g_logger.log(nvinfer1::ILogger::Severity::kERROR, msg.c_str());
        // std::cout << "" << "Failed to build runtime parser!" << std::endl;
        std::cout << "\033[1;31m" << "\nFailed to build runtime parser!\n" << "\033[0m" << std::endl;
        return EXIT_FAILURE;
    }
    engine_stream.seekg(0, std::ios::end);
    const int engine_size = engine_stream.tellg();

    engine_stream.seekg(0, std::ios::beg); 
    void* engine_str = malloc(engine_size);
    engine_stream.read((char*)engine_str, engine_size);
    
    nvinfer1::ICudaEngine *engine = runtime->deserializeCudaEngine(engine_str, engine_size, NULL);
    if (engine == nullptr) 
    {
        // std::string msg("Failed to build engine parser!");
        // g_logger.log(nvinfer1::ILogger::Severity::kERROR, msg.c_str());

        std::cout << "\033[1;31m" << "\nFailed to build engine parser!\n" << "\033[0m" << std::endl;

        return EXIT_FAILURE;
    }
    *engine_ptr = engine;
    for (int bi = 0; bi < engine->getNbBindings(); bi++) {
        if (engine->bindingIsInput(bi) == true){
            printf("Binding %d (%s): Input. \n", bi, engine->getBindingName(bi));
        }
        else{
            printf("Binding %d (%s): Output. \n", bi, engine->getBindingName(bi));
        }
    }
    return EXIT_SUCCESS;
}

void print_dim(nvinfer1::Dims dim){
    for(auto i = 0; i < dim.nbDims; i++){
        printf("%d%c", dim.d[i], i == dim.nbDims - 1 ? '\n' : ' ');
    }
}
int BEVDet::InitEngine(const std::string &imgstage_file, const std::string &bevstage_file)
{
    if(DeserializeTRTEngine(imgstage_file, &imgstage_engine))
    {
        return EXIT_FAILURE;
    }
    if(DeserializeTRTEngine(bevstage_file, &bevstage_engine)){
        return EXIT_FAILURE;
    }
    imgstage_context = imgstage_engine->createExecutionContext();
    bevstage_context = bevstage_engine->createExecutionContext();

    if (imgstage_context == nullptr || bevstage_context == nullptr) {
        std::cerr << "Failed to create TensorRT Execution Context!" << std::endl;
        return EXIT_FAILURE;
    }

    // set bindings
    imgstage_context->setBindingDimensions(0, 
                            nvinfer1::Dims32{4, {N_img, 3, input_img_h, input_img_w}});
    bevstage_context->setBindingDimensions(0,
            nvinfer1::Dims32{4, {1, bevpool_channel * (adj_num + 1), bev_h, bev_w}});
    imgbuffer_map.clear();
    for(auto i = 0; i < imgstage_engine->getNbBindings(); i++){
        auto dim = imgstage_context->getBindingDimensions(i);
        auto name = imgstage_engine->getBindingName(i);
        imgbuffer_map[name] = i;
        std::cout << name << " : ";
        print_dim(dim);

    }
    std::cout << std::endl;

    bevbuffer_map.clear();
    for(auto i = 0; i < bevstage_engine->getNbBindings(); i++){
        auto dim = bevstage_context->getBindingDimensions(i);
        auto name = bevstage_engine->getBindingName(i);
        bevbuffer_map[name] = i;
        std::cout << name << " : ";
        print_dim(dim);
    }    
    
    return EXIT_SUCCESS;
}

void BEVDet::MallocDeviceMemory()
{
    CHECK_CUDA(cudaMalloc((void**)&src_imgs_dev, 
                                N_img * 3 * src_img_h * src_img_w * sizeof(uchar)));

    imgstage_buffer = (void**)new void*[imgstage_engine->getNbBindings()];
    for(int i = 0; i < imgstage_engine->getNbBindings(); i++){
        nvinfer1::Dims32 dim = imgstage_context->getBindingDimensions(i);
        int size = 1;
        for(int j = 0; j < dim.nbDims; j++){
            size *= dim.d[j];
        }
        size *= dataTypeToSize(imgstage_engine->getBindingDataType(i));
        CHECK_CUDA(cudaMalloc(&imgstage_buffer[i], size));
    }

    std::cout << "img num binding : " << imgstage_engine->getNbBindings() << std::endl;

    bevstage_buffer = (void**)new void*[bevstage_engine->getNbBindings()];
    for(int i = 0; i < bevstage_engine->getNbBindings(); i++){
        nvinfer1::Dims32 dim = bevstage_context->getBindingDimensions(i);
        int size = 1;
        for(int j = 0; j < dim.nbDims; j++){
            size *= dim.d[j];
        }
        size *= dataTypeToSize(bevstage_engine->getBindingDataType(i));
        CHECK_CUDA(cudaMalloc(&bevstage_buffer[i], size));
    }
}

/* Infer */
void BEVDet::InitDepth(const std::vector<Eigen::Quaternion<float>> &curr_cams2ego_rot,
                       const std::vector<Eigen::Translation3f> &curr_cams2ego_trans,
                       const std::vector<Eigen::Matrix3f> &cur_cams_intrin)
{}

int BEVDet::DoInfer()
{
    std::cout<< "Do Infer" <<std::endl;
    return 0;
}

int BEVDet::DoInfer(const camsData &cam_data,  
                    std::vector<Box> &out_detections, 
                    float &cost_time,
                    int idx)
{
    std::cout<< "Do Infer" <<std::endl;
    /*  [STEP 1] 
     *  preprocess image, including resize, crop and normalize 
    */

    /* [STEP 2] : image stage network forward */
    // [STEP 3] : bev pool
    // [STEP 4] : align BEV feature
    // [STEP 5] : BEV stage network forward
    // [STEP 6] : postprocess
    return 0;
}

BEVDet::~BEVDet()
{
    imgstage_context->destroy();
    bevstage_context->destroy();

    imgstage_engine->destroy();
    bevstage_engine->destroy();
}

__inline__ size_t dataTypeToSize(nvinfer1::DataType dataType)
{
    switch ((int)dataType)
    {
    case int(nvinfer1::DataType::kFLOAT):
        return 4;
    case int(nvinfer1::DataType::kHALF):
        return 2;
    case int(nvinfer1::DataType::kINT8):
        return 1;
    case int(nvinfer1::DataType::kINT32):
        return 4;
    case int(nvinfer1::DataType::kBOOL):
        return 1;
    default:
        return 4;
    }
}
