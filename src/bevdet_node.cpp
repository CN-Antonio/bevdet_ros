#include "bevdet_node.h"
#include "test.h"

namespace bevdet::bevdet_ros
{

std::vector<std::vector<float>> colormap = {
    {1.0 , 0.62, 0.0, 0.5},     // 0-car
    {1.0 , 0.38, 0.27, 0.5},    // 1-truck
    {1.0 , 0.38, 0.27, 0.5},    // 2-construction_vehicle
    {0.0, 0.0, 0.0, 0.5},
    {0.0, 0.0, 0.0, 0.5},
    {0.0, 0.0, 0.0, 0.5},
    {0.0, 0.0, 0.0, 0.5},
    {0.86, 0.08, 0.23, 0.5},    // 7-bicycle
    {0.0, 0.0, 0.90, 0.5},      // 8-pedestrian
    {0.0, 0.0, 0.0, 0.5},
};

std_msgs::msg::ColorRGBA make_color(int label)
{
    std_msgs::msg::ColorRGBA c;

    c.r = colormap[label][0];
    c.g = colormap[label][1];
    c.b = colormap[label][2];
    c.a = colormap[label][3];

    return c;
}

// std::vector<std_msgs::msg::ColorRGBA> box_color = {
//     {1.0, 0.38, 0.27, 0.5}
// };

// TODO: insert into BEVDet_Node
void publish_boxes(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher, std::vector<Box> boxes)
{
    std::cout<< "len: "<< boxes.size()<< std::endl;

    visualization_msgs::msg::MarkerArray marker_array_del;
    visualization_msgs::msg::Marker box_marker;
    visualization_msgs::msg::MarkerArray box_array;

    // Set new
    box_marker.header.frame_id = "base_link";
    box_marker.type = visualization_msgs::msg::Marker::CUBE;  // 1
    box_marker.action = visualization_msgs::msg::Marker::ADD; // 0
    box_marker.scale.x = 2.0;
    box_marker.color.a = 0.5;
    for(int i=0; i<boxes.size(); i++)
    {
        if(boxes[i].score < 0.2)
            continue;

        box_marker.header.stamp = rclcpp::Clock().now();
        box_marker.id = i;

        box_marker.pose.position.x = boxes[i].x;
        box_marker.pose.position.y = boxes[i].y;
        box_marker.pose.position.z = boxes[i].z + boxes[i].h*0.5;

        double cy = cos(boxes[i].r * 0.5);
        double sy = sin(boxes[i].r * 0.5);
        double cp = 1; // cos(pitch * 0.5);
        double sp = 0; // sin(pitch * 0.5);
        double cr = 1; // cos(roll * 0.5);
        double sr = 0; // sin(roll * 0.5);

        box_marker.pose.orientation.x = cy * cp * sr - sy * sp * cr;
        box_marker.pose.orientation.y = sy * cp * sr + cy * sp * cr;
        box_marker.pose.orientation.z = sy * cp * cr - cy * sp * sr;
        box_marker.pose.orientation.w = cy * cp * cr + sy * sp * sr;

        box_marker.scale.x = boxes[i].l;
        box_marker.scale.y = boxes[i].w;
        box_marker.scale.z = boxes[i].h;

        box_marker.color = make_color(boxes[i].label);
        box_array.markers.push_back(box_marker);
    }
    publisher->publish(box_array);
    box_array.markers.clear();
}

BEVDet_Node::BEVDet_Node(const rclcpp::NodeOptions & node_options):
    rclcpp::Node("bevdet_node", node_options),
    base_frame_(this->declare_parameter<std::string>("base_frame")),
    BEVDet()
    // sub_cloud_top_(this, "/LIDAR_TOP",  rclcpp::QoS{1}.get_rmw_qos_profile())
{
    /* Set ROS parameters */
    // base_frame_ = this->declare_parameter<std::string>("base_frame");

    // ==================  Set subscribers and publishers ========================= //
    sub_pose_ = create_subscription<nav_msgs::msg::Odometry>(
        "~/input/odom", rclcpp::QoS{1},
        std::bind(&BEVDet_Node::callback_odom, this, std::placeholders::_1));
    // sub_cloud_top_.subscribe(this, "/LIDAR_TOP",  rclcpp::QoS{1}.get_rmw_qos_profile());
    sub_img_fl_.subscribe(this, "~/input/image_fl", rclcpp::QoS{1}.get_rmw_qos_profile());
    sub_img_f_.subscribe(this, "~/input/image_f", rclcpp::QoS{1}.get_rmw_qos_profile());
    sub_img_fr_.subscribe(this, "~/input/image_fr", rclcpp::QoS{1}.get_rmw_qos_profile());
    sub_img_bl_.subscribe(this, "~/input/image_bl", rclcpp::QoS{1}.get_rmw_qos_profile());
    sub_img_b_.subscribe(this, "~/input/image_b", rclcpp::QoS{1}.get_rmw_qos_profile());
    sub_img_br_.subscribe(this, "~/input/image_br", rclcpp::QoS{1}.get_rmw_qos_profile());
// CompressedImage
    // sub_cpimg_fl_.subscribe(this, "~/input/image_fl", rclcpp::QoS{1}.get_rmw_qos_profile());
    // sub_cpimg_f_.subscribe(this, "~/input/image_f", rclcpp::QoS{1}.get_rmw_qos_profile());
    // sub_cpimg_fr_.subscribe(this, "~/input/image_fr", rclcpp::QoS{1}.get_rmw_qos_profile());
    // sub_cpimg_bl_.subscribe(this, "~/input/image_bl", rclcpp::QoS{1}.get_rmw_qos_profile());
    // sub_cpimg_b_.subscribe(this, "~/input/image_b", rclcpp::QoS{1}.get_rmw_qos_profile());
    // sub_cpimg_br_.subscribe(this, "~/input/image_br", rclcpp::QoS{1}.get_rmw_qos_profile());

    sync_queue_size_ = declare_parameter<int>("sync_queue_size", 60);   // 10 for each img
    // Create publishers and subscribers
    using std::placeholders::_1;using std::placeholders::_2;
    using std::placeholders::_3;using std::placeholders::_4;
    using std::placeholders::_5;using std::placeholders::_6;
    // using std::placeholders::_7;
// IF RGB Img
    sync_ptr_ = std::make_shared<Sync>(SyncPolicy(sync_queue_size_),
        // sub_cloud_top_,
        sub_img_fl_, sub_img_f_, sub_img_fr_,
        sub_img_bl_, sub_img_b_, sub_img_br_);
    sync_ptr_->registerCallback(
        std::bind(&BEVDet_Node::callback, this, _1, _2, _3, _4, _5, _6));
// IF Compressed Img
    // sync_cp_ptr_ = std::make_shared<SyncCp>(SyncCpPolicy(sync_queue_size_),
    //     sub_cpimg_fl_, sub_cpimg_f_, sub_cpimg_fr_,
    //     sub_cpimg_bl_, sub_cpimg_b_, sub_cpimg_br_);
    // sync_cp_ptr_->registerCallback(
    //     std::bind(&BEVDet_Node::callbackCompressed, this, _1, _2, _3, _4, _5, _6));

    pub_stitched_img = create_publisher<sensor_msgs::msg::Image>(
        "/output/img_stitched", rclcpp::QoS{1});
    pub_cloud_top = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/LIDAR_TOP_retime", rclcpp::QoS{1});
    markers = create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/output/markers", rclcpp::QoS{1});

    /* ================ set Data params ================ */
    ROSInitParams();    // replace InitParams(config_file);


    /* TODO: init TRT engine be applied in BEVDet::InitBEVDet()*/
    // 初始化视角转换
    auto start = std::chrono::high_resolution_clock::now();
    InitViewTransformer();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> t = end - start;
    RCLCPP_INFO(this->get_logger(), "InitVewTransformer cost time : %.4lf ms\n", t.count() * 1000);

    InitEngine(imgstage_file, bevstage_file); // FIXME
    MallocDeviceMemory();
    // 以上为bevdet原构造函数内容

    // gpu分配内参， cuda上分配6张图的大小 每个变量sizeof(uchar)个字节，并用imgs_dev指向该gpu上内存, sizeof(uchar) =1
    CHECK_CUDA(cudaMalloc((void**)&imgs_dev_, N_img * 3 * src_img_w * src_img_h * sizeof(uchar)));

    // test
    // TestSample();
    // timer_ = this->create_wall_timer(
    //         500ms, std::bind(&BEVDet_Node::timer_callback, this)
    // );
}

BEVDet_Node::~BEVDet_Node(){
    RCLCPP_INFO(rclcpp::get_logger("bevdet_node"), "Destructing BEVDet_Node");
}

void BEVDet_Node::ROSInitParams(void)
{
    /* Dataset */
    std::string ns = "";
    ns = "cams.";
    const auto cams_name = declare_parameter<std::vector<std::string>>(ns + "cam_name");

    cams_intrin.clear();
    cams2ego_rot.clear();
    cams2ego_trans.clear();

    cams_intrin.resize(cams_name.size());
    cams2ego_rot.resize(cams_name.size());
    cams2ego_trans.resize(cams_name.size());

    // TODO: check if params are correct
    for(size_t i = 0; i < cams_name.size(); i++){
        ns = "cams." + cams_name[i] + ".";
        cams_intrin[i] = fromVectorMatrix3f(this->declare_parameter<std::vector<double>>(ns + "cam_intrinsic"));
        cams2ego_rot[i] = fromVectorQuater(this->declare_parameter<std::vector<double>>(ns + "sensor2ego_rotation"));
        cams2ego_trans[i] = fromVectorTrans(this->declare_parameter<std::vector<double>>(ns + "sensor2ego_translation"));
    }

    /* engine */
    imgstage_file = declare_parameter<std::string>("ImgStageEngine");
    bevstage_file = declare_parameter<std::string>("BEVStageEngine");

    /* Model */
    N_img = declare_parameter<int>("N");
    src_img_h = declare_parameter<int>("H");
    src_img_w = declare_parameter<int>("W");
    auto input_img = declare_parameter<std::vector<int>>("data_config.input_size");
    input_img_h = input_img[0];
    input_img_w = input_img[1];
    auto crop = declare_parameter<std::vector<int>>("data_config.crop");
    crop_h = crop[0];
    crop_w = crop[1];

    // normalize
    auto mean_ = declare_parameter<std::vector<float>>("mean");
    mean.x = mean_[0];
    mean.y = mean_[1];
    mean.z = mean_[2];
    auto std_ = declare_parameter<std::vector<float>>("std");
    std.x = std_[0];
    std.y = std_[1];
    std.z = std_[2];
    
    auto grid_config_depth = declare_parameter<std::vector<float>>("grid_config.depth");
    depth_start = grid_config_depth[0];
    depth_end =   grid_config_depth[1];
    depth_step =  grid_config_depth[2];
    auto grid_config_x = declare_parameter<std::vector<float>>("grid_config.x");
    x_start = grid_config_x[0];
    x_end =   grid_config_x[1];
    x_step =  grid_config_x[2];
    auto grid_config_y = declare_parameter<std::vector<float>>("grid_config.y");
    y_start = grid_config_y[0];
    y_end =   grid_config_y[1];
    y_step =  grid_config_y[2];
    auto grid_config_z = declare_parameter<std::vector<float>>("grid_config.z");
    z_start = grid_config_z[0];
    z_end =   grid_config_z[1];
    z_step =  grid_config_z[2];

    down_sample = declare_parameter<int>("model.down_sample");
    bevpool_channel = declare_parameter<int>("model.bevpool_channels");

    nms_pre_maxnum = declare_parameter<int>("test_cfg.max_per_img");
    nms_post_maxnum = declare_parameter<int>("test_cfg.post_max_size");
    score_thresh = declare_parameter<float>("test_cfg.score_threshold");
    nms_overlap_thresh = declare_parameter<std::vector<float>>("test_cfg.nms_thr")[0];  // ?

    use_depth = declare_parameter<bool>("use_depth");
    use_adj = declare_parameter<bool>("use_adj");
    if(declare_parameter<std::string>("sampling") == "bicubic"){
        pre_sample = Sampler::bicubic;
    } else {
        pre_sample = Sampler::nearest;
    }

    auto nms_factor_temp = declare_parameter<std::vector<float>>(
        "test_cfg.nms_rescale_factor");
    nms_rescale_factor.clear();
    for(const auto& factor : nms_factor_temp) {
        nms_rescale_factor.push_back(static_cast<float>(factor));
    }
    // for(auto task_factors : nms_factor_temp){
    //     for(float factor : task_factors){
    //         nms_rescale_factor.push_back(factor);
    //     }
    // }

    // for post process
    std::vector<std::vector<std::string>> class_name_pre_task; //unknow usage
    class_num = 0;
    class_num_pre_task = std::vector<int>();
    int task_count = declare_parameter<int>("model.common_head.tasks.count", 1);
    for (int i = 0; i < task_count; i++) {
        std::string task_prefix = "model.tasks." + std::to_string(i) + ".";
        
        auto task_classes = declare_parameter<std::vector<std::string>>(task_prefix + "class_names", std::vector<std::string>{});
        int num = declare_parameter<int>(task_prefix + "num_class", 0);
        class_num_pre_task.push_back(num);
        class_num += num;
        class_name_pre_task.push_back(task_classes);
    }

    auto common_head_channel = declare_parameter<std::vector<int>>("model.common_head.channels");
    auto common_head_name = declare_parameter<std::vector<std::string>>("model.common_head.names");
    for(size_t i = 0; i< common_head_channel.size(); i++){
        out_num_task_head[common_head_name[i]] = common_head_channel[i];
    }

    // DONE: Calculate some params
    resize_radio = (float)input_img_w / src_img_w;
    feat_h = input_img_h / down_sample;
    feat_w = input_img_w / down_sample;
    depth_num = (depth_end - depth_start) / depth_step;
    xgrid_num = (x_end - x_start) / x_step;
    ygrid_num = (y_end - y_start) / y_step;
    zgrid_num = (z_end - z_start) / z_step;
    bev_h = ygrid_num;
    bev_w = xgrid_num;


    post_rot << resize_radio, 0, 0,
                0, resize_radio, 0,
                0, 0, 1;
    post_trans.translation() << -crop_w, -crop_h, 0;

    adj_num = 0;
    if(use_adj){
        adj_num = declare_parameter<int>("adj_num");
        adj_frame_ptr.reset(new adjFrame(adj_num, bev_h * bev_w, bevpool_channel));
    }


    postprocess_ptr.reset(new PostprocessGPU(class_num, score_thresh, nms_overlap_thresh,
                                            nms_pre_maxnum, nms_post_maxnum, down_sample,
                                            bev_h, bev_w, x_step, y_step, x_start, y_start,
                                            class_num_pre_task, nms_rescale_factor));

    std::cout<< "Init Params Finished" <<std::endl;

}
/* pose */
void BEVDet_Node::callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "new odom callback");

    // Extract position and update ego2global_trans
    ego2global_trans.translation() = Eigen::Vector3f(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z
    );

    // Extract rotation quaternion and update ego2global_rot
    ego2global_rot = Eigen::Quaternionf(
        msg->pose.pose.orientation.w,  // w comes first in Eigen
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    );

    RCLCPP_DEBUG(this->get_logger(), 
        "Updated ego pose - Position: [%.2f, %.2f, %.2f], Orientation: [%.2f, %.2f, %.2f, %.2f]",
        ego2global_trans.x(), ego2global_trans.y(), ego2global_trans.z(),
        ego2global_rot.w(), ego2global_rot.x(), ego2global_rot.y(), ego2global_rot.z()
    );
}

/* Image RGB */
void BEVDet_Node::callback(
    // const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_cloud,
    const sensor_msgs::msg::Image::ConstSharedPtr & img_fl_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & img_f_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & img_fr_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & img_bl_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & img_b_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & img_br_msg)
{
    RCLCPP_INFO(this->get_logger(), "new Img callback");

    // TODO: remove opencv
    std::vector<cv::Mat> imgs;
    imgs.reserve(6); // Pre-allocate space for 6 images

    // Convert ROS messages to OpenCV images
    try {
        imgs.emplace_back(cv_bridge::toCvShare(img_fl_msg, "bgr8")->image);
        imgs.emplace_back(cv_bridge::toCvShare(img_f_msg, "bgr8")->image);
        imgs.emplace_back(cv_bridge::toCvShare(img_fr_msg, "bgr8")->image);
        imgs.emplace_back(cv_bridge::toCvShare(img_bl_msg, "bgr8")->image);
        imgs.emplace_back(cv_bridge::toCvShare(img_b_msg, "bgr8")->image);
        imgs.emplace_back(cv_bridge::toCvShare(img_br_msg, "bgr8")->image);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Verify image dimensions
    for (const auto& img : imgs) {
        if (img.empty() || img.cols != src_img_w || img.rows != src_img_h) {
            RCLCPP_ERROR(this->get_logger(), "Invalid image dimensions");
            return;
        }
    }

    cv::Mat img_stitched;

    // Create temporary matrices for the two rows
    cv::Mat row1, row2;
    
    // Concatenate images horizontally for each row (3 images per row)
    cv::hconcat(std::vector<cv::Mat>{imgs[0], imgs[1], imgs[2]}, row1);
    cv::hconcat(std::vector<cv::Mat>{imgs[3], imgs[4], imgs[5]}, row2);
    
    // Concatenate the two rows vertically
    cv::vconcat(std::vector<cv::Mat>{row1, row2}, img_stitched);
    
    // Resize the stitched image to 50% of original size
    cv::resize(img_stitched, img_stitched, cv::Size(), 0.5, 0.5, cv::INTER_AREA);


    // Process images on GPU
    uchar* temp_gpu = nullptr;
    try {
        CHECK_CUDA(cudaMalloc(&temp_gpu, src_img_w * src_img_h * 3));
        
        for (size_t i = 0; i < imgs.size(); i++) {
            if (!imgs[i].isContinuous()) {
                imgs[i] = imgs[i].clone();
            }
            CHECK_CUDA(cudaMemcpy(temp_gpu, imgs[i].data, src_img_w * src_img_h * 3, cudaMemcpyHostToDevice));
            convert_RGBHWC_to_BGRCHW(temp_gpu, imgs_dev_ + i * src_img_w * src_img_h * 3, 3, src_img_h, src_img_w);
            CHECK_CUDA(cudaDeviceSynchronize());
        }
    } catch (const std::runtime_error& e) {
        RCLCPP_ERROR(this->get_logger(), "CUDA error: %s", e.what());
        if (temp_gpu) {
            cudaFree(temp_gpu);
        }
        return;
    }

    // Clean up temporary GPU memory
    if (temp_gpu) {
        CHECK_CUDA(cudaFree(temp_gpu));
    }

    // Process the images
    try {
        DoInfer(imgs_dev_);

        // Create ROS2 messages
        sensor_msgs::msg::Image pub_msg;
        std_msgs::msg::Header header;
        cv_bridge::CvImage cv_bridge;

        // Set header timestamp
        header.stamp = this->get_clock()->now();

        // Convert OpenCV image to ROS message
        cv_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img_stitched);
        cv_bridge.toImageMsg(pub_msg);

        // Publish the message
        pub_stitched_img->publish(pub_msg);

        // publish
        publish_boxes(this->markers, ego_boxes);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Inference error: %s", e.what());
        return;
    }

}

void BEVDet_Node::callbackCompressed(
    // const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_cloud,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_fl_msg,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_f_msg,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_fr_msg,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_bl_msg,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_b_msg,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_br_msg)
{
    RCLCPP_INFO(rclcpp::get_logger("bevdet_node"), "new CompressedImg callback");
    return;

    // TODO: replace opencv
    cv::Mat img_fl, img_f, img_fr, img_bl, img_b, img_br;
    std::vector<cv::Mat> imgs;
    // CompressedImg need toCvCopy!!!
    img_fl = cv_bridge::toCvCopy(img_fl_msg , "bgr8")->image;
    img_f  = cv_bridge::toCvCopy(img_f_msg, "bgr8")->image;
    img_fr = cv_bridge::toCvCopy(img_fr_msg, "bgr8")->image;
    img_bl = cv_bridge::toCvCopy(img_bl_msg , "bgr8")->image;
    img_b  = cv_bridge::toCvCopy(img_b_msg, "bgr8")->image;
    img_br = cv_bridge::toCvCopy(img_br_msg, "bgr8")->image;

    imgs.emplace_back(img_fl);
    imgs.emplace_back(img_f);
    imgs.emplace_back(img_fr);
    imgs.emplace_back(img_bl);
    imgs.emplace_back(img_b);
    imgs.emplace_back(img_br);

    // TODO: optimize decode jpeg imgs
    // std::vector<std::vector<char>> imgs_data;
    // cvImgToArr(imgs, imgs_data);
    // decode_cpu(imgs_data, imgs_dev_, img_w_, img_h_);
    // optimized
    size_t width = 1600;
    size_t height = 900;
    uchar* temp_gpu = nullptr;
    CHECK_CUDA(cudaMalloc(&temp_gpu, width * height * 3));
    for (size_t i = 0; i < imgs.size(); i++) {
        CHECK_CUDA(cudaMemcpy(temp_gpu, imgs[i].data, width * height * 3, cudaMemcpyHostToDevice));
        convert_RGBHWC_to_BGRCHW(temp_gpu, imgs_dev_ + i * width * height * 3, 3, height, width);
        CHECK_CUDA(cudaDeviceSynchronize());
    }
    CHECK_CUDA(cudaFree(temp_gpu));

    // vconcat 6 imgs
    // TODO: cuda/nvjpeg process?
    // origin(now): image/raw -> jpeg -> jpeg concat buffer -> gpu buffer
    // optimism:    image/raw -> raw concat buffer -> gpu buffer
    // finally:     image/raw -(nv)-> gpu buffer

    // img_stitched: from published img to see what pic's form
    cv::Mat img_stitched = cv::Mat::zeros(cv::Size(1600, 900 * 6), CV_8UC3);
    CHECK_CUDA(cudaMemcpy(img_stitched.data, imgs_dev_,
        N_img * src_img_h * src_img_w * 3 * sizeof(uchar), cudaMemcpyDeviceToHost));

    DoInfer(imgs_dev_);

    //create ROS2 messages
    sensor_msgs::msg::Image _img_msg;
    std_msgs::msg::Header _header;
    cv_bridge::CvImage _cv_bridge;
    _header.stamp = this->get_clock() -> now();
    _cv_bridge = cv_bridge::CvImage(_header, sensor_msgs::image_encodings::BGR8, img_f);
    _cv_bridge.toImageMsg(_img_msg);

    //publish
    pub_stitched_img -> publish(_img_msg);      // for debug
    publish_boxes(this->markers, ego_boxes);
}

void BEVDet_Node::TestNuscenes(YAML::Node &config){
    std::cout<< "TestNuscenes" <<std::endl;
    size_t img_N = config["N"].as<size_t>();
    int img_w = config["W"].as<int>();
    int img_h = config["H"].as<int>();
    std::string data_info_path = config["dataset_info"].as<std::string>();
    std::string model_config = config["ModelConfig"].as<std::string>();
    std::string output_dir = config["OutputDir"].as<std::string>();
    std::vector<std::string> cams_name = config["cams"]["cam_name"].as<std::vector<std::string>>();

    DataLoader nuscenes(img_N, img_h, img_w, data_info_path, cams_name);
    BEVDet bevdet(model_config, img_N,
            cams_intrin, cams2ego_rot, cams2ego_trans,
            imgstage_file, bevstage_file);
    double sum_time = 0;
    int  cnt = 0;
    for(int i = 0; i < nuscenes.size(); i++){
    // for(int i = 0; i < 20; i++){
        ego_boxes.clear();
        float time = 0.f;
        bevdet.DoInfer(nuscenes.data(i), ego_boxes, time, i);
        // publish_boxes(this->markers, ego_boxes);

        if(i != 0){
            sum_time += time;
            cnt++;
        }
        // Boxes2Txt(ego_boxes, output_dir + "/bevdet_egoboxes_" + std::to_string(i) + ".txt", true);
        // this->publish_imgs(nuscenes.imgs_data);
        // publish_boxes(this->markers, ego_boxes);
    }
    printf("Infer mean cost time : %.5lf ms\n", sum_time / cnt);
}
void BEVDet_Node::TestSample(){
    std::cout<< "TestSample" <<std::endl;
    std::vector<cv::Mat> imgs;
    std::vector<std::string> img_files = {
        "CAM_FRONT_LEFT.jpg",
        "CAM_FRONT.jpg",
        "CAM_FRONT_RIGHT.jpg",
        "CAM_BACK_LEFT.jpg",
        "CAM_BACK.jpg",
        "CAM_BACK_RIGHT.jpg"
    };
    for(int i = 0; i < 6; i++){
        cv::Mat img = cv::imread("/media/antonio/data0/nuscenes/imgs/" + img_files[i]);
        imgs.push_back(img);
    }

    // Process images on GPU
    uchar* temp_gpu = nullptr;
    try {
        CHECK_CUDA(cudaMalloc(&temp_gpu, src_img_w * src_img_h * 3));
        
        for (size_t i = 0; i < imgs.size(); i++) {
            if (!imgs[i].isContinuous()) {
                imgs[i] = imgs[i].clone();
            }
            CHECK_CUDA(cudaMemcpy(temp_gpu, imgs[i].data, src_img_w * src_img_h * 3, cudaMemcpyHostToDevice));
            convert_RGBHWC_to_BGRCHW(temp_gpu, imgs_dev_ + i * src_img_w * src_img_h * 3, 3, src_img_h, src_img_w);
            CHECK_CUDA(cudaDeviceSynchronize());
        }
    } catch (const std::runtime_error& e) {
        RCLCPP_ERROR(this->get_logger(), "CUDA error: %s", e.what());
        if (temp_gpu) {
            cudaFree(temp_gpu);
        }
        return;
    }

    // Clean up temporary GPU memory
    if (temp_gpu) {
        CHECK_CUDA(cudaFree(temp_gpu));
    }

    // Process the images
    try {
        DoInfer(imgs_dev_);

        // publish
        publish_boxes(this->markers, ego_boxes);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Inference error: %s", e.what());
        return;
    }
}

}  // namespace bevdet::bevdet_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(bevdet::bevdet_ros::BEVDet_Node)