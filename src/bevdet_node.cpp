#include "bevdet_node.h"
#include "test.h"

int main(int argc, char **argv)
{
#ifdef ROS_FOUND
#elif ROS2_FOUND
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
#endif

    /* main function BEGIN */
    // std::shared_ptr<ROS_Node> demo_ptr = std::make_shared<ROS_Node>(
    //     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    // );
    auto node = std::make_shared<ROS_Node>(
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);

    // rclcpp::spin(demo_ptr);
    /* main function END */

    return 0;
}

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
        
        box_marker.color.r = 1.0;   // get color by type
        box_marker.color.g = 0.38;
        box_marker.color.b = 0.27;

        // box_marker.color = colormap[boxes[i].label];

        box_array.markers.push_back(box_marker);
    }
    
    publisher->publish(box_array);
    box_array.markers.clear();
}

ROS_Node::ROS_Node(const rclcpp::NodeOptions & node_options):
    rclcpp::Node("bevdet_node", node_options),
    BEVDet(),
    sub_cloud_top_(this, "/LIDAR_TOP",  rclcpp::QoS{1}.get_rmw_qos_profile()),
    sub_img_fl_(this, "/CAM_FRONT_LEFT/image_raw",  rclcpp::QoS{1}.get_rmw_qos_profile()),
    sub_img_f_ (this, "/CAM_FRONT/image_raw",       rclcpp::QoS{1}.get_rmw_qos_profile()),
    sub_img_fr_(this, "/CAM_FRONT_RIGHT/image_raw", rclcpp::QoS{1}.get_rmw_qos_profile()),
    sub_img_bl_(this, "/CAM_BACK_LEFT/image_raw",   rclcpp::QoS{1}.get_rmw_qos_profile()),
    sub_img_b_(this,   "/CAM_BACK/image_raw",       rclcpp::QoS{1}.get_rmw_qos_profile()),
    sub_img_br_(this, "/CAM_BACK_RIGHT/image_raw",  rclcpp::QoS{1}.get_rmw_qos_profile()),
    sub_cpimg_fl_(this, "/CAM_FRONT_LEFT/image_rect_compressed",  rclcpp::QoS{1}.get_rmw_qos_profile()),
    sub_cpimg_f_ (this, "/CAM_FRONT/image_rect_compressed",       rclcpp::QoS{1}.get_rmw_qos_profile()),
    sub_cpimg_fr_(this, "/CAM_FRONT_RIGHT/image_rect_compressed", rclcpp::QoS{1}.get_rmw_qos_profile()),
    sub_cpimg_bl_(this, "/CAM_BACK_LEFT/image_rect_compressed",   rclcpp::QoS{1}.get_rmw_qos_profile()),
    sub_cpimg_b_(this,   "/CAM_BACK/image_rect_compressed",       rclcpp::QoS{1}.get_rmw_qos_profile()),
    sub_cpimg_br_(this, "/CAM_BACK_RIGHT/image_rect_compressed",  rclcpp::QoS{1}.get_rmw_qos_profile())
{
    // get launch config
    this->get_parameter("configure", config_file);
    this->get_parameter("imgstage", imgstage_file);
    this->get_parameter("bevstage", bevstage_file);
    RCLCPP_INFO(this->get_logger(), "config_filePath: %s", config_file.c_str());
    
    // TODO BEGIN: read from yaml file
    YAML::Node config = YAML::LoadFile(config_file);
    
    std::vector<std::string> cams_name = config["cams"]["cam_name"].as<std::vector<std::string>>();

    cams_intrin.clear();
    cams2ego_rot.clear();
    cams2ego_trans.clear();

    cams_intrin.resize(cams_name.size());
    cams2ego_rot.resize(cams_name.size());
    cams2ego_trans.resize(cams_name.size());

    for(size_t i = 0; i < cams_name.size(); i++){
        cams_intrin[i] = fromYamlMatrix3f(config["cams"][cams_name[i]]["cam_intrinsic"]);
        cams2ego_rot[i] = fromYamlQuater(config["cams"][cams_name[i]]["sensor2ego_rotation"]);//nuscenes.get_cams2ego_rot();
        cams2ego_trans[i] = fromYamlTrans(config["cams"][cams_name[i]]["sensor2ego_translation"]);//nuscenes.get_cams2ego_trans();
    }

    // 模型配置文件路径 
    // imgstage_file = config["ImgStageEngine"].as<std::string>();
    // bevstage_file = config["BEVStageEngine"].as<std::string>();

    // =============  bevdet_lt_depth.yaml
    config_file = config["ModelConfig"].as<std::string>(); //"src/bevdet_ros/config/bevdet/cfgs/bevdet_lt_depth.yaml";
    InitParams(config_file);

    auto start = std::chrono::high_resolution_clock::now();
    
    /* TODO: init TRT engine be applied in BEVDet::InitBEVDet()*/
    // 初始化视角转换
    InitViewTransformer();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> t = end - start;
    printf("InitVewTransformer cost time : %.4lf ms\n", t.count() * 1000);

    // 初始化推理引擎
    InitEngine(imgstage_file, bevstage_file); // FIXME
    MallocDeviceMemory();

    // temporarily for jpeg decode 
    // TODO: remove
    img_N_ = 6;  // 图片数量 6
    img_w_ = 1600;        // W: 1600
    img_h_ = 900;        // H: 900
    // gpu分配内参， cuda上分配6张图的大小 每个变量sizeof(uchar)个字节，并用imgs_dev指向该gpu上内存, sizeof(uchar) =1
    CHECK_CUDA(cudaMalloc((void**)&imgs_dev_, img_N_ * 3 * img_w_ * img_h_ * sizeof(uchar)));

    // ==================  ROS2 Sub&Pub ========================= //
    sync_queue_size_ = declare_parameter<int>("sync_queue_size", 60);   // 10 for each img
    // Create publishers and subscribers
    using std::placeholders::_1;using std::placeholders::_2;
    using std::placeholders::_3;using std::placeholders::_4;
    using std::placeholders::_5;using std::placeholders::_6;
    // using std::placeholders::_7;
    sync_ptr_ = std::make_shared<Sync>(SyncPolicy(sync_queue_size_), 
        // sub_cloud_top_,
        sub_img_fl_, sub_img_f_, sub_img_fr_,
        sub_img_b_, sub_img_bl_, sub_img_br_);
    sync_ptr_->registerCallback(
        std::bind(&ROS_Node::callback, this, _1, _2, _3, _4, _5, _6));
    // Compressed
    // sync_cp_ptr_ = std::make_shared<SyncCp>(SyncCpPolicy(sync_queue_size_), 
    //     sub_cloud_top_,
    //     sub_cpimg_fl_, sub_cpimg_f_, sub_cpimg_fr_,
    //     sub_cpimg_b_, sub_cpimg_bl_, sub_cpimg_br_);
    // sync_cp_ptr_->registerCallback(
    //     std::bind(&ROS_Node::callbackCompressed, this, _1, _2, _3, _4, _5, _6, _7));

    pub_stitched_img = create_publisher<sensor_msgs::msg::Image>(
        "/output/object", rclcpp::QoS{1});
    pub_cloud_top = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/LIDAR_TOP_retime", rclcpp::QoS{1});
    markers = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/markers/infered", rclcpp::QoS{1});

    // test
    // timer_ = this->create_wall_timer(
    //         500ms, std::bind(&ROS_Node::timer_callback, this)
    // );
}

ROS_Node::~ROS_Node(){
    RCLCPP_INFO(rclcpp::get_logger("bevdet_node"), "Destructing ROS_Node");
}

/* Image RGB */
void ROS_Node::callback(
    // const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_cloud, 
    const sensor_msgs::msg::Image::ConstSharedPtr & img_fl_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & img_f_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & img_fr_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & img_b_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & img_bl_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & img_br_msg)
{
    RCLCPP_INFO(rclcpp::get_logger("bevdet_node"), "new Img callback");

    cv::Mat img_fl, img_f, img_fr, img_bl, img_b, img_br;
    std::vector<cv::Mat> imgs;

    img_fl = cv_bridge::toCvShare(img_fl_msg , "bgr8")->image;
    img_f  = cv_bridge::toCvShare(img_f_msg, "bgr8")->image;
    img_fr = cv_bridge::toCvShare(img_fr_msg, "bgr8")->image;
    img_bl = cv_bridge::toCvShare(img_bl_msg , "bgr8")->image;
    img_b  = cv_bridge::toCvShare(img_b_msg, "bgr8")->image;
    img_br = cv_bridge::toCvShare(img_br_msg, "bgr8")->image;

    imgs.emplace_back(img_fl);
    imgs.emplace_back(img_f);
    imgs.emplace_back(img_fr);
    imgs.emplace_back(img_bl);
    imgs.emplace_back(img_b);
    imgs.emplace_back(img_br);

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

    DoInfer(imgs_dev_);

    //publish
    publish_boxes(this->markers, ego_boxes);
}

void ROS_Node::callbackCompressed(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_cloud, 
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_fl_msg,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_f_msg,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_fr_msg,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_b_msg,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_bl_msg,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_br_msg)
{
    RCLCPP_INFO(rclcpp::get_logger("bevdet_node"), "new CompressedImg callback");

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

void ROS_Node::TestNuscenes(YAML::Node &config){
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
    // for(int i = 0; i < nuscenes.size(); i++){
    for(int i = 0; i < 20; i++){
        ego_boxes.clear();
        float time = 0.f;
        bevdet.DoInfer(nuscenes.data(i), ego_boxes, time, i);
        publish_boxes(this->markers, ego_boxes);

        if(i != 0){
            sum_time += time;
            cnt++;
        }
        // Boxes2Txt(ego_boxes, output_dir + "/bevdet_egoboxes_" + std::to_string(i) + ".txt", true);
    }
    printf("Infer mean cost time : %.5lf ms\n", sum_time / cnt);
}
