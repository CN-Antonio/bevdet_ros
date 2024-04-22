#include "bevdet_node.h"

#ifdef ROS2_FOUND
std::mutex g_mtx;
std::condition_variable g_cv;
#endif

static void sigHandler(int sig)
{
    RCLCPP_INFO(rclcpp::get_logger("bevdet_node"), "Node is stopping.....");
#ifdef ROS_FOUND
    ros::shutdown();
#elif ROS2_FOUND
    g_cv.notify_all();
#endif
}

int main(int argc, char **argv)
{
    signal(SIGINT, sigHandler);  ///< bind ctrl+c signal with the sigHandler function

#ifdef DEBUG
    for(int i=0; i<argc; i++){
        std::cout<< "argv["<<i<<"]: " <<argv[i] <<std::endl;
    }
#endif

#ifdef ROS_FOUND
#elif ROS2_FOUND
    rclcpp::init(argc, argv);
    // rclcpp::executors::SingleThreadedExecutor executor;
#endif

    /* main function BEGIN */
    std::shared_ptr<ROS_Node> demo_ptr = std::make_shared<ROS_Node>(
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    // Gets config file from ros parameter ~config
    // demo_ptr->get_parameter();

    // test
    rclcpp::spin(demo_ptr);
    /* main function END */

    // for loop run
#ifdef ROS_FOUND
    ros::spin();
#elif ROS2_FOUND
    std::unique_lock<std::mutex> lck(g_mtx);
    g_cv.wait(lck);
#endif

    return 0;
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
    sub_img_br_(this, "/CAM_BACK_RIGHT/image_raw",  rclcpp::QoS{1}.get_rmw_qos_profile())
{
    // getRosParams()
    pkg_path_ = "/home/antonio/Projects/bevdet_ros";
    std::string config_path = pkg_path_ + "/config/bevdet/config.yaml";
    // config_path = declare_parameter<std::string>("config1", "flashocc.yaml");
    // this->get_parameter("config", config_file);  // TODO: replace with "declare_parameter"
    RCLCPP_INFO(this->get_logger(), "config_filePath: %s", config_path.c_str());

    // BEVDet config yaml //
    config_ = YAML::LoadFile(config_path);

    img_N_ = config_["N"].as<size_t>();  // 图片数量 6
    img_w_ = config_["W"].as<int>();        // H: 900
    img_h_ = config_["H"].as<int>();        // W: 1600

    // 模型配置文件路径 
    model_config_ = pkg_path_ + "/" + config_["ModelConfig"].as<std::string>();
    
    // 权重文件路径 图像部分 bev部分
    imgstage_file_ =pkg_path_ + "/" +  config_["ImgStageEngine"].as<std::string>();
    bevstage_file_ =pkg_path_ +"/" +  config_["BEVStageEngine"].as<std::string>();
    
    // 相机的内参配置参数
    camconfig_ = YAML::LoadFile(pkg_path_ +"/" + config_["CamConfig"].as<std::string>()); 
    // 结果保存文件
    output_lidarbox_ = pkg_path_ +"/" + config_["OutputLidarBox"].as<std::string>();

    // Log config
    // RCLCPP_INFO(this->get_logger(), "Image: %dx%dx%d", img_N_, img_w_, img_h_);
    RCLCPP_INFO(this->get_logger(), "Image: %dx%dx%d\nmodel_config_: %s\n stage: %s, %s\n",
        img_N_, img_w_, img_h_,
        model_config_.c_str(),
        imgstage_file_.c_str(), bevstage_file_.c_str()
        );
    

    
    
    // TODO: init TRT engine and pending
    // gpu分配内参， cuda上分配6张图的大小 每个变量sizeof(uchar)个字节，并用imgs_dev指向该gpu上内存, sizeof(uchar) =1
    // CHECK_CUDA(cudaMalloc((void**)&imgs_dev_, img_N_ * 3 * img_w_ * img_h_ * sizeof(uchar)));

    sync_queue_size_ = declare_parameter<int>("sync_queue_size", 60);   // 10 for each img
    // Create publishers and subscribers
    using std::placeholders::_1;using std::placeholders::_2;
    using std::placeholders::_3;using std::placeholders::_4;
    using std::placeholders::_5;using std::placeholders::_6;
    using std::placeholders::_7;
    sync_ptr_ = std::make_shared<Sync>(SyncPolicy(sync_queue_size_), 
        sub_cloud_top_,
        sub_img_fl_, sub_img_f_, sub_img_fr_,
        sub_img_b_, sub_img_bl_, sub_img_br_);
    sync_ptr_->registerCallback(
        std::bind(&ROS_Node::callback, this, _1, _2, _3, _4, _5, _6, _7));
    stitched_img_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "output/object", rclcpp::QoS{1});

    // test
    // timer_ = this->create_wall_timer(
    //         500ms, std::bind(&ROS_Node::timer_callback, this)
    // );
}

ROS_Node::~ROS_Node(){
    RCLCPP_INFO(rclcpp::get_logger("flashocc_node"), "Destructing ROS_Node");
}

void ROS_Node::Callback_imgs(
    const sensor_msgs::msg::Image::ConstSharedPtr & img_fl_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & img_f_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & img_fr_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & img_b_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & img_bl_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & img_br_msg)
{
    RCLCPP_INFO(rclcpp::get_logger("flashocc_node"), "imgs sync callback");
    
    // TODO: flashocc DoInfer
    
    // TODO: pub outputs

    // TODO: test: stitch 6 img and publish
}

void ROS_Node::callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_cloud, 
        const sensor_msgs::msg::Image::ConstSharedPtr & img_fl_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_f_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_fr_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_b_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_bl_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_br_msg)
{
    RCLCPP_INFO(rclcpp::get_logger("bevdet_node"), "pcl & imgs sync callback");

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    pcl::fromROSMsg(*msg_cloud, *cloud);

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


    // std::vector<Box> ego_boxes;
    // ego_boxes.clear();
    // TODO: flashocc DoInfer
    this->DoInfer();
    
    // TODO: pub outputs

    // TODO: test: stitch 6 img and publish
    std::vector<std::vector<char>> imgs_data;
    cvImgToArr(imgs, imgs_data);

    cv::Mat img_stitched = cv::Mat(900*6, 1600, CV_8UC3, (void *)std::data(imgs_data));

    //create ROS2 messages
    sensor_msgs::msg::Image _img_msg;
    std_msgs::msg::Header _header;
    cv_bridge::CvImage _cv_bridge;
    _header.stamp = this->get_clock() -> now();
    _cv_bridge = cv_bridge::CvImage(_header, sensor_msgs::image_encodings::BGR8, img_stitched);
    _cv_bridge.toImageMsg(_img_msg);

    //publish
    stitched_img_pub_ -> publish(_img_msg);
    
}

// test
void ROS_Node::timer_callback()
{
    if(timer_count_>999) timer_count_=0;

    auto message = std_msgs::msg::String();
    message.data = "Hello_World: " + std::to_string(timer_count_);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

    timer_count_++;
}
