/*
 *ROS related .h file
 */
#ifndef __BEVDET_NODE_H__
#define __BEVDET_NODE_H__

// CPP
#include <chrono>

#include "bevdet.h"

#include "cpu_jpegdecoder.h"

#ifdef ROS_FOUND
#elif ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>  // rosmsg2pcl

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h> // sync 6 cam imgs
#include <message_filters/sync_policies/approximate_time.h> // 时间相近同步
#endif // ROS/2_FOUND

// TODO: remove
using std::chrono::high_resolution_clock;
using std::chrono::duration;
using namespace std::chrono_literals; // 500ms

class ROS_Node : public rclcpp::Node, public BEVDet
{
public:
    explicit ROS_Node(const rclcpp::NodeOptions & node_options);
    ~ROS_Node();

    // for test
    void TestNuscenes(YAML::Node &config);
protected:
    size_t img_N_;
    int img_w_; 
    int img_h_;
    uchar* imgs_dev_ = nullptr; 
private:
    // ROS Init
    std::string pkg_path_;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_stitched_img; // test
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_top;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers;
    // synced msgs sub
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_cloud_top_{}; 
    // RAW
    // message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_fl_{};
    // message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_f_{};
    // message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_fr_{}; 
    // message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_b_{}; 
    // message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_bl_{};
    // message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_br_{};
    // Compressed
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> sub_img_fl_{};
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> sub_img_f_{};
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> sub_img_fr_{}; 
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> sub_img_b_{}; 
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> sub_img_bl_{};
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> sub_img_br_{};

    // sync 6 cam imgs
    int sync_queue_size_;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, 
        // sensor_msgs::msg::Image, sensor_msgs::msg::Image,
        // sensor_msgs::msg::Image, sensor_msgs::msg::Image,
        // sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
        sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage,
        sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage,
        sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;
    typename std::shared_ptr<Sync> sync_ptr_;

    void callback(//const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_cloud, 
        const sensor_msgs::msg::Image::ConstSharedPtr & img_fl_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_f_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_fr_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_b_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_bl_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_br_msg
    );

    void callbackCompressed(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_cloud, 
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_fl_msg,
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_f_msg,
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_fr_msg,
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_b_msg,
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_bl_msg,
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr & img_br_msg
    );

    // test
    size_t timer_count_;
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();
};

#endif // __BEVDET_NODE_H__
