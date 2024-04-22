/*
ROS related .h file
*/
#ifndef __BEVDET_NODE_H__
#define __BEVDET_NODE_H__

// CPP
#include <chrono>

#include "bevdet/bevdet.h"

#ifdef ROS_FOUND
#elif ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h> // sync 6 cam imgs
#include <message_filters/sync_policies/approximate_time.h> // 时间相近同步
#endif // ROS/2_FOUND

using namespace std::chrono_literals; // 500ms

class ROS_Node : public rclcpp::Node, public BEVDet
{
public:
    explicit ROS_Node(const rclcpp::NodeOptions & node_options);
    ~ROS_Node();
protected:
private:
    // ROS Init
    std::string pkg_path_;
    

    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stitched_img_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stitched_img_pub_; // test
    // synced msgs sub
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_cloud_top_{}; 
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_fl_{};
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_f_{};
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_fr_{}; 
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_b_{}; 
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_bl_{};
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_br_{};

    // sync 6 cam imgs
    int sync_queue_size_;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, 
        sensor_msgs::msg::Image, sensor_msgs::msg::Image,
        sensor_msgs::msg::Image, sensor_msgs::msg::Image,
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;
    typename std::shared_ptr<Sync> sync_ptr_;
    
    void Callback_imgs(//const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_cloud, 
        const sensor_msgs::msg::Image::ConstSharedPtr & img_fl_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_f_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_fr_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_b_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_bl_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_br_msg
    );

    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_cloud, 
        const sensor_msgs::msg::Image::ConstSharedPtr & img_fl_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_f_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_fr_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_b_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_bl_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & img_br_msg
    );

    // test
    size_t timer_count_;
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();
};

#endif // __BEVDET_NODE_H__
