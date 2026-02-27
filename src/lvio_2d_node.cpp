#include <rclcpp/rclcpp.hpp>
#include "trajectory/trajectory.h"
#include "utilies/params.h"
#include "utilies/visualization.h"
#include "trajectory/dispatch.h"

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>

// 将原本继承 rclcpp::Node 的设计，改为接收 Node 智能指针的系统包装类
class LvioSystem
{
public:
    // 构造函数：这里初始化订阅者，并将轨迹指针传给调度队列
    LvioSystem(rclcpp::Node::SharedPtr node) 
        : node_(node), sensor_queues(&tra)
    {
        // 1. 初始化 QoS (相当于 ROS 1 的 queue_size)
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1000));

        // 2. 创建订阅者
        laser_sub = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            PARAM(laser_topic), qos,
            std::bind(&LvioSystem::laser_call_back, this, std::placeholders::_1));

        imu_sub = node_->create_subscription<sensor_msgs::msg::Imu>(
            PARAM(imu_topic), qos,
            std::bind(&LvioSystem::imu_call_back, this, std::placeholders::_1));

        wheel_odom_sub = node_->create_subscription<nav_msgs::msg::Odometry>(
            PARAM(wheel_odom_topic), qos,
            std::bind(&LvioSystem::wheel_odom_call_back, this, std::placeholders::_1));

        // 3. ImageTransport 在 ROS 2 中需要 node 指针
        it = std::make_shared<image_transport::ImageTransport>(node_);
        camera_sub = it->subscribe(
            PARAM(camera_topic), 1,
            std::bind(&LvioSystem::image_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(node_->get_logger(), "LVIO_2D System Initialized Successfully!");
    }

private:
    rclcpp::Node::SharedPtr node_;
    
    // 核心算法类
    lvio_2d::trajectory tra;
    lvio_2d::dispatch_queue sensor_queues;

    // 订阅者智能指针
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub;
    
    std::shared_ptr<image_transport::ImageTransport> it;
    image_transport::Subscriber camera_sub;

    // 回调函数
    void laser_call_back(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
    {
        if (!PARAM(enable_laser)) return;
        sensor_queues.add_laser_msg(msg);
    }

    void imu_call_back(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
    {
        sensor_queues.add_imu_msg(msg);
    }

    void wheel_odom_call_back(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        sensor_queues.add_wheel_odom_msg(msg);
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        if (!PARAM(enable_camera)) return;
        sensor_queues.add_camera_msg(msg);
    }
};

int main(int argc, char *argv[])
{
    // 1. 初始化 ROS 2 环境
    rclcpp::init(argc, argv);
    
    // 2. 创建一个独立的基础节点
    auto node = std::make_shared<rclcpp::Node>("lvio_2d_node");
    
    // ==========================================
    // 3. 【最关键的一步】：严格控制初始化顺序！
    // 必须在任何算法模块实例化之前，先给单例分配 node 指针
    // ==========================================
    lvio_2d::param::manager::get_param_manager(node);
    lvio_2d::visualization::get_ptr(node);
    
    // 4. 此时参数已经就绪，实例化主系统非常安全
    LvioSystem system(node);
    
    // 5. 开启事件循环
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}