#pragma once
#include "macro.h"
#include "trajectory/camera_type.h"
#include "trajectory/keyframe_type.h"
#include "trajectory/laser_type.h"
#include "utilies/common.h"
#include "utilies/params.h"

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/transform_broadcaster.h>

// ROS 2 消息头文件
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>
#include <deque>
#include <unordered_map>

namespace lvio_2d
{
    class visualization
    {
    public:
        using ptr = std::shared_ptr<visualization>;
        // ROS 2 需要传入 Node 指针来初始化发布者
        static ptr get_ptr(rclcpp::Node::SharedPtr node = nullptr);

        visualization(rclcpp::Node::SharedPtr node);
        ~visualization();

        void add_image_to_show(const std::string &topic, const cv::Mat &src);
        void add_status_to_show(const Eigen::Isometry3d &tf, const TRAJECTORY_STATUS &status, const double &time);
        void add_path_to_show(const std::string &topic, const std::vector<Eigen::Isometry3d> &path);
        void add_odom_to_show(const std::string &topic, const Eigen::Isometry3d &pose, const Eigen::Vector3d &linear, const Eigen::Vector3d &angular);
        void add_scan_to_show(const scan::ptr &scan_ptr);
        void add_laser_map_to_show(const laser_map::ptr &laser_map_ptr);
        void add_feature_map_to_show(const feature_map::ptr &feature_map_ptr);
        void add_laser_match_to_show(const laser_match::ptr &laser_match_ptr);

    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<image_transport::ImageTransport> it_;
        
        std::unordered_map<std::string, image_transport::Publisher> image_pubs;
        std::unordered_map<std::string, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> path_pubs;
        std::unordered_map<std::string, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odom_pubs;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br;

        const Eigen::Isometry3d &T_imu_to_camera;
        const Eigen::Isometry3d &T_imu_to_laser;
        const Eigen::Isometry3d &T_imu_to_wheel;

        std::thread visualization_thread;
        // 任务队列保持不变...
        std::deque<std::tuple<std::string, cv::Mat>> image_tasks;
        std::deque<std::tuple<Eigen::Isometry3d, TRAJECTORY_STATUS, double>> status_tasks;
        std::deque<std::tuple<std::string, std::vector<Eigen::Isometry3d>>> path_tasks;
        std::deque<scan::ptr> scan_tasks;
        std::deque<laser_map::ptr> laser_map_tasks;
        std::deque<std::tuple<std::string, Eigen::Isometry3d, Eigen::Vector3d, Eigen::Vector3d>> odom_tasks;
        std::deque<std::tuple<feature_map::ptr>> feature_map_tasks;
        std::deque<std::tuple<laser_match::ptr>> laser_match_tasks;

        std::mutex task_mutex;
        std::condition_variable task_cv;
        bool quit;

        void do_visualization();
        void do_status_show(const Eigen::Isometry3d &tf, const TRAJECTORY_STATUS &status, const double &time);
        void do_image_show(const std::string &topic, const cv::Mat &src);
        void do_path_show(const std::string &topic, const std::vector<Eigen::Isometry3d> &path);
        void do_scan_to_show(const scan::ptr &scan_ptr);
        void do_laser_map_to_show(const laser_map::ptr &laser_map_ptr);
        void do_odom_to_show(const std::string &topic, const Eigen::Isometry3d &pose, const Eigen::Vector3d &linear, const Eigen::Vector3d &angular);
        void do_feature_map_to_show(const feature_map::ptr &feature_map_ptr);
        void do_laser_match_to_show(const laser_match::ptr &laser_match_ptr);
    };
}