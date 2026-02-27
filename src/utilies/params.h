#pragma once
#include "timerAndColor/color.h"
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp> // 替换 ros/ros.h
#include <yaml-cpp/yaml.h>
#include <memory>

#define PARAM(name) (lvio_2d::param::manager::get_param_manager()->name)

namespace lvio_2d
{
    enum TRAJECTORY_STATUS { INITIALIZING, TRACKING };

    namespace param
    {
        class manager
        {
        public:
            using ptr = std::shared_ptr<manager>;

            // --- 参数字段保持不变 ---
            std::string wheel_odom_topic, laser_topic, imu_topic, camera_topic;
            Eigen::Isometry3d T_imu_to_camera, T_imu_to_laser, T_imu_to_wheel;
            Eigen::Vector3d imu_noise_acc_sigma, imu_bias_acc_sigma, imu_noise_gyro_sigma, imu_bias_gyro_sigma, wheel_sigma;
            Eigen::Vector2d camera_sigma;
            double max_camera_reproject_error, max_camera_feature_dis;
            int max_feature_num, slide_window_size;
            double p_motion_threshold, q_motion_threshold, g;
            Eigen::Matrix3d camera_K;
            double manifold_p_sigma, manifold_q_sigma;
            double w_laser_each_scan, h_laser_each_scan, laser_resolution;
            double line_continuous_threshold, line_max_tolerance_angle, line_min_len, line_max_dis, line_to_line_sigma;
            bool enable_camera, enable_laser, enable_camera_vis, enable_laser_vis;
            double feature_min_dis, min_delta_t;
            int FPS;
            double key_frame_p_motion_threshold, key_frame_q_motion_threshold;
            double a_res, d_res, verify_loop_rate, loop_max_dis, loop_edge_k;
            double ref_motion_filter_p, ref_motion_filter_q, loop_max_tf_p, loop_max_tf_q;
            int submap_count, laser_loop_min_match_threshold, loop_detect_min_interval, ref_n_accumulation;
            std::string output_dir;
            bool output_tum, use_ground_p_factor, use_ground_q_factor, fast_mode;
            Eigen::Vector3d loop_sigma_p, loop_sigma_q;

            // --- ROS 2 适配方法 ---
            // 第一次调用时需要传入 node 指针
            static manager::ptr get_param_manager(rclcpp::Node::SharedPtr node = nullptr);

            static void update_T_imu_to_camera(const Eigen::Isometry3d &T_i_c) {
                get_param_manager()->T_imu_to_camera = T_i_c;
            }

        private:
            // 构造函数现在接收 ROS 2 Node 指针
            manager(rclcpp::Node::SharedPtr node);
            bool check_param();
        };
    } 
}