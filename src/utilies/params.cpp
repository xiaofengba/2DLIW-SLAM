#include "params.h"
#include "utilies/common.h"

// 辅助宏：ROS 2 风格
#define ECHO_PARAM(name) GREEN_INFO(true, #name << ":\t" << name << std::endl)

// 在 ROS 2 中，必须先 declare 再 get
#define LOAD_PARAM(node, name) \
    do { \
        node->declare_parameter(#name, name); \
        node->get_parameter(#name, name); \
        ECHO_PARAM(name); \
    } while (0)

// // 处理 Eigen 类型 (ROS 2 参数通常是 std::vector)
// template <typename T>
// T load_eigen_param(rclcpp::Node::SharedPtr node, const std::string &name, int rows, int cols) {
//     std::vector<double> vec;
//     node->declare_parameter(name, std::vector<double>(rows * cols, 0.0));
//     node->get_parameter(name, vec);
    
//     T res;
//     if (rows == 4 && cols == 4) { // Isometry3d
//         Eigen::Matrix4d mat;
//         for (int i = 0; i < 16; ++i) mat.data()[i] = vec[i]; // 注意 Data() 是列优先
//         // 或者按照你原来的逻辑 [r*4+c]
//         for(int r=0; r<4; ++r) for(int c=0; c<4; ++c) mat(r,c) = vec[r*4+c];
//         res = Eigen::Isometry3d(mat);
//     } else {
//         for(int r=0; r<rows; ++r) 
//             for(int c=0; c<cols; ++c) 
//                 res(r,c) = vec[r*cols + c];
//     }
//     return res;
// }

template <typename T>
T load_eigen_param(rclcpp::Node::SharedPtr node, const std::string &name, int rows, int cols) {
    std::vector<double> vec;
    node->declare_parameter(name, std::vector<double>(rows * cols, 0.0));
    node->get_parameter(name, vec);
    
    T res;
    // 【关键修改】：使用 if constexpr 和 std::is_same_v 在编译期判断类型 T
    if constexpr (std::is_same_v<T, Eigen::Isometry3d>) { 
        Eigen::Matrix4d mat;
        for(int r=0; r<4; ++r) 
            for(int c=0; c<4; ++c) 
                mat(r,c) = vec[r*4+c];
        res = Eigen::Isometry3d(mat);
    } else {
        // 如果 T 不是 Isometry3d (比如是 Vector3d, Matrix3d)，只编译这部分
        for(int r=0; r<rows; ++r) 
            for(int c=0; c<cols; ++c) 
                res(r,c) = vec[r*cols + c];
    }
    return res;
}

namespace lvio_2d {
    namespace param {

        manager::ptr manager::get_param_manager(rclcpp::Node::SharedPtr node) {
            static manager::ptr ptr = nullptr;
            if (!ptr) {
                if (!node) {
                    throw std::runtime_error("Param manager must be initialized with a node pointer first!");
                }
                ptr = manager::ptr(new manager(node));
            }
            return ptr;
        }

        manager::manager(rclcpp::Node::SharedPtr node) {
            // 加载基础类型
            LOAD_PARAM(node, wheel_odom_topic);
            LOAD_PARAM(node, laser_topic);
            LOAD_PARAM(node, imu_topic);
            LOAD_PARAM(node, camera_topic);
            LOAD_PARAM(node, g);
            LOAD_PARAM(node, manifold_p_sigma);
            LOAD_PARAM(node, manifold_q_sigma);

            // 加载 Eigen 类型
            T_imu_to_camera = load_eigen_param<Eigen::Isometry3d>(node, "T_imu_to_camera", 4, 4);
            T_imu_to_laser = load_eigen_param<Eigen::Isometry3d>(node, "T_imu_to_laser", 4, 4);
            T_imu_to_wheel = load_eigen_param<Eigen::Isometry3d>(node, "T_imu_to_wheel", 4, 4);

            imu_noise_acc_sigma = load_eigen_param<Eigen::Vector3d>(node, "imu_noise_acc_sigma", 3, 1);
            imu_bias_acc_sigma = load_eigen_param<Eigen::Vector3d>(node, "imu_bias_acc_sigma", 3, 1);
            imu_noise_gyro_sigma = load_eigen_param<Eigen::Vector3d>(node, "imu_noise_gyro_sigma", 3, 1);
            imu_bias_gyro_sigma = load_eigen_param<Eigen::Vector3d>(node, "imu_bias_gyro_sigma", 3, 1);
            wheel_sigma = load_eigen_param<Eigen::Vector3d>(node, "wheel_sigma", 3, 1);
            loop_sigma_p = load_eigen_param<Eigen::Vector3d>(node, "loop_sigma_p", 3, 1);
            loop_sigma_q = load_eigen_param<Eigen::Vector3d>(node, "loop_sigma_q", 3, 1);
            camera_sigma = load_eigen_param<Eigen::Vector2d>(node, "camera_sigma", 2, 1);
            camera_K = load_eigen_param<Eigen::Matrix3d>(node, "camera_K", 3, 3);

            // 其余参数
            LOAD_PARAM(node, max_feature_num);
            LOAD_PARAM(node, slide_window_size);
            LOAD_PARAM(node, p_motion_threshold);
            LOAD_PARAM(node, q_motion_threshold);
            LOAD_PARAM(node, w_laser_each_scan);
            LOAD_PARAM(node, h_laser_each_scan);
            LOAD_PARAM(node, laser_resolution);
            LOAD_PARAM(node, line_continuous_threshold);
            LOAD_PARAM(node, line_max_tolerance_angle);
            LOAD_PARAM(node, line_min_len);
            LOAD_PARAM(node, line_max_dis);
            LOAD_PARAM(node, line_to_line_sigma);
            LOAD_PARAM(node, enable_camera);
            LOAD_PARAM(node, enable_laser);
            LOAD_PARAM(node, enable_camera_vis);
            LOAD_PARAM(node, enable_laser_vis);
            LOAD_PARAM(node, max_camera_reproject_error);
            LOAD_PARAM(node, max_camera_feature_dis);
            LOAD_PARAM(node, feature_min_dis);
            LOAD_PARAM(node, FPS);
            LOAD_PARAM(node, key_frame_p_motion_threshold);
            LOAD_PARAM(node, key_frame_q_motion_threshold);
            LOAD_PARAM(node, a_res);
            LOAD_PARAM(node, d_res);
            LOAD_PARAM(node, submap_count);
            LOAD_PARAM(node, laser_loop_min_match_threshold);
            LOAD_PARAM(node, loop_detect_min_interval);
            LOAD_PARAM(node, output_dir);
            LOAD_PARAM(node, output_tum);
            LOAD_PARAM(node, use_ground_p_factor);
            LOAD_PARAM(node, use_ground_q_factor);
            LOAD_PARAM(node, verify_loop_rate);
            LOAD_PARAM(node, loop_max_dis);
            LOAD_PARAM(node, loop_edge_k);
            LOAD_PARAM(node, ref_motion_filter_p);
            LOAD_PARAM(node, ref_motion_filter_q);
            LOAD_PARAM(node, ref_n_accumulation);
            LOAD_PARAM(node, loop_max_tf_p);
            LOAD_PARAM(node, loop_max_tf_q);
            LOAD_PARAM(node, fast_mode);

            if (!check_param()) exit(-1);
        }

        bool manager::check_param() {
            if (!enable_laser) enable_laser_vis = false;
            if (!enable_camera) enable_camera_vis = false;
            if (!enable_camera && !enable_laser) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR: disable camera and disable laser");
                return false;
            }
            max_camera_reproject_error = max_camera_reproject_error / camera_K(0, 0);
            min_delta_t = 1.0 / double(FPS);
            return true;
        }
    } 
}