#pragma once
#include "factor/factor_common.h"
namespace lvio_2d
{
    struct ground_noise
    {
        double manifold_p_sqrt_info;
        double manifold_q_sqrt_info;

        using const_ptr = std::shared_ptr<const ground_noise>;
        static const_ptr get_ground_noise()
        {
            static const_ptr ret = nullptr;
            if (!ret)
                ret = std::make_shared<const ground_noise>();
            return ret;
        }
        ground_noise()
        {
            manifold_p_sqrt_info = 1.0 / PARAM(manifold_p_sigma);
            manifold_q_sqrt_info = 1.0 / PARAM(manifold_q_sigma);
        }
    };

    struct ground_factor_p
    {
        template <typename T>
        bool operator()(const T *const p_w_i,
                        const T *const theta_w_i,
                        T *res) const
        {
            using vector3 = Eigen::Matrix<T, 3, 1>;
            using vector6 = Eigen::Matrix<T, 6, 1>;
            using matrix3 = Eigen::Matrix<T, 3, 3>;

            Eigen::Transform<T, 3, Eigen::Isometry> tf_w_i = lie::make_tf<T>(
                Eigen::Map<const Eigen::Matrix<T, 3, 1>>(p_w_i),
                Eigen::Map<const Eigen::Matrix<T, 3, 1>>(theta_w_i));

            Eigen::Transform<T, 3, Eigen::Isometry> T_i_w = PARAM(T_imu_to_wheel).template cast<T>();

            Eigen::Transform<T, 3, Eigen::Isometry> tf_w_o = tf_w_i * T_i_w;

            T dis_from_plane = tf_w_o.matrix()(2, 3);

            res[0] = T(ground_noise::get_ground_noise()->manifold_p_sqrt_info) * dis_from_plane;
            return true;
        }
        static ceres::CostFunction *Create()
        {
            return new ceres::AutoDiffCostFunction<ground_factor_p, 1, 3, 3>(
                new ground_factor_p());
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    struct ground_factor_q
    {
        template <typename T>
        bool operator()(const T *const p_w_i,
                        const T *const theta_w_i,
                        T *res) const
        {
            using vector3 = Eigen::Matrix<T, 3, 1>;
            using vector6 = Eigen::Matrix<T, 6, 1>;
            using matrix3 = Eigen::Matrix<T, 3, 3>;

            // 1. 获取 IMU 到世界坐标系的变换
            Eigen::Transform<T, 3, Eigen::Isometry> tf_w_i = lie::make_tf<T>(
                Eigen::Map<const Eigen::Matrix<T, 3, 1>>(p_w_i),
                Eigen::Map<const Eigen::Matrix<T, 3, 1>>(theta_w_i));

            // 2. 获取底盘（轮式里程计）到世界坐标系的变换
            Eigen::Transform<T, 3, Eigen::Isometry> T_i_w = PARAM(T_imu_to_wheel).template cast<T>();
            Eigen::Transform<T, 3, Eigen::Isometry> tf_w_o = tf_w_i * T_i_w;

            // 3. 提取底盘坐标系的 Z 轴在世界坐标系下的朝向
            vector3 z_axis = tf_w_o.matrix().template block<3, 1>(0, 2);
            
            // 4. 【核心修复】计算倾斜度，避开 norm() 的求导奇点
            // 机器人的 Z 轴向量为 [x, y, z]。当机器人未倾斜时，x 和 y 应当为 0。
            // 倾斜角度的 sin 值刚好等于 sqrt(x^2 + y^2)。
            // 加上 1e-12（极其微小的值），防止在完美平贴地面时 sqrt(0) 引发导数除零错误。
            T tilt_squared = z_axis(0) * z_axis(0) + z_axis(1) * z_axis(1);
            T safe_angle_approx = ceres::sqrt(tilt_squared + T(1e-12));

            res[0] = T(ground_noise::get_ground_noise()->manifold_q_sqrt_info) * safe_angle_approx;
            return true;
        }
        static ceres::CostFunction *Create()
        {
            return new ceres::AutoDiffCostFunction<ground_factor_q, 1, 3, 3>(
                new ground_factor_q());
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

} // namespace lvio_2d