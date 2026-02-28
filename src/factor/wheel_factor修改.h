#pragma once
#include "factor/factor_common.h"
#include "factor/wheel_odom_preintegration.h"
namespace lvio_2d
{
    struct wheel_odom_factor
    {
        wheel_odom_preint_result::ptr wheel_odom_preint_result_;
        wheel_odom_factor(const wheel_odom_preint_result::ptr &wheel_odom_preint_result_) : wheel_odom_preint_result_(wheel_odom_preint_result_)
        {
        }
        template <typename T>
        bool operator()(const T *const p_w_i,
                        const T *const theta_w_i,
                        const T *const p_w_j,
                        const T *const theta_w_j,
                        T *res) const
        {
            using vector3 = Eigen::Matrix<T, 3, 1>;

            const Eigen::Map<const vector3> pi(p_w_i);
            const Eigen::Map<const vector3> thetai(theta_w_i);

            const Eigen::Map<const vector3> pj(p_w_j);
            const Eigen::Map<const vector3> thetaj(theta_w_j);

            Eigen::Transform<T, 3, Eigen::Isometry> T_i_w = PARAM(T_imu_to_wheel).template cast<T>();

            Eigen::Transform<T, 3, Eigen::Isometry> tf_i = lie::make_tf<T>(pi, thetai) * T_i_w;
            Eigen::Transform<T, 3, Eigen::Isometry> tf_j = lie::make_tf<T>(pj, thetaj) * T_i_w;

            Eigen::Transform<T, 3, Eigen::Isometry> w_tf_ij = tf_i.inverse() * tf_j;

            // 提取当前优化的相对位姿 (p, q) 和 里程计预积分的测量位姿 (op, oq)
            auto [p, q] = lie::log_SE3(w_tf_ij);
            auto [op, oq] = lie::log_SE3<T>(wheel_odom_preint_result_->delta_Tij.cast<T>());

            // 【修复 1：安全的长度计算】
            // 在根号内加上极小值 1e-12，防止机器人静止时导数分母为 0 导致 NaN
            T len = ceres::sqrt(p(0) * p(0) + p(1) * p(1) + T(1e-12));
            T o_len = ceres::sqrt(op(0) * op(0) + op(1) * op(1) + T(1e-12));

            // 【修复 2：安全的方向误差计算】
            // 利用二维叉积公式直接求夹角的 sin 值，避开 norm() 和 asin() 的双重奇异点
            T cross_z = p(0) * op(1) - p(1) * op(0);
            T safe_denom = len * o_len;
            T angle_sin = cross_z / safe_denom;

            // 组装残差 0：移动距离误差
            res[0] = T(wheel_odom_preint_result_->sqrt_inverse_P(0, 0)) * (len - o_len);
            
            // 组装残差 1：移动方向误差
            res[1] = T(wheel_odom_preint_result_->sqrt_inverse_P(1, 1)) * angle_sin;
            
            // 【修复 3：修复严重逻辑 Bug，恢复旋转符号】
            // 在 2D 中，旋转完全集中在 Z 轴。直接作差，不再使用丢失符号的 norm()！
            res[2] = T(wheel_odom_preint_result_->sqrt_inverse_P(2, 2)) * (q(2) - oq(2));

            return true;
        }

        static ceres::CostFunction *Create(
            const wheel_odom_preint_result::ptr &wheel_odom_preint_result_)
        {
            return new ceres::AutoDiffCostFunction<wheel_odom_factor, 3, 3, 3, 3, 3>(
                new wheel_odom_factor(wheel_odom_preint_result_));
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

} // namespace lvio_2d