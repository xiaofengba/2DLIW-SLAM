#include "utilies/common.h"
#include <iostream>
#include <rclcpp/rclcpp.hpp>

namespace convert
{
    std::tuple<std::shared_ptr<std::vector<Eigen::Vector3d>>, std::shared_ptr<std::vector<double>>>
    laser_to_point_times(const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg)
    {
        float angle_start = msg->angle_min;
        float angle_increment = msg->angle_increment;
        // range_min/max 等字段名在 ROS 2 中保持一致
        float time_increment = msg->time_increment;
        
        // ROS 2 获取秒级时间戳的写法
        double time = rclcpp::Time(msg->header.stamp).seconds();

        auto points_ptr = std::make_shared<std::vector<Eigen::Vector3d>>();
        auto times_ptr = std::make_shared<std::vector<double>>();

        points_ptr->reserve(msg->ranges.size());
        
        if (angle_increment > 0)
        {
            for (size_t i = 0; i < msg->ranges.size(); i++)
            {
                float r = msg->ranges[i];
                if (!std::isnan(r) && !std::isinf(r) && r > 0.1)
                {
                    auto point = Eigen::Vector3d(cos(angle_start + i * angle_increment) * r,
                                                 sin(angle_start + i * angle_increment) * r,
                                                 0);
                    if (!points_ptr->empty())
                    {
                        if ((point - points_ptr->back()).norm() < 0.01)
                            continue;
                    }
                    points_ptr->emplace_back(point);
                    times_ptr->emplace_back(time + i * time_increment);
                }
            }
        }
        else
        {
            // 在 ROS 2 中建议使用自己的 Logger 打印错误，或保留标准输出
            std::cerr << "error happened: angle_increment <= 0" << std::endl;
            exit(-1);
        }
        return {points_ptr, times_ptr};
    }
} // namespace convert

namespace e_laser
{
    // ICP_solve 实现逻辑与框架无关，保持原样即可
    Eigen::Isometry3d ICP_solve(const std::vector<Eigen::Vector3d> &p1, const std::vector<Eigen::Vector3d> &p2)
    {
        // ... (保持原有的质心计算和 SVD 逻辑)
        Eigen::Vector3d p_center1(0, 0, 0), p_center2(0, 0, 0);
        for (size_t i = 0; i < p1.size(); i++) {
            p_center1 += p1[i];
            p_center2 += p2[i];
        }
        p_center1 /= (double)p1.size();
        p_center2 /= (double)p2.size();
        
        Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
        for (size_t i = 0; i < p1.size(); i++) {
            W += (p1[i] - p_center1) * (p2[i] - p_center2).transpose();
        }
        
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d R12 = svd.matrixU() * svd.matrixV().transpose();
        
        // 检查行列式，防止反射矩阵
        if (R12.determinant() < 0) {
            Eigen::Matrix3d V = svd.matrixV();
            V.col(2) *= -1.0;
            R12 = svd.matrixU() * V.transpose();
        }

        Eigen::Vector3d t12 = p_center1 - R12 * p_center2;
        Eigen::Isometry3d T12 = Eigen::Isometry3d::Identity();
        T12.linear() = R12;
        T12.translation() = t12;
        return T12;
    }
} // namespace e_laser



namespace e_cv
{
    double triangulate(const Eigen::Vector3d &cam_point1,
                       const Eigen::Vector3d &cam_point2,
                       const Eigen::Isometry3d &tf_1_2)
    {
        Eigen::Matrix3d R_12 = tf_1_2.matrix().block<3, 3>(0, 0);
        Eigen::Vector3d t_12 = tf_1_2.matrix().block<3, 1>(0, 3);
        if (t_12.norm() < 0.001)
            return 0;
        Eigen::Matrix<double, 3, 2> magic_matrix;
        magic_matrix.block<3, 1>(0, 0) = cam_point1;
        magic_matrix.block<3, 1>(0, 1) = -R_12 * cam_point2;

        Eigen::JacobiSVD<Eigen::Matrix<double, 3, 2>> svd(magic_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Vector2d s = svd.solve(t_12);

        return s(1);
    }
    std::tuple<double, double> triangulate_SVD(Eigen::Vector3d cam_point1, Eigen::Vector3d cam_point2, Eigen::Isometry3d &tf_1_2)
    {
        Eigen::Isometry3d T_1_w = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d T_2_w = tf_1_2.inverse();
        double u1 = cam_point1(0);
        double v1 = cam_point1(1);
        double u2 = cam_point2(0);
        double v2 = cam_point2(1);

        Eigen::Matrix4d magic_matrix;
        magic_matrix.row(0) = T_1_w.matrix().row(2) * u1 - T_1_w.matrix().row(0);
        magic_matrix.row(1) = T_1_w.matrix().row(2) * v1 - T_1_w.matrix().row(1);

        magic_matrix.row(2) = T_2_w.matrix().row(2) * u2 - T_2_w.matrix().row(0);
        magic_matrix.row(3) = T_2_w.matrix().row(2) * v2 - T_2_w.matrix().row(1);

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(magic_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // Eigen::Matrix4d U = svd.matrixU();
        Eigen::Matrix4d V = svd.matrixV();
        // Eigen::Vector4d S = svd.singularValues();

        Eigen::Vector4d p_w = V.col(3);
        p_w = p_w / p_w(3);
        double s1 = p_w(2);
        double s2 = (T_2_w * p_w.block<3, 1>(0, 0))(2);

        return {s1, (T_2_w * p_w.block<3, 1>(0, 0) / s2 - cam_point2).norm()};
    }

    std::tuple<Eigen::Vector3d, double> triangulate_points_SVD(const std::vector<Eigen::Vector3d> &cam_points,
                                                               const std::vector<Eigen::Isometry3d> &tfs)
    {
        assert(cam_points.size() == tfs.size());
        assert(cam_points.size() >= 2);

        Eigen::MatrixXd magic_matrix(cam_points.size() * 2, 4);
        for (int i = 0; i < cam_points.size(); i++)
        {
            double u = cam_points[i](0);
            double v = cam_points[i](1);
            Eigen::Isometry3d tf_i_w = tfs[i].inverse();
            magic_matrix.row(i * 2 + 0) = tf_i_w.matrix().row(2) * u - tf_i_w.matrix().row(0);
            magic_matrix.row(i * 2 + 1) = tf_i_w.matrix().row(2) * v - tf_i_w.matrix().row(1);
        }
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(magic_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix4d V = svd.matrixV();

        Eigen::Vector3d p_w = V.col(3).block<3, 1>(0, 0);
        p_w /= V(3, 3);

        double error = 0;
        for (int i = 0; i < cam_points.size(); i++)
        {
            Eigen::Vector3d tmp = tfs[i].inverse() * p_w;
            tmp /= tmp(2);
            error += (tmp - cam_points[i]).norm();
        }
        error /= cam_points.size();
        return {p_w.block<3, 1>(0, 0), error};
    }

} // namespace e_cv
