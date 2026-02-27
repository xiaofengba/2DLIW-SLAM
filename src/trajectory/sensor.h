#pragma once
#include "utilies/utilies.h"
#include <Eigen/Dense>

// cv_bridge 在 Humble 中仍保留 .h 后缀
#include <cv_bridge/cv_bridge.h> 
#include <image_transport/image_transport.hpp>

// ROS 2 消息头文件：全小写蛇形命名，带 msg 目录，.hpp 后缀
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>

#include <opencv2/opencv.hpp>

namespace lvio_2d
{
    namespace sensor
    {
        struct imu
        {
            using u_ptr = std::unique_ptr<imu>;
            Eigen::Vector3d acc;
            Eigen::Vector3d gyro;
            double time_stamp;
            
            imu() : acc(0, 0, 0),
                    gyro(0, 0, 0),
                    time_stamp(TIME_MIN)
            {
            }
            
            // 替换为 ConstSharedPtr
            imu(const sensor_msgs::msg::Imu::ConstSharedPtr &msg) 
                : acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
                  gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                  time_stamp(rclcpp::Time(msg->header.stamp).seconds()) // 替换 toSec()
            {
            }
            
            ~imu()
            {
                // std::cout << "release imu" << std::endl;
            }
        };

        struct laser
        {
            using u_ptr = std::unique_ptr<laser>;
            double time_stamp;
            std::shared_ptr<std::vector<Eigen::Vector3d>> points_ptr;
            std::shared_ptr<std::vector<double>> times_ptr;

            // 替换为 ConstSharedPtr
            laser(const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg) 
                : time_stamp(rclcpp::Time(msg->header.stamp).seconds()) // 替换 toSec()
            {
                std::tie(points_ptr, times_ptr) = convert::laser_to_point_times(msg);
            }
            
            ~laser()
            {
                // std::cout << "release laser" << std::endl;
            }
            
            void correct(const Eigen::Vector3d &linear, const Eigen::Vector3d &angular)
            {
                // cv::Mat board(800, 1600, CV_8UC3);
                // cv::rectangle(board, cv::Point(0, 0), cv::Point(1599, 799), cv::Scalar(0, 0, 0), -1);

                // Eigen::Vector3d origin(400, 400, 0);
                // double scalar = 60;

                // std::cout << "liner:" << linear.transpose() << "angluar:" << angular.transpose() << std::endl;
                for (int i = 0; i < points_ptr->size(); i++)
                {
                    double dt = (*times_ptr)[i] - time_stamp;
                    Eigen::Isometry3d T_i_j = lie::make_tf<double>(dt * linear, dt * angular);
                    
                    Eigen::Vector3d tmp2 = T_i_j * ((*points_ptr)[i]);

                    (*points_ptr)[i] = tmp2;
                }
                // cv::imshow("scan", board);
                // cv::waitKey(1);
            }
        };

        struct wheel_odom
        {
            using u_ptr = std::unique_ptr<wheel_odom>;

            Eigen::Isometry3d pose;
            double time_stamp;
            
            wheel_odom() : pose(Eigen::Isometry3d::Identity()) {}
            
            // 替换为 ConstSharedPtr
            wheel_odom(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
            {
                pose = Eigen::Isometry3d::Identity();
                Eigen::Quaterniond q(
                    msg->pose.pose.orientation.w,
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z);
                q.normalize();
                
                Eigen::Vector3d t(
                    msg->pose.pose.position.x,
                    msg->pose.pose.position.y,
                    msg->pose.pose.position.z);
                    
                pose.matrix().template block<3, 3>(0, 0) = q.toRotationMatrix();
                pose.matrix().template block<3, 1>(0, 3) = t;
                
                // 替换 toSec()
                time_stamp = rclcpp::Time(msg->header.stamp).seconds(); 
            }
            
            ~wheel_odom()
            {
                // std::cout << "release wheel odom" << std::endl;
            }
        };

        struct camera
        {
            using u_ptr = std::unique_ptr<camera>;

            cv::Mat image;
            cv::Mat color_image;
            double time_stamp;
            
            // 替换为 ConstSharedPtr
            camera(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
            {
                image = cv_bridge::toCvShare(msg, "mono8")->image;
                if (PARAM(enable_camera_vis))
                    color_image = cv_bridge::toCvShare(msg, "bgr8")->image;

                // 替换 toSec()
                time_stamp = rclcpp::Time(msg->header.stamp).seconds();
            }
            
            ~camera()
            {
                // std::cout << "release camera" << std::endl;
            }
        };
    } // namespace sensor
} // namespace lvio_2d