#pragma once
#include <rclcpp/rclcpp.hpp> // 【修改】替换 ros/ros.h
#include "trajectory/sensor.h"
#include "trajectory/trajectory.h"
#include "utilies/common.h"
#include <condition_variable>
#include <limits>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>

// 【新增】包含 ROS 2 消息头文件以防万一
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace lvio_2d
{
    inline const std::string CAMERA = "camera";
    inline const std::string LASER = "laser";
    inline const std::string WHEEL_ODOM = "wheel_odom";
    inline const std::string IMU = "imu";

    class sensor_data
    {
    public:
        // in seconds
        double time;
        virtual void add_to_trajectory(trajectory *trajectory_ptr) = 0;

        sensor_data(const double &time) : time(time)
        {
        }
        virtual ~sensor_data()
        {
        }
    };

    template <typename DataType>
    class dispatchable_sensor_data : public sensor_data
    {
    public:
        dispatchable_sensor_data(const double &time, std::unique_ptr<DataType> &&data_ptr) : sensor_data(time),
                                                                                             data_ptr(std::move(data_ptr))
        {
        }
        void add_to_trajectory(trajectory *trajectory_ptr) override
        {
            trajectory_ptr->add_sensor_data(data_ptr);
        }

    private:
        std::unique_ptr<DataType> data_ptr;
    };

    template <typename sensor_type, typename T>
    std::unique_ptr<dispatchable_sensor_data<sensor_type>> create_dispatchable_data(const T &msg)
    {
        // 【修改】ROS 2 时间戳处理机制
        return std::unique_ptr<dispatchable_sensor_data<sensor_type>>(
            new dispatchable_sensor_data<sensor_type>(rclcpp::Time(msg->header.stamp).seconds(),
                                                      std::unique_ptr<sensor_type>(new sensor_type(msg))));
    }

    class dispatch_queue
    {
    private:
        std::unordered_map<std::string, std::deque<std::unique_ptr<sensor_data>>> dispatch_data_queues;
        trajectory *trajectory_ptr;
        double last_dispatch_time;
        std::thread dispatch_thread;

        std::mutex dispatch_mutex;
        std::condition_variable dispatch_cv;
        std::unordered_map<std::string, int> dispatch_counts;

        std::unordered_map<std::string, std::deque<double>> use_records;

        bool quit;

    public:
        ~dispatch_queue()
        {
            {
                std::unique_lock<std::mutex> lock(dispatch_mutex);
                quit = true;
                dispatch_cv.notify_one();
            }
            dispatch_thread.join();
        }
        dispatch_queue(trajectory *trajectory_ptr) : trajectory_ptr(trajectory_ptr)
        {
            last_dispatch_time = TIME_MIN;
            dispatch_data_queues[LASER] = std::deque<std::unique_ptr<sensor_data>>();
            dispatch_data_queues[IMU] = std::deque<std::unique_ptr<sensor_data>>();
            dispatch_data_queues[WHEEL_ODOM] = std::deque<std::unique_ptr<sensor_data>>();
            dispatch_data_queues[CAMERA] = std::deque<std::unique_ptr<sensor_data>>();

            dispatch_counts[LASER] = 0;
            dispatch_counts[IMU] = 0;
            dispatch_counts[WHEEL_ODOM] = 0;
            dispatch_counts[CAMERA] = 0;

            use_records[LASER] = std::deque<double>();
            use_records[IMU] = std::deque<double>();
            use_records[WHEEL_ODOM] = std::deque<double>();
            use_records[CAMERA] = std::deque<double>();

            quit = false;
            dispatch_thread = std::thread(std::bind(&dispatch_queue::dispatch, this));
        };
        
        // 【修改】消息类型改为 msg::LaserScan::ConstSharedPtr
        void add_laser_msg(const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg)
        {
            std::unique_lock<std::mutex> lock(dispatch_mutex);
            double time = rclcpp::Time(msg->header.stamp).seconds(); // 【修改】时间戳
            if (last_dispatch_time >= time)
            {
                // RCLCPP_WARN(rclcpp::get_logger("dispatch_queue"), "old laser msg.delay: %f s.throw.", last_dispatch_time - time);
                return;
            }
            if (!dispatch_data_queues[LASER].empty() && dispatch_data_queues[LASER].back()->time >= time)
            {
                RCLCPP_WARN(rclcpp::get_logger("dispatch_queue"), "unolder laser msg.delay: %f s.throw.", dispatch_data_queues[LASER].back()->time - time);
                return;
            }
            auto tmp_ptr = create_dispatchable_data<sensor::laser>(msg);

            {
                dispatch_data_queues[LASER].emplace_back(std::move(tmp_ptr));
                dispatch_counts[LASER]++;
                dispatch_cv.notify_one();
            }
        }
        
        // 【修改】消息类型改为 msg::Imu::ConstSharedPtr
        void add_imu_msg(const sensor_msgs::msg::Imu::ConstSharedPtr &msg)
        {
            std::unique_lock<std::mutex> lock(dispatch_mutex);
            double time = rclcpp::Time(msg->header.stamp).seconds(); // 【修改】时间戳
            if (last_dispatch_time >= time)
            {
                // RCLCPP_WARN(rclcpp::get_logger("dispatch_queue"), "old imu msg.delay: %f s.throw.", last_dispatch_time - time);
                return;
            }
            if (!dispatch_data_queues[IMU].empty() && dispatch_data_queues[IMU].back()->time >= time)
            {
                RCLCPP_WARN(rclcpp::get_logger("dispatch_queue"), "unolder imu msg.delay: %f s.throw.", dispatch_data_queues[IMU].back()->time - time);
                return;
            }
            auto tmp_ptr = create_dispatchable_data<sensor::imu>(msg);
            {
                dispatch_data_queues[IMU].emplace_back(std::move(tmp_ptr));
                dispatch_counts[IMU]++;
                dispatch_cv.notify_one();
            }
        }
        
        // 【修改】消息类型改为 msg::Odometry::ConstSharedPtr
        void add_wheel_odom_msg(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
        {
            std::unique_lock<std::mutex> lock(dispatch_mutex);

            double time = rclcpp::Time(msg->header.stamp).seconds(); // 【修改】时间戳
            if (last_dispatch_time >= time)
            {
                RCLCPP_WARN(rclcpp::get_logger("dispatch_queue"), "old wheel msg.delay: %f s.throw.", last_dispatch_time - time);
                return;
            }
            if (!dispatch_data_queues[WHEEL_ODOM].empty() && dispatch_data_queues[WHEEL_ODOM].back()->time >= time)
            {
                RCLCPP_WARN(rclcpp::get_logger("dispatch_queue"), "unolder wheel_odom msg.delay: %f s.throw.", dispatch_data_queues[WHEEL_ODOM].back()->time - time);
                return;
            }
            auto tmp_ptr = create_dispatchable_data<sensor::wheel_odom>(msg);
            {
                dispatch_data_queues[WHEEL_ODOM].emplace_back(std::move(tmp_ptr));
                dispatch_counts[WHEEL_ODOM]++;
                dispatch_cv.notify_one();
            }
        }
        
        // 【修改】消息类型改为 msg::Image::ConstSharedPtr
        void add_camera_msg(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
        {
            std::unique_lock<std::mutex> lock(dispatch_mutex);
            double time = rclcpp::Time(msg->header.stamp).seconds(); // 【修改】时间戳
            if (last_dispatch_time >= time)
            {
                RCLCPP_WARN(rclcpp::get_logger("dispatch_queue"), "old camera msg.delay: %f s.throw.", last_dispatch_time - time);
                return;
            }
            if (!dispatch_data_queues[CAMERA].empty() && dispatch_data_queues[CAMERA].back()->time >= time)
            {
                RCLCPP_WARN(rclcpp::get_logger("dispatch_queue"), "unolder camera msg.delay: %f s.throw.", dispatch_data_queues[CAMERA].back()->time - time);
                return;
            }
            auto tmp_ptr = create_dispatchable_data<sensor::camera>(msg);
            {
                dispatch_data_queues[CAMERA].emplace_back(std::move(tmp_ptr));
                dispatch_counts[CAMERA]++;
                dispatch_cv.notify_one();
            }
        }
        
        void dispatch()
        {
            while (!quit)
            {
                std::string oldest_key = "none";
                double oldest_time = TIME_MAX;

                std::unique_ptr<lvio_2d::sensor_data> ptr;
                {
                    std::unique_lock<std::mutex> lock(dispatch_mutex);
                    while ((PARAM(enable_camera) && dispatch_counts[CAMERA] < 40) ||
                           (PARAM(enable_laser) && dispatch_counts[LASER] < 40) ||
                           dispatch_counts[WHEEL_ODOM] < 40 || dispatch_counts[IMU] < 40)
                    {
                        if (quit)
                            return;
                        dispatch_cv.wait(lock);
                    }
                    for (const auto &[key, queue] : dispatch_data_queues)
                    {
                        if (!PARAM(enable_laser) && key == LASER)
                            continue;
                        if (!PARAM(enable_camera) && key == CAMERA)
                            continue;
                        if (queue.empty())
                            break;
                        if (queue.front()->time < oldest_time)
                        {
                            oldest_time = queue.front()->time;
                            oldest_key = key;
                        }
                    }
                    if (oldest_key == "none")
                        continue;
                    ptr = std::move(dispatch_data_queues[oldest_key].front());
                    dispatch_data_queues[oldest_key].pop_front();
                    if (oldest_time <= last_dispatch_time)
                        continue;
                    last_dispatch_time = oldest_time;
                    dispatch_counts[oldest_key]--;
                    if (quit)
                        return;

                    auto &record = use_records[oldest_key];
                    record.push_back(ptr->time);
                    if (record.size() > 100)
                        record.pop_front();
                }
                ptr->add_to_trajectory(trajectory_ptr);
            }
        }
    };
} // namespace lvio_2d