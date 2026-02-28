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
    // 注意这里的传感器名字，实际上就是各个缓冲队列的名字
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

    // 传感器融合的入口，系统为 LiDAR、IMU、Wheel Odom 和 Camera 分别维护了独立的缓存队列。每一个传感器有单独的add
    // 把异步、凌乱、多频的传感器数据，整理成一股严格按时间戳升序排列的单线程数据流，然后喂给后端的算法（Trajectory）处理。
    // 注意这个函数的构造函数，构造函数启动了一个线程，位于本文件的末尾
    class dispatch_queue
    {
    private:
        // 这是一个字典，里面存储了不同传感器的测量数据，索引未传感器的标识。有一个计数器
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
            double time = rclcpp::Time(msg->header.stamp).seconds();  
            // 时间还变小了？
            if (last_dispatch_time >= time)
            {
                // RCLCPP_WARN(rclcpp::get_logger("dispatch_queue"), "old imu msg.delay: %f s.throw.", last_dispatch_time - time);
                return;
            }
            // IMU队列中的最后一个元素的时间戳比较
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
        
        /*
        这是整个系统跳动的心脏。它是一个运行在独立子线程（dispatch_thread）中的死循环。在多传感器 SLAM（尤其是带有高频 IMU 的系统）中，数据必须绝对按时间先后顺序处理。
        例如：IMU（200Hz）、里程计（50Hz）、LiDAR（10Hz）。我们不能因为 LiDAR 频率低就先处理 LiDAR，必须先把 LiDAR 时间戳之前的所有 IMU 和 Odom 数据全积分完，才能对这帧 LiDAR 进行去畸变和状态更新。
        */
        // void dispatch()
        // {
        //     while (!quit)
        //     {
        //         std::string oldest_key = "none";
        //         double oldest_time = TIME_MAX;

        //         std::unique_ptr<lvio_2d::sensor_data> ptr;
        //         {
        //             std::unique_lock<std::mutex> lock(dispatch_mutex);
        //             // 每一个传感器需要填充，否则等待
        //             while ((PARAM(enable_camera) && dispatch_counts[CAMERA] < 2) ||
        //                    (PARAM(enable_laser) && dispatch_counts[LASER] < 2) ||
        //                    dispatch_counts[WHEEL_ODOM] < 10 || dispatch_counts[IMU] < 40)
        //             {
        //                 if (quit)
        //                     return;
        //                 dispatch_cv.wait(lock);
        //             }
        //             // 寻找最古老的数据 —— 绝对时序排序 (Sorting)，更新 oldest_time 和 oldest_key
        //             for (const auto &[key, queue] : dispatch_data_queues)
        //             {
        //                 if (!PARAM(enable_laser) && key == LASER)
        //                     continue;
        //                 if (!PARAM(enable_camera) && key == CAMERA)
        //                     continue;
        //                 if (queue.empty())
        //                     break;
        //                 if (queue.front()->time < oldest_time)
        //                 {
        //                     oldest_time = queue.front()->time;
        //                     oldest_key = key;
        //                 }
        //             }
        //             if (oldest_key == "none")
        //                 continue;
        //             // 出队与时间防倒流机制 (Dequeue & Validation)
        //             ptr = std::move(dispatch_data_queues[oldest_key].front());
        //             dispatch_data_queues[oldest_key].pop_front();
        //             if (oldest_time <= last_dispatch_time)
        //                 continue;
        //             last_dispatch_time = oldest_time;
        //             dispatch_counts[oldest_key]--;
        //             if (quit)
        //                 return;

        //             auto &record = use_records[oldest_key];
        //             record.push_back(ptr->time);
        //             if (record.size() > 100)
        //                 record.pop_front();
        //         }

        //         /* 将数据ptr丢给 odom 模块
        //         接下来，系统调用 ptr->add_to_trajectory。因为 ptr 是一个使用了多态的基类指针，这句话会自动根据数据的真实类型（比如它是激光），去调用 trajectory::add_sensor_data(laser_ptr) 执行极其复杂的特征提取、匹配和 Ceres 优化。
        //         */
        //         ptr->add_to_trajectory(trajectory_ptr);
        //     }
        // }


        void dispatch()
        {
            while (!quit)
            {
                std::unique_ptr<lvio_2d::sensor_data> ptr;
                std::string oldest_key = "none";
                double oldest_time = TIME_MAX;

                {
                    std::unique_lock<std::mutex> lock(dispatch_mutex);
                    
                    bool ready_to_pop = false;
                    bool force_dispatch = false;

                    // --- 工业级：基于时间戳对齐与超时的等待循环 ---
                    while (!quit)
                    {
                        // 1. 检查是否有传感器完全没有数据 (冷启动或掉线)
                        bool has_empty = false;
                        if (PARAM(enable_camera) && dispatch_data_queues[CAMERA].empty()) has_empty = true;
                        if (PARAM(enable_laser) && dispatch_data_queues[LASER].empty()) has_empty = true;
                        if (dispatch_data_queues[WHEEL_ODOM].empty()) has_empty = true;
                        if (dispatch_data_queues[IMU].empty()) has_empty = true;

                        if (!has_empty)
                        {
                            // 2. 找到所有队列中最老的那帧数据的时间戳
                            oldest_time = TIME_MAX;
                            if (PARAM(enable_camera)) oldest_time = std::min(oldest_time, dispatch_data_queues[CAMERA].front()->time);
                            if (PARAM(enable_laser))  oldest_time = std::min(oldest_time, dispatch_data_queues[LASER].front()->time);
                            oldest_time = std::min(oldest_time, dispatch_data_queues[WHEEL_ODOM].front()->time);
                            oldest_time = std::min(oldest_time, dispatch_data_queues[IMU].front()->time);

                            // 3. 核心时间对齐逻辑：检查所有传感器的【最新数据(back)】是否都盖过了这个 oldest_time
                            // 如果有传感器的最新数据还没到达 oldest_time，说明它可能在路上，必须等它！
                            bool all_covered = true;
                            if (PARAM(enable_camera) && dispatch_data_queues[CAMERA].back()->time < oldest_time) all_covered = false;
                            if (PARAM(enable_laser)  && dispatch_data_queues[LASER].back()->time < oldest_time) all_covered = false;
                            if (dispatch_data_queues[WHEEL_ODOM].back()->time < oldest_time) all_covered = false;
                            if (dispatch_data_queues[IMU].back()->time < oldest_time) all_covered = false;

                            if (all_covered) {
                                ready_to_pop = true;
                                break; // 完美满足对齐条件，立刻跳出等待！(零延迟)
                            }
                        }

                        // 4. 不满足对齐条件，进入带超时的等待 (看门狗：最大等待 50 毫秒)
                        // 50ms 足够覆盖目前绝大多数 10Hz/20Hz 传感器的帧间抖动了
                        auto status = dispatch_cv.wait_for(lock, std::chrono::milliseconds(50));
                        
                        if (status == std::cv_status::timeout)
                        {
                            // 发生超时！说明某个传感器可能掉线、驱动崩溃或网络拥堵极高
                            // 工业级做法：强制打破同步，找出当前可用队列中最老的数据处理掉，防止 OOM (内存溢出) 和系统死锁
                            force_dispatch = true;
                            break;
                        }
                    }

                    if (quit) return;

                    // --- 寻找内存中真实存在的最老数据 ---
                    oldest_time = TIME_MAX;
                    for (const auto &[key, queue] : dispatch_data_queues)
                    {
                        if (!PARAM(enable_laser) && key == LASER) continue;
                        if (!PARAM(enable_camera) && key == CAMERA) continue;
                        if (queue.empty()) continue; // 注意：如果是强制放行，某些故障传感器的队列可能是空的

                        if (queue.front()->time < oldest_time)
                        {
                            oldest_time = queue.front()->time;
                            oldest_key = key;
                        }
                    }

                    if (oldest_key == "none") continue;

                    // 如果是因为超时触发了强制放行，打印警告以便开发人员排查硬件故障
                    if (force_dispatch) {
                        static int warn_cnt = 0;
                        if (warn_cnt++ % 100 == 0) { // 降频打印，防止刷屏
                            std::cout << "\033[1;33m[WARN] Sensor timeout or delayed! Forcing dispatch to prevent OOM. Processing: " 
                                      << oldest_key << ".\033[0m\n";
                        }
                    }

                    // --- 出队与状态更新 ---
                    ptr = std::move(dispatch_data_queues[oldest_key].front());
                    dispatch_data_queues[oldest_key].pop_front();
                    dispatch_counts[oldest_key]--;
                    
                    if (oldest_time <= last_dispatch_time)
                        continue; // 坚决丢弃时光倒流的脏数据
                        
                    last_dispatch_time = oldest_time;

                    auto &record = use_records[oldest_key];
                    record.push_back(ptr->time);
                    if (record.size() > 100)
                        record.pop_front();
                }

                /* 解锁后，将数据ptr丢给 odom 模块执行复杂的算法计算，实际仍然调用的是trajectory_ptr中的接口 */
                ptr->add_to_trajectory(trajectory_ptr);
            }
        }
    };
} // namespace lvio_2d