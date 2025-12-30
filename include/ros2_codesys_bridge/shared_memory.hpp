#ifndef ROS2_CODESYS_BRIDGE__SHARED_MEMORY_HPP_
#define ROS2_CODESYS_BRIDGE__SHARED_MEMORY_HPP_

#include <cstdint>
#include <string>

namespace ros2_codesys_bridge
{
    // 共享内存数据结构
    struct SharedMemoryData
    {
        // 来自ROS的速度指令
        float cmd_vx;       // 线速度x (m/s)
        float cmd_vy;       // 线速度y (m/s)
        float cmd_yaw;      // 角速度 (rad/s)
        uint64_t cmd_timestamp;  // 指令时间戳

        // 来自CODESYS的里程计数据
        float odom_x;       // x坐标 (m)
        float odom_y;       // y坐标 (m)
        float odom_yaw;     // 偏航角 (rad)
        float odom_vx;      // 线速度x (m/s)
        float odom_vy;      // 线速度y (m/s)
        float odom_vth;     // 角速度 (rad/s)
        uint64_t odom_timestamp;  // 里程计时间戳

        // 心跳检测
        uint64_t heartbeat;  // 心跳计数器
    };

    class SharedMemory
    {
    public:
        SharedMemory(const std::string &name, size_t size);
        ~SharedMemory();

        bool is_valid() const { return valid_; }
        SharedMemoryData* get_data() { return data_; }
        void lock();
        void unlock();

    private:
        int shm_fd_;
        void *ptr_;
        SharedMemoryData *data_;
        bool valid_;
        std::string name_;
    };
}  // namespace ros2_codesys_bridge

#endif  // ROS2_CODESYS_BRIDGE__SHARED_MEMORY_HPP_