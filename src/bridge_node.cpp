#include "ros2_codesys_bridge/bridge_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <chrono>

using namespace std::chrono_literals;

namespace ros2_codesys_bridge
{
    BridgeNode::BridgeNode() : Node("ros2_codesys_bridge")
    {
        // 声明参数
        this->declare_parameter("base_frame", "base_footprint");
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("update_rate", 50.0);

        // 获取参数
        base_frame_ = this->get_parameter("base_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        double update_rate = this->get_parameter("update_rate").as_double();

        // 初始化共享内存
        shared_memory_ = std::make_unique<SharedMemory>("/ros2_codesys_bridge", sizeof(SharedMemoryData));
        if (!shared_memory_->is_valid())
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize shared memory");
            rclcpp::shutdown();
        }

        // 初始化ROS组件
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&BridgeNode::cmd_vel_callback, this, std::placeholders::_1));	

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 初始化定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000 / update_rate)),
            std::bind(&BridgeNode::timer_callback, this));

        // 初始化状态变量
        heartbeat_counter_ = 0;
        last_cmd_vel_.linear.x = 0.0;
        last_cmd_vel_.linear.y = 0.0;
        last_cmd_vel_.angular.z = 0.0;

        RCLCPP_INFO(this->get_logger(), "ros2_codesys_bridge initialized");
    }

    BridgeNode::~BridgeNode()
    {
        RCLCPP_INFO(this->get_logger(), "ros2_codesys_bridge shutdown");
    }

    void BridgeNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 打印cmd_vel消息
        RCLCPP_INFO_STREAM(this->get_logger(), 
            "Received cmd_vel: vx=" << msg->linear.x << ", vy=" << msg->linear.y << ", yaw=" << msg->angular.z);

        // 检查是否有变化
        if (is_cmd_vel_changed(*msg))
        {
            shared_memory_->lock();
            auto data = shared_memory_->get_data();
            
            // 更新速度指令
            data->cmd_vx = msg->linear.x;
            data->cmd_vy = msg->linear.y;
            data->cmd_yaw = msg->angular.z;
            data->cmd_timestamp = this->now().nanoseconds();
            
            // 更新心跳
            data->heartbeat = ++heartbeat_counter_;
            
            shared_memory_->unlock();
            last_cmd_vel_ = *msg;
        }
    }

    void BridgeNode::timer_callback()
    {
        // 从共享内存读取里程计数据并发布
        shared_memory_->lock();
        auto data = shared_memory_->get_data();
        
        // 发布里程计消息
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = base_frame_;

        // 位置信息
        odom_msg.pose.pose.position.x = data->odom_x;
        odom_msg.pose.pose.position.y = data->odom_y;
        odom_msg.pose.pose.position.z = 0.0;

        // 姿态信息(四元数)
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, data->odom_yaw);
        odom_msg.pose.pose.orientation.x = quat.x();
        odom_msg.pose.pose.orientation.y = quat.y();
        odom_msg.pose.pose.orientation.z = quat.z();
        odom_msg.pose.pose.orientation.w = quat.w();

        // 速度信息
        odom_msg.twist.twist.linear.x = data->odom_vx;
        odom_msg.twist.twist.linear.y = data->odom_vy;
        odom_msg.twist.twist.angular.z = data->odom_vth;

        odom_pub_->publish(odom_msg);

        // 发布TF变换
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = odom_frame_;
        transform.child_frame_id = base_frame_;
        transform.transform.translation.x = data->odom_x;
        transform.transform.translation.y = data->odom_y;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation = odom_msg.pose.pose.orientation;

        tf_broadcaster_->sendTransform(transform);

        shared_memory_->unlock();
    }

    bool BridgeNode::is_cmd_vel_changed(const geometry_msgs::msg::Twist &new_cmd)
    {
        // 检查速度是否有显著变化
        const double eps = 1e-6;
        return (std::fabs(new_cmd.linear.x - last_cmd_vel_.linear.x) > eps) ||
               (std::fabs(new_cmd.linear.y - last_cmd_vel_.linear.y) > eps) ||
               (std::fabs(new_cmd.angular.z - last_cmd_vel_.angular.z) > eps);
    }
}  // namespace ros2_codesys_bridge

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ros2_codesys_bridge::BridgeNode>());
    rclcpp::shutdown();
    return 0;
}