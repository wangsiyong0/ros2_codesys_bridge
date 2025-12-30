#ifndef ROS2_CODESYS_BRIDGE__BRIDGE_NODE_HPP_
#define ROS2_CODESYS_BRIDGE__BRIDGE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "ros2_codesys_bridge/shared_memory.hpp"

namespace ros2_codesys_bridge
{
	class BridgeNode : public rclcpp::Node
	{
	public:
		BridgeNode();
		~BridgeNode() override;

	private:
		void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
		void timer_callback();
		bool is_cmd_vel_changed(const geometry_msgs::msg::Twist &new_cmd);

		// ROS组件
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
		std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub_;
		std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
		rclcpp::TimerBase::SharedPtr timer_;

		// 共享内存
		std::unique_ptr<SharedMemory> shared_memory_;

		// 状态变量
		geometry_msgs::msg::Twist last_cmd_vel_;
		uint64_t heartbeat_counter_;
		std::string base_frame_;
		std::string odom_frame_;
	};
} // namespace ros2_codesys_bridge

#endif // ROS2_CODESYS_BRIDGE__BRIDGE_NODE_HPP_