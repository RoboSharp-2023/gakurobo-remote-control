#include <memory>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <chrono>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"



const double dt = 0.001;  // 制御周期

class PIDController
	{
	public:
	PIDController() : error_integral_(0.0), prev_error_(0.0) {}

	double computeCommand(double error)
	{
		error_integral_ += error * dt;
		double error_derivative = -(error - prev_error_) / dt;
		prev_error_ = error;
		return Kp * error + Ki * error_integral_ + Kd * error_derivative;
	}

private:
	double error_integral_;
	double prev_error_;
	// PID制御用のパラメータ
	static constexpr double Kp = 1.0;
	static constexpr double Ki = 0.0;
	static constexpr double Kd = 0.0;
};

class TargetMover : public rclcpp::Node
{
public:
  TargetMover() : Node("target_mover"), clk(std::make_shared<rclcpp::Clock>()), tf_buffer_(clk), tf_listener_(tf_buffer_, true)
  {
    now_position_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
        "now_position", 10, std::bind(&TargetMover::nowPositionCallback, this, std::placeholders::_1));

    target_position_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
        "target_position", 10, std::bind(&TargetMover::targetPositionCallback, this, std::placeholders::_1));

    velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("tf_spd_msg", 10);

    pid_controller_.reset(new PIDController());
  }

private:
	void nowPositionCallback(const geometry_msgs::msg::TransformStamped::SharedPtr now_position_msg)
	{
		try {
			// tf2_rosを使用して現在位置と目標位置の差分を計算する
			geometry_msgs::msg::TransformStamped transform_stamped;
			
			transform_stamped = tf_buffer_.lookupTransform(
			  now_position_msg->header.frame_id, target_position_msg_->header.frame_id, rclcpp::Time(0), tf2::durationFromSec(1.0));
			double dx = transform_stamped.transform.translation.x;
			double dy = transform_stamped.transform.translation.y;
			double dist = hypot(dx, dy);

			// PID制御によって目標速度を計算する
			double error = dist;
			double vel_cmd = pid_controller_->computeCommand(error);

			// 速度上限を適用する
			vel_cmd = std::min(vel_cmd, spd_limit);

			// 加速度上限を適用する
			double accel_cmd = (vel_cmd - prev_vel_cmd_) / dt;
			accel_cmd = std::min(accel_cmd, accel_limit);
			accel_cmd = std::max(accel_cmd, -accel_limit);
			vel_cmd = prev_vel_cmd_ + accel_cmd * dt;
			vel_cmd = std::min(vel_cmd, spd_limit);
			vel_cmd = std::max(vel_cmd, -spd_limit);
			prev_vel_cmd_ = vel_cmd;

			// 目標速度を出力する need to improve
			geometry_msgs::msg::Twist vel_msg;
			vel_msg.linear.x = vel_cmd;
			vel_msg.linear.y = 0.0;
			vel_msg.linear.z = 0.0;
			vel_msg.angular.x = 0.0;
			vel_msg.angular.y = 0.0;
			vel_msg.angular.z = 0.0;
			velocity_pub_->publish(vel_msg);

		} catch (tf2::TransformException &ex) {
			RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
			return;
		}
	}

	void targetPositionCallback(const geometry_msgs::msg::TransformStamped::SharedPtr target_position_msg)
	{
		target_position_msg_ = target_position_msg;
		geometry_msgs::msg::Twist t;
	}

	const double spd_limit = 1.0;  // 速度上限 m/s
	const double accel_limit = 1.0;  // 加速度上限 m/s^2

	rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr now_position_sub_;
	rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr target_position_sub_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;

	rclcpp::Clock::SharedPtr clk;
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;
	geometry_msgs::msg::TransformStamped::SharedPtr target_position_msg_;

	std::shared_ptr<PIDController> pid_controller_;
	double prev_vel_cmd_ = 0.0;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TargetMover>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
