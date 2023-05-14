#include <memory>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <chrono>
#include <array>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

using namespace std::chrono_literals;

class controller_node:public rclcpp::Node {
	sensor_msgs::msg::Joy::SharedPtr                               ctrl_msg;
	rclcpp::TimerBase::SharedPtr                                   pub_tim;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr         ctrl_sub;
	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr float_pub;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr        twist_pub;

	void sub_callback(sensor_msgs::msg::Joy::SharedPtr msg) {
		ctrl_msg = msg;
	}

	void tim_callback() {
		if(ctrl_msg == nullptr) {
			return;
		}

		geometry_msgs::msg::Twist twist_msg;
		std_msgs::msg::Float32MultiArray float_msg;
		static uint8_t rotate = 0, state_sqr_button = 0;
		constexpr double coefficient[3] = {0.1, 0.2, 0.5};
		static double sum = 0.0;
		
		rotate += (ctrl_msg->buttons[3] & (ctrl_msg->buttons[3] ^ state_sqr_button)) ? 1 : 0;
		rotate %= 3;
		state_sqr_button = ctrl_msg->buttons[3];
		//need to change
		twist_msg.linear.x = ctrl_msg->axes[0] * coefficient[rotate];
		twist_msg.linear.y = ctrl_msg->axes[1] * coefficient[rotate];
		twist_msg.angular.z = ctrl_msg->axes[3] * coefficient[rotate];

		float_msg.data.resize(2);
		constexpr double shot_coeff = 0.001;
		sum = float_msg.data[0] = (sum + (ctrl_msg->axes[5] - ctrl_msg->axes[2])  * shot_coeff) * (!ctrl_msg->buttons[0] ? 1.0: 0.0);

		static bool state_L1 = false, state_R1 = false;

		sum = float_msg.data[0] = sum + ((ctrl_msg->buttons[4] & (ctrl_msg->buttons[4] ^ state_L1)) ? 0.01: 0.0 )+ ((ctrl_msg->buttons[5] & (ctrl_msg->buttons[5] ^ state_R1)) ? -0.01: 0.0);

		state_L1 = ctrl_msg->buttons[4];
		state_R1 = ctrl_msg->buttons[5];

		if (sum > 1.0) sum = float_msg.data[0] = 1.0;
		if (sum < 0.0) sum = float_msg.data[0] = 0.0;

		float_msg.data[1] = ctrl_msg->buttons[1]? 1.0: 0.0;

		RCLCPP_INFO(this->get_logger(), "%f", float_msg.data[0]);
		twist_pub->publish(twist_msg);
		float_pub->publish(float_msg);
	}
public:
	
	controller_node(std::string node_name = "controller_node") : Node(node_name) {
		pub_tim = create_wall_timer(10ms, std::bind(&controller_node::tim_callback, this));
		ctrl_sub = create_subscription<sensor_msgs::msg::Joy>(
			"joy", 10, std::bind(&controller_node::sub_callback, this, std::placeholders::_1));
		twist_pub = create_publisher<geometry_msgs::msg::Twist>("tf_spd_msg", 10);
		float_pub = create_publisher<std_msgs::msg::Float32MultiArray>("shot_msg", 10);
	}
};


int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<controller_node>());
	rclcpp::shutdown();
	return 0;
}
