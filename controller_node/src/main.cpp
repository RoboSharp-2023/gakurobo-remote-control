#include <memory>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <chrono>
#include <array>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

using namespace std::chrono_literals;

class controller_node:public rclcpp::Node {
	sensor_msgs::msg::Joy::SharedPtr                               ctrl_msg;
	rclcpp::TimerBase::SharedPtr                                   pub_tim;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr         ctrl_sub;
	rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr   header_pub;
	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr float_pub;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr        twist_pub;
	rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr   solenoid_pub;

	std_msgs::msg::UInt8MultiArray header_msg;

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

		header_msg.data[2] = 0;
		header_msg.data[3] = 0;

		header_pub->publish(header_msg);
		twist_pub->publish(twist_msg);
		float_pub->publish(float_msg);

		static bool old_button1_state = false;

		std_msgs::msg::UInt8MultiArray solenoid_msg;
		solenoid_msg.data.resize(8);

		solenoid_msg.data[3] = ctrl_msg->buttons[1]? 0x85: 0;

		if(old_button1_state ^ ctrl_msg->buttons[1])
			solenoid_pub->publish(solenoid_msg);
		
		old_button1_state = ctrl_msg->buttons[1];

		union{
			struct{
				uint16_t ID;
				uint16_t cmd;//00:duty 01:current 02:speed 03:position
				float M;
			};
			uint8_t bin[8];
		} reload_msg;

		auto lambda = [](float a) {
			constexpr float Hz = 100, max = 0.1, accel_tim = 0.2;
			static float old_spd = 0;

			auto tmp = a-old_spd;
			tmp = std::min(tmp, 0.02f);
			tmp = std::max(tmp, -0.02f);

			return tmp;
		};

		reload_msg.bin[1] = 7;
		reload_msg.cmd = 2;
		reload_msg.M   = 0.1*ctrl_msg->axes[7];

		std_msgs::msg::UInt8MultiArray reload_std_msg;
		reload_std_msg.data.resize(8);

		for (size_t i = 0 ; reload_std_msg.data.size() < 8 ; i++){
			reload_std_msg.data[i] = reload_msg.bin[i];
		}

		//RCLCPP_INFO(this->get_logger(), "%d, %d, %d ,%d, %d, %d, %d, %d", reload_msg.bin[0], reload_msg.bin[1], reload_msg.bin[2], reload_msg.bin[3], reload_msg.bin[4], reload_msg.bin[5], reload_msg.bin[6], reload_msg.bin[7]);

		// solenoid_pub->publish(reload_std_msg);
	}
public:
	
	controller_node(std::string node_name = "controller_node") : Node(node_name) {
		pub_tim = create_wall_timer(10ms, std::bind(&controller_node::tim_callback, this));
		ctrl_sub = create_subscription<sensor_msgs::msg::Joy>(
			"joy", 10, std::bind(&controller_node::sub_callback, this, std::placeholders::_1));
		header_pub = create_publisher<std_msgs::msg::UInt8MultiArray>("header_msg", 10);
		twist_pub = create_publisher<geometry_msgs::msg::Twist>("tf_spd_msg", 10);
		float_pub = create_publisher<std_msgs::msg::Float32MultiArray>("shot_msg", 10);
		solenoid_pub = create_publisher<std_msgs::msg::UInt8MultiArray>("solenoid_uart_msg", 10);
		

		header_msg.data.resize(4);
		header_msg.data[0] = 'r';
		header_msg.data[1] = 'g';
	}
};


int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<controller_node>());
	rclcpp::shutdown();
	return 0;
}
