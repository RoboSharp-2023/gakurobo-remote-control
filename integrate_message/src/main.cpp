#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <memory>
#include <algorithm>
#include <functional>
#include <chrono>
#include <string>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;


class integrate_msg: public rclcpp::Node {
	rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr header_sub;
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr omni_sub;
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr shot_sub;
	rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr uint_sub;

	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr float_pub;
	rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr uart_pub;

	rclcpp::TimerBase::SharedPtr float_pub_tim;
	rclcpp::TimerBase::SharedPtr uart_pub_tim;

	std_msgs::msg::Float32MultiArray float_pub_msg;
	std_msgs::msg::UInt8MultiArray uart_msg;
	static constexpr size_t float_pub_msg_size = 6;
	static constexpr size_t uart_msg_size = sizeof(uint8_t)*4 + sizeof(float)*6/sizeof(uint8_t);

	void header_sub_callback(std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
		for(int i = 0; i < msg->data.size(); i++) 
			uart_msg.data[i] = msg->data[i];
	}

	void omni_sub_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
		for (size_t i = 0; i < msg->data.size(); i++) float_pub_msg.data[i] = msg->data[i];
	}

	void shot_sub_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
		float_pub_msg.data[4] = msg->data[0];
		float_pub_msg.data[5] = msg->data[1];
	}

	void uint_sub_callback (std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
		for(size_t i = 0; i < msg->data.size(); i++) uart_msg.data[i+4] = msg->data[i];
	}

	void float_tim_callback() {
		float_pub->publish(float_pub_msg);
	}

	void uart_tim_callback() {
		uart_pub->publish(uart_msg);
	}

public:

	integrate_msg(std::string node_name = "integrate_msg") :Node(node_name) {
		header_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
			"header_msg", 10, std::bind(&integrate_msg::header_sub_callback, this, std::placeholders::_1));
		omni_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
			"omni_msg", 10, std::bind(&integrate_msg::omni_sub_callback, this, std::placeholders::_1));
		shot_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
			"shot_msg", 10, std::bind(&integrate_msg::shot_sub_callback, this, std::placeholders::_1));
		uint_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
			"uint_msg", 10, std::bind(&integrate_msg::uint_sub_callback, this, std::placeholders::_1));
		
		float_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("float_msg", 10);
		uart_pub  = this->create_publisher<std_msgs::msg::UInt8MultiArray>("uart_msg", 10);    

		float_pub_tim  = this->create_wall_timer(10ms, std::bind(&integrate_msg::float_tim_callback, this));
		uart_pub_tim   = this->create_wall_timer(10ms, std::bind(&integrate_msg::uart_tim_callback, this));

		float_pub_msg.data.resize(float_pub_msg_size);
		uart_msg.data.resize(uart_msg_size);
	}
};

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<integrate_msg>());
	rclcpp::shutdown();

	return 0;
}
