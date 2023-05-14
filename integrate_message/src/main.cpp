#include <rclcpp/rclcpp.hpp>
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
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr omni_sub;
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr shot_sub;
	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub;
	rclcpp::TimerBase::SharedPtr pub_tim;
	std_msgs::msg::Float32MultiArray pub_msg;
	static constexpr size_t pub_msg_size = 6;

	void omni_sub_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
		for (size_t i = 0; i < msg->data.size(); i++) pub_msg.data[i] = msg->data[i];
	}

	void shot_sub_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
		pub_msg.data[4] = msg->data[0];
		pub_msg.data[5] = msg->data[1];
	}

	void tim_callback() {
		pub->publish(pub_msg);
	}

public:

	integrate_msg(std::string node_name = "integrate_msg") :Node(node_name) {
		omni_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
			"omni_msg", 10, std::bind(&integrate_msg::omni_sub_callback, this, std::placeholders::_1));
		shot_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
			"shot_msg", 10, std::bind(&integrate_msg::shot_sub_callback, this, std::placeholders::_1));
		pub      = this->create_publisher<std_msgs::msg::Float32MultiArray>("spd_msg", 10);
		pub_tim  = this->create_wall_timer(10ms, std::bind(&integrate_msg::tim_callback, this));

		pub_msg.data.resize(pub_msg_size);
	}
};

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<integrate_msg>());
	rclcpp::shutdown();

	return 0;
}
