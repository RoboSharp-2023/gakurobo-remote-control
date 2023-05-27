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
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

using namespace std::chrono_literals;

class controller_node:public rclcpp::Node {
	sensor_msgs::msg::Joy::SharedPtr                               ctrl_msg;
	rclcpp::TimerBase::SharedPtr                                   pub_tim;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr         ctrl_sub;
	rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr   header_pub;
	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr shot_pub;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr        twist_pub;
	rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr   solenoid_pub;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr        set_sum_sub;

	std_msgs::msg::UInt8MultiArray header_msg;

	double sum = 0.0;

	void sub_callback(sensor_msgs::msg::Joy::SharedPtr msg) {
		ctrl_msg = msg;
	}

	void tim_callback() {
		if(ctrl_msg == nullptr) {
			return;
		}

		geometry_msgs::msg::Twist twist_msg;
		std_msgs::msg::Float32MultiArray shot_msg;
		static uint8_t rotate = 0, state_sqr_button = 0;
		constexpr double coefficient[3] = {0.1, 0.2, 0.5};
		
		
		rotate += (ctrl_msg->buttons[3] & (ctrl_msg->buttons[3] ^ state_sqr_button)) ? 1 : 0;
		rotate %= 3;
		state_sqr_button = ctrl_msg->buttons[3];
		//need to change
		twist_msg.linear.x = ctrl_msg->axes[0] * coefficient[rotate];
		twist_msg.linear.y = ctrl_msg->axes[1] * coefficient[rotate];
		twist_msg.angular.z = ctrl_msg->axes[3] * coefficient[rotate];

		shot_msg.data.resize(2);
		constexpr double shot_coeff = 0.001;
		sum = shot_msg.data[0] = (sum + (ctrl_msg->axes[5] - ctrl_msg->axes[2])  * shot_coeff) * (!ctrl_msg->buttons[0] ? 1.0: 0.0);

		static bool state_L1 = false, state_R1 = false;

		sum = shot_msg.data[0] = sum + ((ctrl_msg->buttons[4] & (ctrl_msg->buttons[4] ^ state_L1)) ? 0.01: 0.0 )+ ((ctrl_msg->buttons[5] & (ctrl_msg->buttons[5] ^ state_R1)) ? -0.01: 0.0);

		state_L1 = ctrl_msg->buttons[4];
		state_R1 = ctrl_msg->buttons[5];

		if (sum > 0.940) sum = shot_msg.data[0] = 0.94;
		if (sum < 0.0) sum = shot_msg.data[0] = 0.0;

		header_msg.data[2] = 0;
		header_msg.data[3] = 0;

		header_pub->publish(header_msg);
		twist_pub->publish(twist_msg);
		shot_pub->publish(shot_msg);

		static bool old_button1_state = false;

		union{
			struct{
				uint8_t type[2];
				uint8_t ID;
				uint8_t cmd;
				float M;
			};
			uint8_t bin[8];
		} uni_solenoid;

		static auto solenoid_counter = 0u;
		solenoid_counter += ((old_button1_state ^ ctrl_msg->buttons[1])& ctrl_msg->buttons[1])? 1: 0;
		solenoid_counter %= 3;
		std::array <uint16_t, 3> cmd = { 0x04, 0x00, 0x84 }; //todo: I need to ask Matsuura for the sequence order of the solenoids;  bit  7: shot , 2: reload , 0: collect
		uni_solenoid.type[0] = 23;
		uni_solenoid.type[1] = 1;
		uni_solenoid.ID = 1;
		uni_solenoid.cmd = cmd[solenoid_counter];
		uni_solenoid.M = 0.0;

		bool state = false;


		static bool old_state_triangle = false;
		static uint8_t old_cmd = false;

		uni_solenoid.cmd |= old_cmd ^= (state = ((ctrl_msg->buttons[2] ^ old_state_triangle) & ctrl_msg->buttons[2])) ? 0x01: 0x00;
		old_state_triangle = ctrl_msg->buttons[2];
		

		std_msgs::msg::UInt8MultiArray solenoid_msg;
		solenoid_msg.data.resize(8);

		for(int i = 0; i < 8; i++) 
			solenoid_msg.data[i] = uni_solenoid.bin[i];
		

		if(uni_solenoid.cmd & 0x04 && uni_solenoid.cmd & 0x01) uni_solenoid.cmd &= ~0x01;
		if((old_button1_state ^ ctrl_msg->buttons[1] )& ctrl_msg->buttons[1] || state )
			solenoid_pub->publish(solenoid_msg);
		
		old_button1_state = ctrl_msg->buttons[1];



		auto lambda = [](float a) {
			constexpr float Hz = 100, max = 0.1, accel_tim = 0.2;
			static float old_spd = 0;

			auto tmp = a-old_spd;
			tmp = std::min(tmp, 0.02f);
			tmp = std::max(tmp, -0.02f);

			return tmp;
		};

		union{
			struct{
				uint16_t ID;
				uint16_t cmd;//00:duty 01:current 02:speed 03:position
				float M;
			};
			uint8_t bin[8];
		}reload_msg;

		reload_msg.ID = 0x10;
		reload_msg.cmd = 2;

		bool tmp = 0;

		if(ctrl_msg->axes[7] == 0) reload_msg.bin[5] = 0;
		else if(ctrl_msg->axes[7] < 0) reload_msg.bin[5] = 1;
		else if(ctrl_msg->axes[7] > 0) reload_msg.bin[5] = -1;

		std_msgs::msg::UInt8MultiArray reload_std_msg;
		reload_std_msg.data.resize(8);

		for (size_t i = 0 ; i < 8 ; i++){
			reload_std_msg.data[i] = reload_msg.bin[i];
		}

		 solenoid_pub->publish(reload_std_msg);
	}

	void set_sum_callback(std_msgs::msg::Float32::SharedPtr msg) {
		sum = msg->data;
	}
	
public:
	
	controller_node(std::string node_name = "controller_node") : Node(node_name) {
		pub_tim = create_wall_timer(10ms, std::bind(&controller_node::tim_callback, this));
		ctrl_sub = create_subscription<sensor_msgs::msg::Joy>(
			"joy", 10, std::bind(&controller_node::sub_callback, this, std::placeholders::_1));
		header_pub = create_publisher<std_msgs::msg::UInt8MultiArray>("header_msg", 10);
		twist_pub = create_publisher<geometry_msgs::msg::Twist>("tf_spd_msg", 10);
		shot_pub = create_publisher<std_msgs::msg::Float32MultiArray>("shot_msg", 10);
		solenoid_pub = create_publisher<std_msgs::msg::UInt8MultiArray>("solenoid_uart_msg", 10);
		set_sum_sub  = create_subscription<std_msgs::msg::Float32>(
			"set_sum", 10, std::bind(&controller_node::set_sum_callback, this, std::placeholders::_1));
		

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
