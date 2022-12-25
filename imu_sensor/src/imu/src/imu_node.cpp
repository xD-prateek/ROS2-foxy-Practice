#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msg/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"

class imu_data : public rclcpp::Node
{
public:
	imu_data()
		: Node("imu_pub")
	{
		publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("topic", 10);
		timer_ = this->create_wall_timer(100ms, std::bind(&imu_data::timer_callback, this));
	}
private:
	//we set the imu sensor data to the respective messages here
	void timer_callback() {

	}
};

int main(int argc, char const *argv[])
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<imu_data>();
	node->declare_parameter("i2c_address");
	node->declare_parameter("frame_id");
	node->declare_parameter("poll");
	node->declare_parameter("topicImu");

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}