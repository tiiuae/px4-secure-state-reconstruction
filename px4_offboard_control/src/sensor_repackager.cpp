#include <std_msgs/msg/float64_multi_array.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <iostream>

class SensorRepackager : public rclcpp::Node
{
public:
	SensorRepackager() : Node("sensor_repackager")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		sensor_matrix_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/sensor_matrix", qos);

		ekf_subscriber_1_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position/raw/1", qos, std::bind(&SensorRepackager::ekf_callback_1, this, std::placeholders::_1));
		ekf_subscriber_2_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position/raw/2", qos, std::bind(&SensorRepackager::ekf_callback_2, this, std::placeholders::_1));

		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(10),
			std::bind(&SensorRepackager::timerCallback, this));
	}

private:
	px4_msgs::msg::VehicleLocalPosition ekf_1, ekf_2;

	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr sensor_matrix_publisher_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr ekf_subscriber_1_, ekf_subscriber_2_;

	void ekf_callback_1(px4_msgs::msg::VehicleLocalPosition position);
	void ekf_callback_2(px4_msgs::msg::VehicleLocalPosition position);

	void timerCallback();
	rclcpp::TimerBase::SharedPtr timer_;
};

/**
 * @brief Repackage data from sensors into a sensor matrix and publish at predefined interval
*/
void SensorRepackager::timerCallback()
{
	std_msgs::msg::Float64MultiArray output;
	std::vector<double> sensor_vector{static_cast<double>(ekf_1.x), static_cast<double>(ekf_1.vx), static_cast<double>(ekf_2.x), static_cast<double>(ekf_2.vx)};
	output.data = sensor_vector;
	sensor_matrix_publisher_->publish(output);
}

/**
 * @brief Obtain and store sensor output from one of the EKFs
 * @param position	Position output from the EKF
 */
void SensorRepackager::ekf_callback_1(px4_msgs::msg::VehicleLocalPosition position)
{
	this->ekf_1 = position;
}

/**
 * @brief Obtain and store sensor output from one of the EKFs
 * @param position	Position output from the EKF
 */
void SensorRepackager::ekf_callback_2(px4_msgs::msg::VehicleLocalPosition position)
{
	this->ekf_2 = position;
}

int main(int argc, char *argv[])
{
	std::cout << "Starting sensor_repackager node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SensorRepackager>());

	rclcpp::shutdown();
	return 0;
}
