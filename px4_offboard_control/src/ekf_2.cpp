#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <iostream>

class EKF2 : public rclcpp::Node
{
public:
	EKF2() : Node("ekf2")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		gt_ekf_publisher_ = this->create_publisher<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position/gt", qos);
		raw_ekf_publisher_ = this->create_publisher<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position/raw", qos);
		vehicle_position_subscriber = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, std::bind(&EKF2::position_callback, this, std::placeholders::_1));
	}

private:

	rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr gt_ekf_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr raw_ekf_publisher_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_position_subscriber;

	void position_callback(px4_msgs::msg::VehicleLocalPosition position);
};

/**
 * @brief Obtain local position of vehicle and set the velocity 
 * @param position	Position output from the EKF
 */
void EKF2::position_callback(px4_msgs::msg::VehicleLocalPosition position)
{
	px4_msgs::msg::VehicleLocalPosition ekf2(position);

	gt_ekf_publisher_->publish(ekf2);
	raw_ekf_publisher_->publish(ekf2);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting ekf2 node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<EKF2>());

	rclcpp::shutdown();
	return 0;
}

