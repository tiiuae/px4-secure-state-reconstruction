#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;

class EKF2 : public rclcpp::Node
{
public:
	EKF2() : Node("ekf2")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		ekf_copy_publisher_ = this->create_publisher<VehicleLocalPosition>("/fmu/out/vehicle_local_position_2", qos);
		vehicle_position_subscriber = this->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, std::bind(&EKF2::position_callback, this, _1));
	}

private:

	rclcpp::Publisher<VehicleLocalPosition>::SharedPtr ekf_copy_publisher_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_position_subscriber;

	void position_callback(VehicleLocalPosition position);
	void attack(VehicleLocalPosition& copy);
};

void EKF2::attack(VehicleLocalPosition& copy)
{
	copy.x /= 2;
	copy.y /= 2;
	copy.z /= 2;
}

/**
 * @brief Obtain local position of vehicle and set the velocity 
 * @param position	Position output from the EKF
 */
void EKF2::position_callback(VehicleLocalPosition position)
{
	VehicleLocalPosition ekf2(position);

	attack(ekf2);

	ekf_copy_publisher_->publish(ekf2);
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

