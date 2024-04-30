#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace std_msgs::msg;
using std::placeholders::_1;

class Attacker : public rclcpp::Node
{
public:
	Attacker() : Node("attacker")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        attack_flag = false;

		attacked_ekf_publisher_ = this->create_publisher<VehicleLocalPosition>("/fmu/out/vehicle_local_position/attacked", qos);
		raw_vehicle_position_subscriber = this->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position/raw", qos, std::bind(&Attacker::position_callback, this, _1));
		attack_trigger_subscriber = this->create_subscription<Bool>("/attack_trigger", qos, std::bind(&Attacker::attack_trigger, this, _1));
	}

private:

    bool attack_flag;

	rclcpp::Publisher<VehicleLocalPosition>::SharedPtr attacked_ekf_publisher_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr raw_vehicle_position_subscriber;
	rclcpp::Subscription<Bool>::SharedPtr attack_trigger_subscriber;

	void position_callback(VehicleLocalPosition position);
	void attack(VehicleLocalPosition& copy);
	void attack_trigger(Bool trigger);
};

void Attacker::attack(VehicleLocalPosition& copy)
{
    if (attack_flag)
    {
        copy.x /= 2;
        copy.y /= 2;
        copy.z /= 2;
    } else {
        copy.x *= 1;
        copy.y *= 1;
        copy.z *= 1;
    }
}

/**
 * @brief Obtain local position of vehicle and set the velocity 
 * @param position	Position output from the EKF
 */
void Attacker::attack_trigger(Bool trigger)
{
    attack_flag = trigger.data;
}

/**
 * @brief Obtain local position of vehicle and set the velocity 
 * @param position	Position output from the EKF
 */
void Attacker::position_callback(VehicleLocalPosition position)
{
    attack(position);

	attacked_ekf_publisher_->publish(position);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting attacker node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Attacker>());

	rclcpp::shutdown();
	return 0;
}


