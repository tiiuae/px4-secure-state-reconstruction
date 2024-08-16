#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <stdint.h>

#include <iostream>
#include <random>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace std_msgs::msg;
using std::placeholders::_1;

class Attacker : public rclcpp::Node {
public:
    Attacker() : Node("attacker") {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5),
                               qos_profile);
        attack_flag = false;

        attacked_ekf_publisher_ = this->create_publisher<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position/attacked", qos);
        raw_vehicle_position_subscriber =
            this->create_subscription<VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position", qos,
                std::bind(&Attacker::position_callback, this, _1));
        attack_trigger_subscriber = this->create_subscription<std_msgs::msg::Empty>(
            /*"/attack_trigger",*/
            "/start_ssr", qos, std::bind(&Attacker::attack_trigger, this, _1));
        dist = std::normal_distribution<double>(mean, stddev);
    }

private:
    bool attack_flag;

    rclcpp::Publisher<VehicleLocalPosition>::SharedPtr attacked_ekf_publisher_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr
    raw_vehicle_position_subscriber;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr
    attack_trigger_subscriber;

    void position_callback(VehicleLocalPosition position);
    void attack(VehicleLocalPosition &copy);
    void attack_trigger(std_msgs::msg::Empty);

    const double mean = 0.0;
    const double stddev = 1.0;
    std::default_random_engine generator;
    std::normal_distribution<double> dist;
};

void Attacker::attack(VehicleLocalPosition &copy) {
    if (attack_flag) {
        copy.x *= 1;
        copy.vx *= -1;
        copy.y *= 1;
        copy.vy *= (1 + dist(generator));
        /*copy.y = 0;*/
        /*copy.z = 0;*/
    } else {
        copy.x *= 1;
        copy.vx *= 1;
        copy.y *= 1;
        copy.vy *= 1;
        /*copy.y *= 1;*/
        /*copy.z *= 1;*/
    }
}

/**
 * @brief Obtain local position of vehicle and set the velocity
 * @param position	Position output from the EKF
 */
void Attacker::attack_trigger(std_msgs::msg::Empty) {
    attack_flag = !attack_flag;
}

/**
 * @brief Obtain local position of vehicle and set the velocity
 * @param position	Position output from the EKF
 */
void Attacker::position_callback(VehicleLocalPosition position) {
    attack(position);

    attacked_ekf_publisher_->publish(position);
}

int main(int argc, char *argv[]) {
    std::cout << "Starting attacker node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Attacker>());

    rclcpp::shutdown();
    return 0;
}
