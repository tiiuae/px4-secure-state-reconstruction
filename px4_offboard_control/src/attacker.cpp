#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
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
        this->load_parameters_from_yaml();

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5),
                               qos_profile);
        this->attack_flag = false;
        this->declare_parameter<bool>("attacker_state", this->attack_flag);

        attacked_ekf_publisher_ = this->create_publisher<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position/attacked", qos);
        raw_vehicle_position_subscriber =
            this->create_subscription<VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position", qos,
                std::bind(&Attacker::position_callback, this, _1));
        attack_trigger_subscriber = this->create_subscription<std_msgs::msg::Bool>(
            "/attack_trigger", qos, std::bind(&Attacker::attack_trigger, this, _1));
    }

private:
    bool attack_flag;

    rclcpp::Publisher<VehicleLocalPosition>::SharedPtr attacked_ekf_publisher_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr
    raw_vehicle_position_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    attack_trigger_subscriber;

    void position_callback(VehicleLocalPosition position);
    void attack(VehicleLocalPosition &copy);
    void attack_trigger(std_msgs::msg::Bool);
    void load_parameters_from_yaml();

    float attack_state(std::string attack_type, float value, double attack_param, std::normal_distribution<double> dist);

    std::default_random_engine generator;
    std::normal_distribution<double> x_dist, y_dist, vx_dist, vy_dist;

    bool attack_x, attack_y, attack_vx, attack_vy;
    std::string attack_type_x, attack_type_y, attack_type_vx, attack_type_vy;
    float attack_param_x, attack_param_y, attack_param_vx, attack_param_vy;
};

void Attacker::attack(VehicleLocalPosition &copy) {
    if (attack_flag) {
        copy.x = (this->attack_x) ? this->attack_state(this->attack_type_x, copy.x, this->attack_param_x, this->x_dist) : copy.x;
        copy.vx = (this->attack_vx) ? this->attack_state(this->attack_type_vx, copy.vx, this->attack_param_vx, this->vx_dist) : copy.vx;
        copy.y = (this->attack_y) ? this->attack_state(this->attack_type_y, copy.y, this->attack_param_y, this->y_dist) : copy.y;
        copy.vy = (this->attack_vy) ? this->attack_state(this->attack_type_vy, copy.vx, this->attack_param_vy, this->vy_dist) : copy.vy;
    } else {
        copy.x *= 1;
        copy.vx *= 1;
        copy.y *= 1;
        copy.vy *= 1;
    }
}

/**
 * @brief Obtain local position of vehicle and set the velocity
 * @param position	Position output from the EKF
 */
void Attacker::attack_trigger(std_msgs::msg::Bool msg) {
    attack_flag = msg.data;
    this->set_parameter(rclcpp::Parameter("attacker_state", this->attack_flag));
}

/**
 * @brief Obtain local position of vehicle and set the velocity
 * @param position	Position output from the EKF
 */
void Attacker::position_callback(VehicleLocalPosition position) {
    attack(position);

    attacked_ekf_publisher_->publish(position);
}

float Attacker::attack_state(std::string attack_type, float value, double attack_param, std::normal_distribution<double> dist){
    if (attack_type == "shift") {
        value = static_cast<float>(value + attack_param);
    } else if (attack_type == "scale") {
        value = static_cast<float>(value * attack_param);
    } else if (attack_type == "const") {
        value = static_cast<float>(attack_param);
    } else if (attack_type == "noise") {
        value = static_cast<float>(value * (1 + dist(generator)));
    }
    return value;
}

void Attacker::load_parameters_from_yaml(){
    this->declare_parameter("x.attack", rclcpp::PARAMETER_BOOL);
    this->declare_parameter("x.attack_type", rclcpp::PARAMETER_STRING);
    this->declare_parameter("x.attack_param", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("y.attack", rclcpp::PARAMETER_BOOL);
    this->declare_parameter("y.attack_type", rclcpp::PARAMETER_STRING);
    this->declare_parameter("y.attack_param", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("vx.attack", rclcpp::PARAMETER_BOOL);
    this->declare_parameter("vx.attack_type", rclcpp::PARAMETER_STRING);
    this->declare_parameter("vx.attack_param", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("vy.attack", rclcpp::PARAMETER_BOOL);
    this->declare_parameter("vy.attack_type", rclcpp::PARAMETER_STRING);
    this->declare_parameter("vy.attack_param", rclcpp::PARAMETER_DOUBLE);

    std::vector<rclcpp::Parameter> parameters =this->get_parameters({
        "x.attack",
        "x.attack_type",
        "x.attack_param",
        "y.attack",
        "y.attack_type",
        "y.attack_param",
        "vx.attack",
        "vx.attack_type",
        "vx.attack_param",
        "vy.attack",
        "vy.attack_type",
        "vy.attack_param",
    });

    this->attack_x = parameters[0].get_value<bool>();
    this->attack_type_x = parameters[1].get_value<std::string>();
    this->attack_param_x = parameters[2].get_value<float>();
    this->attack_y = parameters[3].get_value<bool>();
    this->attack_type_y = parameters[4].get_value<std::string>();
    this->attack_param_y = parameters[5].get_value<float>();
    this->attack_vx = parameters[6].get_value<bool>();
    this->attack_type_vx = parameters[7].get_value<std::string>();
    this->attack_param_vx = parameters[8].get_value<float>();
    this->attack_vy = parameters[9].get_value<bool>();
    this->attack_type_vy = parameters[10].get_value<std::string>();
    this->attack_param_vy = parameters[11].get_value<float>();

    RCLCPP_INFO(this->get_logger(), "Attack on:");
    if (this->attack_x) {
        if (this->attack_type_x == "noise") {
            this->x_dist = std::normal_distribution<double>(0, this->attack_param_x);
        }
        RCLCPP_INFO(this->get_logger(), "  x: ");
        RCLCPP_INFO(this->get_logger(), "    type: %s", this->attack_type_x.c_str());
        RCLCPP_INFO(this->get_logger(), "    param: %f", this->attack_param_x);
    }

    if (this->attack_y) {
        if (this->attack_type_y == "noise") {
            this->y_dist = std::normal_distribution<double>(0, this->attack_param_y);
        }
        RCLCPP_INFO(this->get_logger(), "  y: ");
        RCLCPP_INFO(this->get_logger(), "    type: %s", this->attack_type_y.c_str());
        RCLCPP_INFO(this->get_logger(), "    param: %f", this->attack_param_y);
    }

    if (this->attack_vx) {
        if (this->attack_type_vx == "noise") {
            this->vx_dist = std::normal_distribution<double>(0, this->attack_param_vx);
        }
        RCLCPP_INFO(this->get_logger(), "  vx: ");
        RCLCPP_INFO(this->get_logger(), "    type: %s", this->attack_type_vx.c_str());
        RCLCPP_INFO(this->get_logger(), "    param: %f", this->attack_param_vx);
    }

    if (this->attack_vy) {
        if (this->attack_type_vy == "noise") {
            this->vy_dist = std::normal_distribution<double>(0, this->attack_param_vy);
        }
        RCLCPP_INFO(this->get_logger(), "  vy: ");
        RCLCPP_INFO(this->get_logger(), "    type: %s", this->attack_type_vy.c_str());
        RCLCPP_INFO(this->get_logger(), "    param: %f", this->attack_param_vy);
    }
}


int main(int argc, char *argv[]) {
    std::cout << "Starting attacker node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Attacker>());

    rclcpp::shutdown();
    return 0;
}
