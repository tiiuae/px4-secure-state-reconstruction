#include <chrono>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_offboard_control/msg/timestamped_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <stdint.h>

/*#include <chrono>*/
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;

class OffboardControlXvel : public rclcpp::Node {
public:
    OffboardControlXvel() : Node("offboard_control_xvel") {

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        input_matrix_publisher_ = this->create_publisher<px4_offboard_control::msg::TimestampedArray>("/input_matrices", 10);

        ekf_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, std::bind(&OffboardControlXvel::ekf_callback_, this, std::placeholders::_1));
        attacked_subscriber = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position/attacked", qos, std::bind(&OffboardControlXvel::attacked_callback, this, std::placeholders::_1));
        ssr_start_subscriber_ = this->create_subscription<std_msgs::msg::Empty>("/start_ssr", qos, std::bind(&OffboardControlXvel::ssr_start_callback_, this, std::placeholders::_1));

        states.push_back(px4_msgs::msg::VehicleLocalPosition());
        states.push_back(px4_msgs::msg::VehicleLocalPosition());

        sampling_freq = 20; // in Hertz
        offboard_setpoint_counter_ = 0;
        position = px4_msgs::msg::VehicleLocalPosition();

        std::vector<float> point = {5., 5.};
        coordinates.push_back(point);
        coordinates.emplace_back(std::vector<float>{-5., 5.});
        coordinates.emplace_back(std::vector<float>{-5., -5.});
        coordinates.emplace_back(std::vector<float>{5., -5.});

        start_ssr = false;

        /*time_counter = 0;*/
        /*Ts = 2.5;*/

        target = 0;
        threshold = 0.5;

        auto timer_callback = [this]() -> void {
            if (offboard_setpoint_counter_ == 10) {
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                this->arm();
            }

            if (!start_ssr) {
                vx = 0 - position.x;
                vy = 0 - position.y;
                vz = -5 - position.z;
                if (abs(vx) > 5) {
                    vx = vx / abs(vx) * 5; // Input in x
                }
                if (abs(vy) > 5) {
                    vy = vy / abs(vy) * 5; // Input in y
                }
                if (abs(vz) > 5) {
                    vz = vz / abs(vz) * 5; // Input in z
                }
            } else {
                px4_msgs::msg::VehicleLocalPosition reference;
                this->state_merger(reference); // <-- Standard averaging

                vx = coordinates[target][0] - reference.x;
                vy = coordinates[target][1] - reference.y;
                vz = -5 - reference.z;

                if (abs(vx) > 5) {
                    vx = vx / abs(vx) * 5; // Input in x
                }
                if (abs(vy) > 5) {
                    vy = vy / abs(vy) * 5; // Input in y
                }
                if (abs(vz) > 5) {
                    vz = vz / abs(vz) * 5; // Input in z
                }

                float vxs = vx * vx;
                float vys = vy * vy;
                float vzs = vz * vz;
                float separation = (float)sqrt(vxs + vys + vzs);

                if (separation < threshold) {
                    target++;
                    if (target > (int)coordinates.size() - 1) {
                        target = 0;
                    }
                }
                /*vx = 1.5 * std::sin(time_counter / Ts);*/
                /*vy = 1.5 * std::cos(time_counter / Ts);*/
                /*vx = 1.5 * std::sin(time_counter / Ts);*/
                /*vy = 1.5 * std::cos(time_counter / Ts);*/
                /*vz = -5 - position.z;*/
                /*if (abs(vz) > 5) {*/
                /*    vz = vz / abs(vz) * 5; // Input in z*/
                /*}*/

                /*time_counter += 1 / sampling_freq;*/
                px4_offboard_control::msg::TimestampedArray input; // Input vector
                std::vector<double> input_vector{static_cast<double>(vx), static_cast<double>(vy)};
                input.array.data = input_vector;
                input_matrix_publisher_->publish(input);
            }

            // offboard_control_mode needs to be paired with trajectory_setpoint
            publish_offboard_control_mode();
            publish_trajectory_setpoint();

            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }
        };
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<long>(1 / sampling_freq * 1000)),
            timer_callback);
    }

    void arm();
    void disarm();

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<px4_offboard_control::msg::TimestampedArray>::SharedPtr input_matrix_publisher_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr ekf_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr ssr_start_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr attacked_subscriber;

    px4_msgs::msg::VehicleLocalPosition position;
    uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent
    float vx, vy, vz, sampling_freq, threshold;
    /*float time_counter, Ts;*/
    int target;
    bool start_ssr;
    std::vector<std::vector<float>> coordinates;
    std::vector<px4_msgs::msg::VehicleLocalPosition> states;

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void ekf_callback_(px4_msgs::msg::VehicleLocalPosition msg);
    void attacked_callback(px4_msgs::msg::VehicleLocalPosition msg);
    void ssr_start_callback_(std_msgs::msg::Empty msg);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0,
                                 float param2 = 0.0);
    void state_merger(px4_msgs::msg::VehicleLocalPosition &output);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControlXvel::arm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControlXvel::disarm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControlXvel::publish_offboard_control_mode() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControlXvel::publish_trajectory_setpoint() {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {NAN, NAN, NAN};
    msg.velocity = {vx, vy, vz};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD
 * codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControlXvel::publish_vehicle_command(uint16_t command,
                                                  float param1, float param2) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

/**
 * @brief Obtain and store sensor output from one of the EKFs
 * @param msg Position output from the EKF
 */
void OffboardControlXvel::ekf_callback_(
    px4_msgs::msg::VehicleLocalPosition msg) {
    this->position = msg;
    states[0] = msg;
}

/**
 * @brief Obtain and shift the state of state reconstruction event input
 * @param msg Empty trigger
 */
void OffboardControlXvel::ssr_start_callback_(std_msgs::msg::Empty msg) {
    this->start_ssr = !this->start_ssr;
}

int main(int argc, char *argv[]) {
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControlXvel>());

    rclcpp::shutdown();
    return 0;
}

/**
 * @brief Obtain local position of vehicle at attacked sensor
 * @param position	Position output from the EKF
 */
void OffboardControlXvel::state_merger(px4_msgs::msg::VehicleLocalPosition &output) {
    int length = states.size();
    for (px4_msgs::msg::VehicleLocalPosition &entry : states) {
        output.x += entry.x;
        output.y += entry.y;
        output.z += entry.z;
    }
    output.x /= length;
    output.y /= length;
    output.z /= length;
}

/**
 * @brief Obtain local position of vehicle at attacked sensor
 * @param position	Position output from the EKF
 */
void OffboardControlXvel::attacked_callback(px4_msgs::msg::VehicleLocalPosition position) {
    states[1] = position;
}
