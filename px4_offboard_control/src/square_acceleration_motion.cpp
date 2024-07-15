#include <chrono>
#include <ostream>
#include <cmath>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

/*#include <chrono>*/
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;

class SquareAccelerationMotion : public rclcpp::Node {
public:
    SquareAccelerationMotion() : Node("square_acceleration_motion") {

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5),
                               qos_profile);

        offboard_control_mode_publisher_ =
            this->create_publisher<OffboardControlMode>(
                "/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ =
            this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        ekf_subscriber = this->create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos,
            std::bind(&SquareAccelerationMotion::ekf_callback, this, _1));

        state = VehicleLocalPosition();

        offboard_setpoint_counter_ = 0;
        velocities = {{5.0, 0.0, 0.0}, {0.0, 5.0, 0.0}, {-5.0, 0.0, 0.0}, {0.0, -5.0, 0.0}};

        target = 0;
        threshold = 0.5;
        hovering = false;
        timer_period = 20ms;
        velocity_change_interval = 125;
        time_counter = 0;
        traj_points = 0;

        auto timer_callback = [this]() -> void {
            if (offboard_setpoint_counter_ == 10) {
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                                              1, 6);
                this->arm();
            }

            if (!hovering) {
                // Use a sinusoidal function to update az for initial hovering
                az = -2 * (std::sin(time_counter - M_PI/2) +1 );
                if (time_counter > 2 * M_PI) {
                    hovering = true;
                }
                ax = 0.0f;
                ay = 0.0f;

                time_counter += 0.02; // Adjust time increment as needed

            } else {
                ax = 0.00000000000;
                ay = 0.00000000000;
                az = 0.00000000000;
            }
            //
            publish_offboard_control_mode();
            publish_trajectory_setpoint();


            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }
        };
        timer_ = this->create_wall_timer(timer_period, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr
    offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr
    trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr ekf_subscriber;


    uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent
    float ax, ay, az, threshold;
    bool hovering;
    int target, velocity_change_interval, traj_points;
    float time_counter;
    std::vector<std::vector<float>> velocities;
    VehicleLocalPosition state;
    std::chrono::milliseconds timer_period;

    void arm();
    void disarm();
    void takeoff();
    void ekf_callback(VehicleLocalPosition position);
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0,
                                 float param2 = 0.0);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void SquareAccelerationMotion::arm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                            1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void SquareAccelerationMotion::disarm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                            0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Send a command to Takeoff the vehicle
 */
void SquareAccelerationMotion::takeoff() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0.0);

    RCLCPP_INFO(this->get_logger(), "Takeoff command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void SquareAccelerationMotion::publish_offboard_control_mode() {
    OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = true;
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
void SquareAccelerationMotion::publish_trajectory_setpoint() {
    TrajectorySetpoint msg{};
    msg.position = {NAN, NAN, NAN};
    msg.velocity = {NAN, NAN, NAN};
    msg.acceleration = {ax, ay, az};
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
void SquareAccelerationMotion::publish_vehicle_command(uint16_t command, float param1,
                                              float param2) {
    VehicleCommand msg{};
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
 * @brief Obtain local position of vehicle at ground truth sensor
 * @param position	Position output from the EKF
 */
void SquareAccelerationMotion::ekf_callback(VehicleLocalPosition position) {
    state = position;
}

int main(int argc, char *argv[]) {
    std::cout << "Starting square acceleration control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquareAccelerationMotion>());

    rclcpp::shutdown();
    return 0;
}

