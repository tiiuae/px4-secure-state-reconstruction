#include <chrono>
#include <ostream>
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

class SquareVelocityMotion : public rclcpp::Node {
public:
    SquareVelocityMotion() : Node("square_velocity_motion") {

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
            std::bind(&SquareVelocityMotion::ekf_callback, this, _1));

        state = VehicleLocalPosition();

        offboard_setpoint_counter_ = 0;
        velocities = {{5.0, 0.0, 0.0}, {0.0, 5.0, 0.0}, {-5.0, 0.0, 0.0}, {0.0, -5.0, 0.0}};

        target = 0;
        threshold = 0.5;
        hovering = true;
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

            if (hovering) {
                vx = 0 - state.x;
                vy = 0 - state.y;
                vz = -5 - state.z;

                if (abs(vx) > 5) vx = vx / abs(vx) * 5;
                if (abs(vy) > 5) vy = vy / abs(vy) * 5;
                if (abs(vz) > 5) vz = vz / abs(vz) * 5;

                float separation = sqrt(vx * vx + vy * vy + vz * vz);
                if (separation < threshold) hovering = false;

            } else {
                if (++time_counter >= velocity_change_interval) {
                    if (traj_points > 20){
                        vx = 0 - state.x;
                        vy = 0 - state.y;
                        vz = -5 - state.z;
                    } else {
                        time_counter = 0;
                        traj_points++;
                        target = (target + 1) % velocities.size();
                        vx = velocities[target][0];
                        vy = velocities[target][1];
                        vz = velocities[target][2];
                    }
                }
            }

            // offboard_control_mode needs to be paired with trajectory_setpoint
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
    float vx, vy, vz, threshold;
    bool hovering;
    int target, time_counter, velocity_change_interval, traj_points;
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
void SquareVelocityMotion::arm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                            1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void SquareVelocityMotion::disarm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                            0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Send a command to Takeoff the vehicle
 */
void SquareVelocityMotion::takeoff() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0.0);

    RCLCPP_INFO(this->get_logger(), "Takeoff command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void SquareVelocityMotion::publish_offboard_control_mode() {
    OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = true;
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
void SquareVelocityMotion::publish_trajectory_setpoint() {
    TrajectorySetpoint msg{};
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
void SquareVelocityMotion::publish_vehicle_command(uint16_t command, float param1,
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
void SquareVelocityMotion::ekf_callback(VehicleLocalPosition position) {
    state = position;
}

int main(int argc, char *argv[]) {
    std::cout << "Starting square velocity control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquareVelocityMotion>());

    rclcpp::shutdown();
    return 0;
}
