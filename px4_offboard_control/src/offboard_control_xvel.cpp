#include "px4_offboard_control/msg/detail/timestamped_array__struct.hpp"
#include <chrono>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_offboard_control/msg/timestamped_array.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <stdint.h>

/*#include <chrono>*/
#include <iostream>
#include <vector>

using namespace std::chrono;
using namespace std::chrono_literals;

class OffboardControlXvel : public rclcpp::Node {
public:
    OffboardControlXvel() : Node("offboard_control_xvel") {

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5),
                               qos_profile);

        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        ekf_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, std::bind(&OffboardControlXvel::ekf_callback_, this, std::placeholders::_1));

        input_matrix_publisher_ = this->create_publisher<px4_offboard_control::msg::TimestampedArray>("/input_matrices", 10);
        sensor_matrix_subscriber_ = this->create_subscription<px4_offboard_control::msg::TimestampedArray>("/sensor_matrices", qos, std::bind(&OffboardControlXvel::update_sensor_matrix_callback_, this, std::placeholders::_1));
        safe_input_subscriber_ = this->create_subscription<px4_offboard_control::msg::TimestampedArray>("/u_safe", qos, std::bind(&OffboardControlXvel::update_safe_control_callback_, this, std::placeholders::_1));
        estimated_states_subscriber_ = this->create_subscription<px4_offboard_control::msg::TimestampedArray>("/estimated_states", qos, std::bind(&OffboardControlXvel::update_estimated_states_callback_, this, std::placeholders::_1));

        ssr_start_subscriber_ = this->create_subscription<std_msgs::msg::Empty>("/start_ssr", qos, std::bind(&OffboardControlXvel::ssr_start_callback_, this, std::placeholders::_1));
        enable_safe_control_subscriber_ = this->create_subscription<std_msgs::msg::Bool>("/safe_control", qos, std::bind(&OffboardControlXvel::enable_safe_control_callback_, this, std::placeholders::_1));
        enable_state_reconstructor_subscriber_ = this->create_subscription<std_msgs::msg::Bool>("/state_reconstruction", qos, std::bind(&OffboardControlXvel::enable_state_reconstruction_callback_, this, std::placeholders::_1));


        parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "state_estimator");
        while (!parameters_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return; 
            }
            RCLCPP_INFO(this->get_logger(),  
                        "service not available, waiting again...");  
        }
        // 4. Get the parameter value
        std::vector<rclcpp::Parameter> parameters = parameters_client_->get_parameters({"system_model/A/dim/row",
                                                                                        "system_model/B/dim/column",
                                                                                        "system_model/C/dim/row"});
        this->n = parameters[0].get_value<int>();
        this->m = parameters[1].get_value<int>();
        this->p = parameters[2].get_value<int>();

        sampling_freq = 20; // in Hertz
        offboard_setpoint_counter_ = 0;
        position = px4_msgs::msg::VehicleLocalPosition();

        std::vector<float> point = {5.7, 5.7};
        coordinates.push_back(point);
        coordinates.emplace_back(std::vector<float>{-5.7, 5.7});
        coordinates.emplace_back(std::vector<float>{-5.7, -5.7});
        coordinates.emplace_back(std::vector<float>{5.7, -5.7});

        start_ssr = false;
        this->safe_control = false;
        this->state_reconstruction = false;
        this->declare_parameter<bool>("safe_control_state", this->safe_control);

        u_safe.push_back(0);
        u_safe.push_back(0);

        target = 0;
        threshold = 0.5;
        dvx = 2 / sampling_freq; // Set your desired bound for vx variation
        dvy = 2 / sampling_freq; // Set your desired bound for vy variation
        prev_vx = 0.0;           // Store previous vx
        prev_vy = 0.0;           // Store previous vy
        new_vx = 0.0;
        new_vy = 0.0;

        auto timer_callback = [this]() -> void {
            if (offboard_setpoint_counter_ == 10) {
                this->publish_vehicle_command(
                    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
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

                new_vx = 0.3 * vx;
                new_vy = 0.3 * vy;

                // Apply bounding on vx variation
                if (std::abs(new_vx - prev_vx) > dvx) {
                    vx = prev_vx +
                        std::copysign(dvx, new_vx - prev_vx); // Bound variation to dvx
                } else {
                    vx = new_vx;
                }

                // Apply bounding on vy variation
                if (std::abs(new_vy - prev_vy) > dvy) {
                    vy = prev_vy +
                        std::copysign(dvy, new_vy - prev_vy); // Bound variation to dvy
                } else {
                    vy = new_vy;
                }

                // Update previous values
                prev_vx = vx;
                prev_vy = vy;

                px4_offboard_control::msg::TimestampedArray input; // Input vector
                std::vector<double> input_vector{static_cast<double>(vx),
                    static_cast<double>(vy)};
                input.array.data = input_vector;
                input_matrix_publisher_->publish(input);

                if (safe_control) {
                    vx = u_safe[0];
                    vy = u_safe[1];
                }
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

    rclcpp::SyncParametersClient::SharedPtr parameters_client_;

    rclcpp::Publisher<px4_offboard_control::msg::TimestampedArray>::SharedPtr input_matrix_publisher_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr ekf_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_safe_control_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_state_reconstructor_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr ssr_start_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr attacked_subscriber_;
    rclcpp::Subscription<px4_offboard_control::msg::TimestampedArray>::SharedPtr sensor_matrix_subscriber_;
    rclcpp::Subscription<px4_offboard_control::msg::TimestampedArray>::SharedPtr safe_input_subscriber_;
    rclcpp::Subscription<px4_offboard_control::msg::TimestampedArray>::SharedPtr estimated_states_subscriber_;

    px4_msgs::msg::VehicleLocalPosition position;

    uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent
    float vx, vy, vz, sampling_freq, threshold, dvx, dvy, prev_vx, prev_vy, new_vx, new_vy;
    int target, n, m, p;
    bool start_ssr, safe_control, state_reconstruction;
    std::vector<std::vector<float>> coordinates;
    px4_offboard_control::msg::TimestampedArray states, estimated_states;
    std::vector<float> u_safe;

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void ekf_callback_(px4_msgs::msg::VehicleLocalPosition msg);
    void attacked_callback(px4_msgs::msg::VehicleLocalPosition msg);
    void ssr_start_callback_(std_msgs::msg::Empty msg);
    void enable_safe_control_callback_(std_msgs::msg::Bool msg);
    void enable_state_reconstruction_callback_(std_msgs::msg::Bool msg);
    void update_safe_control_callback_(px4_offboard_control::msg::TimestampedArray msg);
    void update_sensor_matrix_callback_(px4_offboard_control::msg::TimestampedArray msg);
    void update_estimated_states_callback_(px4_offboard_control::msg::TimestampedArray msg);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
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
 * @brief Callback function for receiving EKF position data.
 * 
 * Function triggered when a new message is received on the 
 * `vehicle_local_position` topic. It stores the received position data 
 * in the `position` member variable.
 * 
 * @param msg The `VehicleLocalPosition` message containing the EKF output.
 */
void OffboardControlXvel::ekf_callback_(px4_msgs::msg::VehicleLocalPosition msg) {
    this->position = msg;
}

/**
 * @brief Callback function to toggle the state reconstruction event.
 * 
 * Function triggered when an empty message is received on the 
 * `/start_ssr` topic. It toggles the `start_ssr` boolean member variable, 
 *  starting or stopping the control loop.
 * 
 * @param msg An empty message (not used).
 */
void OffboardControlXvel::ssr_start_callback_(std_msgs::msg::Empty _) {
    this->start_ssr = !this->start_ssr;
}

/**
 * @brief Callback function to enable/disable safe control mode.
 * 
 * Function triggered when a boolean message is received on the 
 * `enable_safe_control` topic. It updates the `safe_control` member 
 * variable with the received value and also sets the `safe_control_state` 
 * parameter on the parameter server.
 * 
 * @param msg A boolean message indicating whether to enable or disable 
 *            safe control mode.
 */
void OffboardControlXvel::enable_safe_control_callback_(std_msgs::msg::Bool msg) {
    this->safe_control = msg.data;
    this->set_parameter(rclcpp::Parameter("safe_control_state", this->safe_control));
}


/**
 * @brief Merge sensor data or estimated states to determine vehicle position.
 * 
 * This function calculates the vehicle's local position based on either:
 *  - Raw sensor data (if state reconstruction is off)
 *  - Estimated states from the state reconstructor (if state reconstruction is on)
 * 
 * The calculated position is stored in the provided `output` parameter.
 * 
 * @param output A reference to a `VehicleLocalPosition` message where the 
 *               calculated position will be stored.
 */
void OffboardControlXvel::state_merger(px4_msgs::msg::VehicleLocalPosition &output) {
    if (this->state_reconstruction && this->estimated_states.array.data.size()>0) {
        int combinations = static_cast<int>(this->estimated_states.array.data.size() / this->n);
        for (int i = 0; i < combinations; i++){
            output.x += this->estimated_states.array.data[0 + 4*i];
            output.y += this->estimated_states.array.data[2 + 4*i];
        }

        output.x /= combinations;
        output.y /= combinations;
    } else {
        output.x = states.array.data[0] + states.array.data[4];
        output.y = states.array.data[2] + states.array.data[6];

        output.x /= 2;
        output.y /= 2;
    }
}

/**
 * @brief Callback function to update the safe control input.
 * 
 * Function triggered when a new message is received on the
 * `update_safe_control` topic. It extracts the safe control input values from
 * the message and stores them in the `u_safe` array.
 * 
 * @param msg A `TimestampedArray` message containing the safe control input.
 */
void OffboardControlXvel::update_safe_control_callback_(px4_offboard_control::msg::TimestampedArray msg) {
    u_safe[0] = msg.array.data[0];
    u_safe[1] = msg.array.data[1];
}

/**
 * @brief Callback function to update the sensor matrix.
 * 
 * Function triggered when a new message is received on the
 * `/sensor_matrices` topic. It stores the received sensor data in the
 * `states` member variable.
 * 
 * @param msg A `TimestampedArray` message containing the sensor matrix data.
 */
void OffboardControlXvel::update_sensor_matrix_callback_(px4_offboard_control::msg::TimestampedArray msg) {
    this->states = msg;
}

/**
 * @brief Callback function to enable/disable state reconstruction.
 * 
 * Function triggered when a boolean message is received on the
 * `/state_reconstruction` topic. It updates the `state_reconstruction`
 * member variable with the received value.
 * 
 * @param msg A boolean message indicating whether to enable or disable 
 *            state reconstruction.
 */
void OffboardControlXvel::enable_state_reconstruction_callback_(std_msgs::msg::Bool msg) {
    this->state_reconstruction = msg.data;
}

/**
 * @brief Callback function to update the estimated states.
 *
 * Function triggered when a new message is received on the
 * `/estimated_states` topic. It stores the received estimated states in
 * the `estimated_states` member variable.
 *
 * @param msg A `TimestampedArray` message containing the estimated states.
 */
void OffboardControlXvel::update_estimated_states_callback_(px4_offboard_control::msg::TimestampedArray msg) {
    this->estimated_states = msg;
}

int main(int argc, char *argv[]) {
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControlXvel>());

    rclcpp::shutdown();
    return 0;
}
