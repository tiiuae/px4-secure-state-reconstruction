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

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		gt_subscriber = this->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position/gt", qos, std::bind(&OffboardControl::gt_callback, this, _1));
		attacked_subscriber = this->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position/attacked", qos, std::bind(&OffboardControl::attacked_callback, this, _1));

		states.push_back(VehicleLocalPosition());
		states.push_back(VehicleLocalPosition());

		offboard_setpoint_counter_ = 0;
		std::vector<float> point = {5., 5., -5.};
		coordinates.push_back(point);
		coordinates.emplace_back(std::vector<float>{-5., 5., -5.});
		coordinates.emplace_back(std::vector<float>{-5., -5., -5.});
		coordinates.emplace_back(std::vector<float>{5., -5., -5.});

		target = 0;
		threshold = 0.5;

		auto timer_callback = [this]() -> void
		{
			if (offboard_setpoint_counter_ == 10)
			{
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				this->arm();
			}

			VehicleLocalPosition reference;
			this->state_merger(reference); // <-- Standard averaging

			vx = coordinates[target][0] - reference.x;
			vy = coordinates[target][1] - reference.y;
			vz = coordinates[target][2] - reference.z;

			if (abs(vx) > 5)
			{
				vx = vx / abs(vx) * 5; // Input in x
			}
			if (abs(vy) > 5)
			{
				vy = vy / abs(vy) * 5; // Input in y
			}
			if (abs(vz) > 5)
			{
				vz = vz / abs(vz) * 5; // Input in z
			}

			float vxs = vx * vx;
			float vys = vy * vy;
			float vzs = vz * vz;
			float separation = (float)sqrt(vxs + vys + vzs);

			if (separation < threshold)
			{
				target++;
				if (target > (int)coordinates.size() - 1)
				{
					target = 0;
				}
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();

			if (offboard_setpoint_counter_ < 11)
			{
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr gt_subscriber;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr attacked_subscriber;

	/*std::atomic<uint64_t> timestamp_; //!< common synced timestamped*/

	uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent
	float vx, vy, vz, threshold;
	int target;
	std::vector<std::vector<float>> coordinates;
	std::vector<VehicleLocalPosition> states;

	void gt_callback(VehicleLocalPosition position);
	void attacked_callback(VehicleLocalPosition position);
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void state_merger(VehicleLocalPosition& output);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
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
void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	msg.position = {NAN, NAN, NAN};
	msg.velocity = {vx, vy, vz};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
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
void OffboardControl::gt_callback(VehicleLocalPosition position)
{
	states[0] = position;
}

/**
 * @brief Obtain local position of vehicle at attacked sensor
 * @param position	Position output from the EKF
 */
void OffboardControl::attacked_callback(VehicleLocalPosition position)
{
	states[1] = position;
}

/**
 * @brief Obtain local position of vehicle at attacked sensor
 * @param position	Position output from the EKF
 */
void OffboardControl::state_merger(VehicleLocalPosition& output)
{
	int length = states.size();
	for (VehicleLocalPosition& entry: states)
	{
		output.x += entry.x;
		output.y += entry.y;
		output.z += entry.z;
	}
	output.x /= length;
	output.y /= length;
	output.z /= length;
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
