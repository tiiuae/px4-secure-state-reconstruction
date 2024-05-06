#include <iostream>
#include <string>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

std::vector<double> pose;

class GZ_EKF : public rclcpp::Node
{
public:
	GZ_EKF() : Node("gz_ekf")
	{
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		ref_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/gz_pose/reference", 10);
		diff_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/gz_pose/difference", 10);

		vehicle_position_subscriber = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, std::bind(&GZ_EKF::position_callback, this, std::placeholders::_1));

	}

private:
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr ref_pub;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr diff_pub;

	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_position_subscriber;
    gz::transport::Node node;
    geometry_msgs::msg::PointStamped gz_reference;
    geometry_msgs::msg::PointStamped pose_diff;

	void position_callback(px4_msgs::msg::VehicleLocalPosition position);
};

/**
 * @brief Obtain local position of vehicle and set the velocity 
 * @param position	Position output from the EKF
 */
void GZ_EKF::position_callback(px4_msgs::msg::VehicleLocalPosition position)
{
	px4_msgs::msg::VehicleLocalPosition ekf2(position);
	gz_reference.point.x = pose.at(0);
    gz_reference.point.y = pose.at(1);
    gz_reference.point.z = -1*pose.at(2);
    gz_reference.header.stamp.sec = position.timestamp/1000000;
    
    pose_diff.header = gz_reference.header;
    pose_diff.point.x = (position.x - gz_reference.point.x);
    pose_diff.point.y = (position.y - gz_reference.point.y);
    pose_diff.point.z = (position.z - gz_reference.point.z);

    ref_pub->publish(gz_reference);
    diff_pub->publish(pose_diff);
}


void gz_position_cb(const gz::msgs::Pose_V &_msg)
{
    pose[1] = _msg.pose(0).position().x();
    pose[0] = _msg.pose(0).position().y();
    pose[2] = _msg.pose(0).position().z();
}

int main(int argc, char *argv[])
{
	std::cout << "Starting ekf2 node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
    
    gz::transport::Node node;
    std::string topic = "/world/default/dynamic_pose/info";
    pose.push_back(0.);
    pose.push_back(0.);
    pose.push_back(0.);
    
    if (node.Subscribe(topic, gz_position_cb))
    {
        std::cerr << "Error subscribing to topic [" << topic << "]" << std::endl;
    }

	rclcpp::spin(std::make_shared<GZ_EKF>());

	rclcpp::shutdown();
	return 0;
}


