#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_offboard_control/msg/timestamped_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <stdint.h>

#include <iostream>
#include <string>

class SensorMatrixDownsampler : public rclcpp::Node {
public:
    SensorMatrixDownsampler() : Node("sensor_matrix_downsampler") {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5),
                               qos_profile);

        sensor_matrix_publisher_ =
            this->create_publisher<px4_offboard_control::msg::TimestampedArray>(
                "/sensor_matrices", 10);

        ekf_subscriber_1_ =
            this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position", qos,
                std::bind(&SensorMatrixDownsampler::ekf_callback_1, this,
                          std::placeholders::_1));
        ekf_subscriber_2_ =
            this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position/attacked", qos,
                std::bind(&SensorMatrixDownsampler::ekf_callback_2, this,
                          std::placeholders::_1));

        sampling_freq = 20; // In Hertz

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<long>(1 / sampling_freq * 1000)),
            std::bind(&SensorMatrixDownsampler::timerCallback, this));
    }

private:
    px4_msgs::msg::VehicleLocalPosition ekf_1, ekf_2;
    float sampling_freq;

    rclcpp::Publisher<px4_offboard_control::msg::TimestampedArray>::SharedPtr
    sensor_matrix_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    ekf_subscriber_1_,
    ekf_subscriber_2_;

    void ekf_callback_1(px4_msgs::msg::VehicleLocalPosition position);
    void ekf_callback_2(px4_msgs::msg::VehicleLocalPosition position);

    void timerCallback();
    rclcpp::TimerBase::SharedPtr timer_;
};

/*
 * @brief Repackage data from sensors into a sensor matrix and publish at
 * predefined interval
 */
void SensorMatrixDownsampler::timerCallback() {
    /*std_msgs::msg::Float64MultiArray output;*/
    px4_offboard_control::msg::TimestampedArray output;
    std::vector<double> sensor_vector{
        static_cast<double>(ekf_1.x), static_cast<double>(ekf_1.vx),
        static_cast<double>(ekf_1.y), static_cast<double>(ekf_1.vy),
        static_cast<double>(ekf_2.x), static_cast<double>(ekf_2.vx),
        static_cast<double>(ekf_2.y), static_cast<double>(ekf_2.vy)};
    /*output.data = sensor_vector;*/
    output.header.stamp.sec = static_cast<int>(ekf_1.timestamp / 1000000);
    output.header.stamp.nanosec = static_cast<unsigned int>(
        static_cast<unsigned long>(ekf_1.timestamp) * 1000 -
        static_cast<unsigned long>(output.header.stamp.sec) * 1000000000);
    output.array.data = sensor_vector;
    sensor_matrix_publisher_->publish(output);
}

/**
 * @brief Obtain and store sensor output from one of the EKFs
 * @param position	Position output from the EKF
 */
void SensorMatrixDownsampler::ekf_callback_1(
    px4_msgs::msg::VehicleLocalPosition position) {
    this->ekf_1 = position;
}

/**
 * @brief Obtain and store sensor output from one of the EKFs
 * @param position	Position output from the EKF
 */
void SensorMatrixDownsampler::ekf_callback_2(
    px4_msgs::msg::VehicleLocalPosition position) {
    this->ekf_2 = position;
}

int main(int argc, char *argv[]) {
    std::cout << "Starting sensor_matrix_downsampler node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorMatrixDownsampler>());

    rclcpp::shutdown();
    return 0;
}
