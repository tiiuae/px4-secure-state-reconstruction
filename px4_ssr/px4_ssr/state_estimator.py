import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile
import numpy as np

# from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from px4_ssr.drone_system import DroneSystem


class StateEstimator(Node):
    """
    StateEstimator Node for managing drone missions in a ROS2 environment.
    Handles the generation, storage, and execution of waypoints for drone navigation.
    """

    def __init__(self):
        super().__init__("state_estimator", parameter_overrides=[])

        self.qos_profile = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5)

        self.drone = DroneSystem()
        self.y_vec = []

        self.sensor_subscriber = self.create_subscription(
            Float64MultiArray,
            "/sensor_matrix",
            self.secure_state_estimator,
            self.qos_profile,
        )

    def secure_state_estimator(self, msg: Float64MultiArray):
        self.update_sensor_matrix(msg)
        if len(self.y_vec) < self.drone.n:
            return

        Xt_cell = None
        Gamma_set_new = None

        return Xt_cell, Gamma_set_new

    def update_sensor_matrix(self, msg: Float64MultiArray):
        self.y_vec.append(msg.data)
        if len(self.y_vec) > self.drone.n:
            self.y_vec = self.y_vec[1:]


def main(args=None):
    print("Starting state_estimator node...")
    rclpy.init(args=args)

    state_estimator = StateEstimator()

    try:
        rclpy.spin(state_estimator)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
