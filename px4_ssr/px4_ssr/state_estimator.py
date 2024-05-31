import rclpy
# from rclpy.qos import QoSProfile, DurabilityPolicy
# from rclpy.duration import Duration
from rclpy.node import Node
from px4_ssr.drone_system import DroneSystem


class StateEstimator(Node):
    """
    StateEstimator Node for managing drone missions in a ROS2 environment.
    Handles the generation, storage, and execution of waypoints for drone navigation.
    """

    def __init__(self):
        super().__init__("state_estimator", parameter_overrides=[])

        self.drone = DroneSystem()
        # print(self.drone.__dict__)
        print(self.drone.F_cell, "\n", self.drone.O_cell)

    # def secure_state_estimator():
    #
    #     return Xt_cell, Gamma_set_new

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
