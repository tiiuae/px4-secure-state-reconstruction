import rclpy
# from rclpy.qos import QoSProfile, DurabilityPolicy
# from rclpy.duration import Duration
from rclpy.node import Node
from drone_system import DroneSystem


class MissionManager(Node):
    """
    MissionManager Node for managing drone missions in a ROS2 environment.
    Handles the generation, storage, and execution of waypoints for drone navigation.
    """

    def __init__(self):
        super().__init__("mission_manager", parameter_overrides=[])

        drone = DroneSystem()


def main(args=None):
    print("Starting mission_manager node...")
    rclpy.init(args=args)

    mission_manager = MissionManager()

    try:
        rclpy.spin(mission_manager)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
