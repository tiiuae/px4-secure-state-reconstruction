import numpy as np
import rclpy
from px4_offboard_control.msg import TimestampedArray
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile
from std_msgs.msg import Empty

from px4_ssr.drone_system import (
    TS,
    SafeProblem,
    SecureStateReconstruct,
    SSProblem,
    SystemModel,
)


class XProb:
    def __init__(self, x0, A_gamma, gamma):
        self.x0 = x0
        self.A_gamma = A_gamma
        self.gamma = gamma


class SafeController(Node):
    """
    StateEstimator Node for reconstructing state given sensor readings.
    """

    def __init__(self):
        super().__init__("state_estimator", parameter_overrides=[])

        self.qos_profile = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5)
        system_model = SystemModel()

        # self.s = self.drone.p - 1
        self.dtsys_a, self.dtsys_b, self.dtsys_c, self.dtsys_d = (
            SSProblem.convert_ct_to_dt(
                system_model.Ac, system_model.Bc, system_model.Cc, system_model.Dc, TS
            )
        )
        self.n: int = self.dtsys_a.shape[0]
        self.m: int = self.dtsys_c.shape[0]

        self.s = 2
        self.y_vec = []
        self.u_vec = []
        self.x_array = []
        self.gamma_set = []
        self.start_ssr = False

        h = np.vstack([np.identity(self.n),-np.identity(self.n)])
        q = 10*np.ones((2*self.n,1))
        gamma = 0.8

        self.safe_problem = SafeProblem(self.dtsys_a, self.dtsys_b, h, q, gamma)

        self.ssr_subscriber = self.create_subscription(
            Empty,
            "/start_ssr",
            self.start_ssr_subscriber,
            self.qos_profile,
        )
        self.sensor_subscriber = self.create_subscription(
            TimestampedArray,
            "/sensor_matrices",
            self.update_sensor_matrix,
            10,
        )
        self.estimated_states_subscriber = self.create_subscription(
            TimestampedArray,
            "/estimated_states",
            self.secure_safe_controller,
            10,
        )
        self.input_subscriber = self.create_subscription(
            TimestampedArray,
            "/input_matrices",
            self.update_input_matrix,
            10,
        )
        self.safe_controls_publisher = self.create_publisher(
            TimestampedArray, "/u_safe", 10
        )

    def start_ssr_subscriber(self, msg: Empty):
        self.start_ssr = not self.start_ssr

    def secure_safe_controller(self, msg: TimestampedArray):
        # self.update_estimated_states(msg)
        self.x_est = np.array(msg.array.data)

        if not self.start_ssr:
            return
        # Prerequisite is to have enough past readings
        # if len(self.y_vec) < self.drone.n:
        if len(self.y_vec) < self.n or len(self.u_vec) < self.n:
            return
        # Reshape flattened
        self.x_est = np.reshape(self.x_est, (self.n, -1))
        self.safe_problem.u_seq = np.array(self.u_vec)
        # For the moment treat u_nom = u_safe
        # i.e. the safety_filter is running in "open-loop"
        # TODO: Differentiate between u_nom and u_safe
        ####### The SSR takes in u_safe not u_nom
        u_safe, lic, flag = self.safe_problem.cal_safe_control(np.array(self.u_vec[-1]), self.x_est.T)

        print("---")
        print("u_safe: ", u_safe)
        print("u_nom: ", self.u_vec[-1])
        print("flag: ", flag)
        print("---")
        u_safe_out = TimestampedArray()
        u_safe_out.header = msg.header
        u_safe_out.array.data = list(u_safe)

        self.safe_controls_publisher.publish(u_safe_out)

    def update_sensor_matrix(self, msg: TimestampedArray):
        # [[x0, y0, z0], ... , [xn-1, yn-1, zn-1]]
        self.y_vec.append(list(msg.array.data))

        # Keep only the last n readings
        # if len(self.y_vec) > self.drone.n:
        if len(self.y_vec) > self.n:
            self.y_vec = self.y_vec[1:]

    def update_input_matrix(self, msg: TimestampedArray):
        # [[u0], [u1], ... , [un-1]]
        self.u_vec.append(list(msg.array.data))

        # Keep only the last n readings
        # if len(self.u_vec) > self.drone.n:
        if len(self.u_vec) > self.n:
            self.u_vec = self.u_vec[1:]

    def update_estimated_states(self, msg: TimestampedArray):
        self.x_array.append(list(msg.array.data))

        # Keep only the last n readings
        # if len(self.x_vec) > self.drone.n:
        if len(self.x_array) > self.n:
            self.x_array = self.x_array[1:]

def main(args=None):
    print("Starting safe_controller node...")
    rclpy.init(args=args)

    state_estimator = SafeController()

    try:
        rclpy.spin(state_estimator)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
