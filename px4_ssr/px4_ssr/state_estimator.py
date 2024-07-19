import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile
import numpy as np
import scipy as sc
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Empty
from px4_ssr.drone_system import DroneSystem, SSProblem, SecureStateReconstruct, EPS, TS, nchoosek

class XProb:
    
    def __init__(self, x0, A_gamma, gamma):
        self.x0 = x0
        self.A_gamma = A_gamma
        self.gamma = gamma

class StateEstimator(Node):
    """
    StateEstimator Node for reconstructing state given sensor readings.
    """

    def __init__(self):
        super().__init__("state_estimator", parameter_overrides=[])

        self.qos_profile = QoSProfile(history = HistoryPolicy.KEEP_LAST, depth = 5)
        Ac = np.matrix(
            [
                [0, 1],  # x
                [0, 0],  # xdot
            ]
        )
        Bc = np.matrix(
            [
                [1],
                [0],
            ]
        )
        Cc = np.matrix(
                [
                    [1, 0],  # x1
                    [0, 1],  # xdot1
                    [1, 0],  # x2
                    [0, 1],  # xdot2
                ]
            )
        Dc = np.zeros((Cc.shape[0],Bc.shape[1]))

        # self.drone = DroneSystem()
        # self.s = self.drone.p - 1
        self.dtsys_a, self.dtsys_b, self.dtsys_c, self.dtsys_d = SSProblem.convert_ct_to_dt(Ac, Bc, Cc, Dc, TS)
        self.n: int = self.dtsys_a.shape[0]

        self.s = 0
        self.y_vec = []
        self.u_vec = []
        self.gamma_set = []
        self.start_ssr = False

        self.ssr_subscriber = self.create_subscription(
            Empty,
            "/start_ssr",
            self.start_ssr_subscriber,
            self.qos_profile,
        )
        self.sensor_subscriber = self.create_subscription(
            Float64MultiArray,
            "/sensor_matrices",
            self.secure_state_estimator,
            10,
        )
        self.input_subscriber = self.create_subscription(
            Float64MultiArray,
            "/input_matrices",
            self.update_input_matrix,
            10,
        )


    def start_ssr_subscriber(self, msg: Empty):
        self.start_ssr = not self.start_ssr

    def secure_state_estimator(self, msg: Float64MultiArray):
        self.update_sensor_matrix(msg)

        if not self.start_ssr:
            return
        # Prerequisite is to have enough past readings
        # if len(self.y_vec) < self.drone.n:
        if len(self.y_vec) < self.n or len(self.u_vec) < self.n:
            return

        ss_problem = SSProblem(dtsys_a=self.dtsys_a, dtsys_b=self.dtsys_b, dtsys_c=self.dtsys_c, dtsys_d=self.dtsys_d,
                               attack_sensor_count=self.s, output_sequence=np.array(self.y_vec), input_sequence=np.array(self.u_vec))
        ssr_solution = SecureStateReconstruct(ss_problem)
        possible_states, corresp_sensor, _ = ssr_solution.solve(1)
        print(possible_states)

    def update_sensor_matrix(self, msg: Float64MultiArray):
        # [[x0, y0, z0], ... , [xn-1, yn-1, zn-1]]
        self.y_vec.append(list(msg.data))

        # Keep only the last n readings
        # if len(self.y_vec) > self.drone.n:
        if len(self.y_vec) > self.n:
            self.y_vec = self.y_vec[1:]

    def update_input_matrix(self, msg: Float64MultiArray):
        # [[u0], [u1], ... , [un-1]]
        self.u_vec.append(list(msg.data))

        # Keep only the last n readings
        # if len(self.u_vec) > self.drone.n:
        if len(self.u_vec) > self.n:
            self.u_vec = self.u_vec[1:]


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
