import itertools
import threading
import time
from geometry_msgs.msg import Point

import numpy as np
import rclpy
from px4_offboard_control.msg import TimestampedArray
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile
from std_msgs.msg import Empty, Bool

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
        super().__init__("safe_controller", parameter_overrides=[])

        self.qos_profile = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5)
        system_model = SystemModel()

        # self.s = self.drone.p - 1
        self.dtsys_a, self.dtsys_b, self.dtsys_c, self.dtsys_d = (
            SSProblem.convert_ct_to_dt(
                system_model.Ac, system_model.Bc, system_model.Cc, system_model.Dc, TS
            )
        )
        self.n: int = self.dtsys_a.shape[0]

        # self.y_vec = []
        self.u_vec = []
        self.x_array = []
        self.sensor_vector = []
        self.gamma_set = []
        self.start_ssr = False
        self.reconstruct_state = False
        self.declare_parameter("state_reconstruction_state", self.reconstruct_state)

        # The absolute limits to square boundary in 2D
        self.square_bound = 5.1
        h = np.vstack([np.identity(self.n),-np.identity(self.n)])
        q = self.square_bound * np.ones((2*self.n,1))
        gamma = 1*TS # tuning parameter

        self.safe_problem = SafeProblem(self.dtsys_a, self.dtsys_b, h, q, gamma)

        self.ssr_subscriber = self.create_subscription(
            Empty,
            "/start_ssr",
            self.start_ssr_subscriber,
            self.qos_profile,
        )
        self.reconstruction_subscriber = self.create_subscription(
            Bool,
            "/state_reconstruction",
            self.enable_state_reconstruction,
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

        # TODO Temporary boundary publisher, can be deleted after Modularization
        self.boundary_thread = threading.Thread(target=self.boundary_loop)
        self.boundary_thread.daemon = True
        self.boundary_thread.start()

        # TODO Temporary boundary loop, can be deleted after Modularization
    def boundary_loop(self):
        self.boundary_publisher = self.create_publisher(Point, "/boundary", 10)

        values = [(self.square_bound, self.square_bound),
                  (-self.square_bound, self.square_bound),
                  (-self.square_bound, -self.square_bound),
                  (self.square_bound, -self.square_bound)
                  ]
        cycle = itertools.cycle(values)

        while rclpy.ok():
            time.sleep(.5)
            point = Point()
            x, y = next(cycle)
            point.x, point.y = float(x), float(y)
            point.z = 0.
            self.boundary_publisher.publish(point)
        return


    def start_ssr_subscriber(self, msg: Empty):
        self.start_ssr = not self.start_ssr

    def secure_safe_controller(self, msg: TimestampedArray):
        # self.update_estimated_states(msg)
        self.x_est = np.array(msg.array.data)

        if not self.start_ssr:
            return

        if not self.reconstruct_state:
            self.x_est = np.array(self.sensor_vector)

        # Reshape flattened
        self.x_est = np.reshape(self.x_est, (self.n, -1))
        self.safe_problem.u_seq = np.array(self.u_vec)

        u_safe, lic, flag = self.safe_problem.cal_safe_control(np.array(self.u_vec[-1]), self.x_est.T)

        if np.linalg.norm(u_safe - self.u_vec[-1])>0.1:
            print(f'u_safe:{u_safe}, u_nom: {self.u_vec[-1]}')
        u_safe_out = TimestampedArray()
        u_safe_out.header = msg.header
        u_safe_out.array.data = list(u_safe)

        self.safe_controls_publisher.publish(u_safe_out)

    def update_sensor_matrix(self, msg: TimestampedArray):
        self.sensor_vector = list(msg.array.data)
        self.sensor_vector = [
            (self.sensor_vector[0] + self.sensor_vector[4]) / 2,
            (self.sensor_vector[1] + self.sensor_vector[5]) / 2,
            (self.sensor_vector[2] + self.sensor_vector[6]) / 2,
            (self.sensor_vector[3] + self.sensor_vector[7]) / 2,
        ]

    def update_input_matrix(self, msg: TimestampedArray):
        # [[u0], [u1], ... , [un-1]]
        self.u_vec.append(list(msg.array.data))

        # Keep only the last n readings
        # if len(self.u_vec) > self.drone.n:
        if len(self.u_vec) > self.n:
            self.u_vec = self.u_vec[1:]

    def enable_state_reconstruction(self, msg: Bool):
        self.reconstruct_state = msg.data
        self.set_parameters([rclpy.Parameter("state_reconstruction_state", rclpy.Parameter.Type.BOOL, self.reconstruct_state)])

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
