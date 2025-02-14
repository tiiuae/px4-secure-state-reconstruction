import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile
from std_msgs.msg import Empty
from px4_offboard_control.msg import TimestampedArray
from px4_msgs.msg import TrajectorySetpoint
from collections import deque


from px4_ssr.drone_system import TS, SecureStateReconstruct, SSProblem, SystemModel,nchoosek


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

        self.qos_profile = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5)
        system_model = SystemModel()

        # self.s = self.drone.p - 1
        self.dtsys_a, self.dtsys_b, self.dtsys_c, self.dtsys_d = (
            SSProblem.convert_ct_to_dt(
                system_model.Ac, system_model.Bc, system_model.Cc, system_model.Dc, TS
            )
        )
        self.declare_parameter("system_model/A/dim/row", np.size(self.dtsys_a, 0))
        self.declare_parameter("system_model/A/dim/column", np.size(self.dtsys_a, 1))
        self.declare_parameter("system_model/B/dim/row", np.size(self.dtsys_b, 0))
        self.declare_parameter("system_model/B/dim/column", np.size(self.dtsys_b, 1))
        self.declare_parameter("system_model/C/dim/row", np.size(self.dtsys_c, 0))
        self.declare_parameter("system_model/C/dim/column", np.size(self.dtsys_c, 1))
        self.declare_parameter("system_model/D/dim/row", np.size(self.dtsys_d, 0))
        self.declare_parameter("system_model/D/dim/column", np.size(self.dtsys_d, 1))

        self.declare_parameter("system_model/A/values", ''.join(str(x) for x in list(np.reshape(np.array(self.dtsys_a), (1, -1))[0])))
        self.declare_parameter("system_model/B/values", ''.join(str(x) for x in list(np.reshape(np.array(self.dtsys_b), (1, -1))[0])))
        self.declare_parameter("system_model/C/values", ''.join(str(x) for x in list(np.reshape(np.array(self.dtsys_c), (1, -1))[0])))
        self.declare_parameter("system_model/D/values", ''.join(str(x) for x in list(np.reshape(np.array(self.dtsys_d), (1, -1))[0])))

        self.n: int = self.dtsys_a.shape[0]

        self.s = 1
        self.y_vec = []
        self.u_vec = []
        self.gamma_set = []
        self.start_ssr = False
        self.estimator_window = 2*self.n
        self.residual_bound = 20
        self.residual_aggregation= {}
        self.possible_sensor_comb = nchoosek(
                [i for i in range(self.dtsys_c.shape[0])], self.dtsys_c.shape[0] - self.s
            )

        self.ssr_subscriber = self.create_subscription(
            Empty,
            "/start_ssr",
            self.start_ssr_subscriber,
            self.qos_profile,
        )
        self.sensor_subscriber = self.create_subscription(
            TimestampedArray,
            "/sensor_matrices",
            self.secure_state_estimator,
            10,
        )

        # self.nominal_input_subscriber = self.create_subscription(
        #     TimestampedArray,
        #     "/input_matrices",
        #     self.update_nominal_input_matrix,
        #     10,
        # )
        # self.safe_input_subscriber = self.create_subscription(
        #     TimestampedArray,
        #     "/u_safe",
        #     # "/input_matrices",
        #     self.update_safe_input_matrix,
        #     10,
        # )

        self.estimated_states_publisher = self.create_publisher(
            TimestampedArray, "/estimated_states", 10
        )
        self.residuals_publisher = self.create_publisher(
            TimestampedArray, "/residuals_list", 10
        )
        self.control_signal = self.create_subscription(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            self.update_input_matrix,
            10
        )

    def start_ssr_subscriber(self, msg: Empty):
        self.start_ssr = not self.start_ssr

    def secure_state_estimator(self, msg: TimestampedArray):
        self.update_sensor_matrix(msg)

        if not self.start_ssr:
            return
        # Prerequisite is to have enough past readings
        # if len(self.y_vec) < self.drone.n:
        if len(self.y_vec) < self.estimator_window or len(self.u_vec) < self.estimator_window:
            return
        # Since u_vec should contain the last n-1 inputs and the most current
        # input is zeros/not decided yet.
        u_vec = self.u_vec[1:]
        u_vec.append([0 for _ in range(self.dtsys_b.shape[1])])

        ss_problem = SSProblem(
            dtsys_a=self.dtsys_a,
            dtsys_b=self.dtsys_b,
            dtsys_c=self.dtsys_c,
            dtsys_d=self.dtsys_d,
            attack_sensor_count=self.s,
            output_sequence=np.array(self.y_vec),
            input_sequence=np.array(u_vec),
        )
        ssr_solution = SecureStateReconstruct(ss_problem,possible_comb=self.possible_sensor_comb)
        # possible_states, corresp_sensor, residuals_list = ssr_solution.solve(np.inf)
        possible_states, corresp_sensor, residuals_list = ssr_solution.solve(self.residual_bound)
        # print("-------------------------------------")
        # print(f"self.s: {self.s}")
        # print(
        #     f"dtsys_a: {self.dtsys_a}, \n dtsys_b: {self.dtsys_b}, \n dtsys_c:{self.dtsys_c} "
        # )
        # print(f"output_sequence:{self.y_vec}")
        # print(f"input_sequence:{u_vec}")
        # # print(f'observation matrix: {ssr_solution.obser[:,:,0]}')
        # # print(f'observation matrix: {ssr_solution.obser[:,:,1]}')
        # # print(f'clean output sequence" {ssr_solution.y_his}')
        # print(f"estimated state: {possible_states}")
        # print(f"corresponding sensors:{corresp_sensor}")
        # print(f"residuals_list:{residuals_list}")

        # corresp_sensor is a list of lst
        if len(corresp_sensor) > 1:
            # do residual aggregation
            for sensor_ind in range(len(corresp_sensor)):
                sensors_key = tuple(corresp_sensor[sensor_ind])
                residual = residuals_list[sensor_ind]        
                # Initialize the deque if the key is new
                if sensors_key not in self.residual_aggregation:
                    self.residual_aggregation[sensors_key] = {'residuals': deque(maxlen=100), 'average': 0,'count': 0}
                # Add the residual to the deque
                self.residual_aggregation[sensors_key]['residuals'].append(residual)
                # Compute the running average
                self.residual_aggregation[sensors_key]['average'] = sum(
                    self.residual_aggregation[sensors_key]['residuals']
                ) / len(self.residual_aggregation[sensors_key]['residuals'])
                self.residual_aggregation[sensors_key]['count'] += 1

            # find minimal average residual
            min_average = float('inf')
            min_count = float('inf')
            for sensor_ind in range(len(corresp_sensor)):
                sensors_key = tuple(corresp_sensor[sensor_ind])
                if self.residual_aggregation[sensors_key]['count']>100 and \
                    self.residual_aggregation[sensors_key]['average']<min_average:
                    min_average = self.residual_aggregation[sensors_key]['average']
                    min_count = self.residual_aggregation[sensors_key]['count']

            # remove some possible sensor combinations based on average residual
            sensors_to_remove = []
            for sensor_ind in range(len(corresp_sensor)):
                sensors_key = tuple(corresp_sensor[sensor_ind])
                average_res = self.residual_aggregation[sensors_key]['average']
                count_res =  self.residual_aggregation[sensors_key]['count']
                print(f'sensor_comb:{sensors_key}, count_res:{count_res},average_res:{average_res}')
                # a valid sensor combination should be consistent (count and the average)
                if (count_res > 5/TS and count_res < 0.9*min_count) or average_res > 1.2*min_average:
                    sensors_to_remove.append(sensor_ind)
                    print('found a sensor combination to remove')

            possible_states = np.delete(possible_states, sensors_to_remove, axis=1)
            corresp_sensor_to_remove = [sensors for i, sensors in enumerate(corresp_sensor) if i in sensors_to_remove]
            residuals_list = [res for i, res in enumerate(residuals_list) if i not in sensors_to_remove]

            set1 = set(tuple(sensors) for sensors in self.possible_sensor_comb )
            to_remove_set = set(tuple(sensors) for sensors in corresp_sensor_to_remove)

            set_exclusion = set1 - to_remove_set
            self.possible_sensor_comb = [item for item in set_exclusion]

        # if there is no sensor combination that gives an error smaller than the threshold,
        # reset the residual aggregation and possible sensor combinations
        if not any(res < self.residual_bound for res in residuals_list):
            self.residual_aggregation = {}
            self.possible_sensor_comb = nchoosek(
                [i for i in range(self.dtsys_c.shape[0])], self.dtsys_c.shape[0] - self.s
            )

        # remove duplicat states
        possible_states = possible_states.transpose()
        possible_states = ssr_solution.remove_duplicate_states(possible_states)
        possible_states = np.hstack(possible_states)

        estimated_states = TimestampedArray()
        estimated_states.header = msg.header
        estimated_states.array.data = list(possible_states.reshape(1, -1)[0])
        self.estimated_states_publisher.publish(estimated_states)
        residuals = TimestampedArray()
        residuals.header = msg.header
        residuals.array.data = list(np.array(residuals_list).reshape(1, -1)[0])
        self.residuals_publisher.publish(residuals)

    def update_sensor_matrix(self, msg: TimestampedArray):
        # [[x0, y0, z0], ... , [xn-1, yn-1, zn-1]]
        self.y_vec.append(list(msg.array.data))

        # Keep only the last n readings
        # if len(self.y_vec) > self.drone.n:
        if len(self.y_vec) > self.estimator_window:
            self.y_vec = self.y_vec[1:]

    # def update_safe_input_matrix(self, msg: TimestampedArray):
        # [[u0], [u1], ... , [un-1]]
        # Keep only the last n readings
        # if len(self.u_vec) > self.drone.n:
        # if len(self.u_vec) >= self.estimator_window:
        #     self.u_vec.append(list(msg.array.data))
        #     self.u_vec = self.u_vec[1:]

    # def update_nominal_input_matrix(self, msg: TimestampedArray):
    #     if len(self.u_vec) < self.estimator_window:
    #         self.u_vec.append(list(msg.array.data))

    def update_input_matrix(self, msg: TrajectorySetpoint):
        self.u_vec.append(list(msg.velocity)[:2])
        if len(self.u_vec) > self.estimator_window:
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
