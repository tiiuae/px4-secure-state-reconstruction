import itertools
from typing import List

import control as ct
import numpy as np
import scipy.linalg as linalg
from cvxopt import matrix, solvers

EPS: float = 1e-6
TS: float = 0.1
TAU: float = 5.0 # 2.0 works

def right_shift_row_array(a, shift_amount):
    """
    shift a row array (1,d) rightwards by shift_amount
    a = np.array([a1, a2, a3]) or a = np.array([[a1, a2, a3]]) or a = np.array([[a1], [a2], [a3]])
    a = right_shift_row_array(a,2)
    # result
    a = np.array([[a2,a3,a1]])
    """
    a.reshape(1, -1)
    to_shift = a[0, -shift_amount:]
    a_shift = np.concatenate((to_shift, a[0, 0:-shift_amount]))
    a_shift.reshape(1, -1)
    return a_shift


def nchoosek(v: List[int], k: int) -> List[List[int]]:
    """
    Returns a list of lists containing all possible combinations of the elements of vector v taken k at a time.

    Args:
        v (List[int]): A list of elements to take combinations from.
        k (int): The number of elements in each combination.

    Returns:
        List[List[int]]: A list of lists where each sublist is a combination of k elements from v.
    """
    return [list(comb) for comb in itertools.combinations(v, k)]

class SystemModel:
    def __init__(self):
        self.Ac = np.matrix(
            [
                [0, 1, 0, 0],       # x1
                [0, -1/TAU, 0, 0],  # x1dot
                [0, 0, 0, 1],       # x2
                [0, 0, 0, -1/TAU],  # x2dot
            ]
        )
        self.Bc = np.matrix(
            [
                [0, 0],
                [0, 1/TAU],  # v1_command
                [0, 0],      # v2_command
                [0, 1/TAU],
            ]
        )
        self.Cc = np.matrix(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],  # x1
                [0, 0, 0, 1],  # x1dot
                [1, 0, 0, 0],  # x2
                [0, 1, 0, 0],  # x2dot
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        self.Dc = np.zeros((self.Cc.shape[0], self.Bc.shape[1]))

class SSProblem:
    """
    This class defines system model, input and measurement data, etc for SSR problem

    Conventions:
    A, B, C, D: state-space system matrices,  2d-array
    n,m,p,s,io_length: integers
    input_sequence, output_sequence: 2d-array. Each row denotes input at one time instant.
    according to (5) in the paper, input output have the same length.
    output_sequence[-1,:] is the most recent input/output at current time  t
    input_sequence[-1,:] is the input to be determined, and must be all zero when initializes.
    input_sequence[0,:]/output_sequence[0,:] is the earliest input/output at time  t-io_length+1 or 0
    """

    def __init__(
        self,
        dtsys_a,
        dtsys_b,
        dtsys_c,
        dtsys_d,
        output_sequence,
        attack_sensor_count=2,
        input_sequence=None,
        measurement_noise_level=None,
    ) -> None:
        self.A = dtsys_a
        self.B = dtsys_b
        self.C = dtsys_c
        self.D = dtsys_d
        self.io_length = output_sequence.shape[0]
        self.noise_level = measurement_noise_level

        self.n = np.shape(dtsys_a)[0]  # dim of states
        self.m = np.shape(dtsys_b)[1]  # dim of inputs
        self.p = np.shape(dtsys_c)[0]  # no. of sensors
        self.s = attack_sensor_count  # max no. of attacked sensors

        if input_sequence is not None:
            self.u_seq = input_sequence
            self.tilde_y_his = output_sequence
        else:
            self.u_seq = np.zeros((self.io_length, self.m))
            self.y_his = output_sequence

        assert self.n == np.shape(self.A)[0]
        assert self.n == np.shape(self.A)[1]
        assert self.n == np.shape(self.B)[0]
        assert self.m == np.shape(self.B)[1]
        assert self.p == np.shape(self.C)[0]
        assert self.n == np.shape(self.C)[1]

        assert self.io_length == np.shape(self.u_seq)[0]
        assert self.m == np.shape(self.u_seq)[1]
        assert self.io_length == np.shape(output_sequence)[0]
        assert self.p == np.shape(output_sequence)[1]

    @staticmethod
    def update_state_one_step(dtsys_a, dtsys_b, state_array, ut):
        """
        This method propagates an array of system states xt forward in time given an input ut

        """
        # xt is of dimension n \times no. of possible states
        n = dtsys_a.shape[0]
        m = dtsys_b.shape[1]
        if state_array.ndim !=2:
            raise KeyError('When propagating state, state_array should be of 2d array')
        if state_array.shape[0] != n:
            raise KeyError('When propagating state, state_array dimension should be n times number of possible states')
        ut.reshape(m, 1)
        x_new = dtsys_a @ state_array + (dtsys_b @ ut).reshape(
            n, 1
        )  #  ().reshape(n,1) for broadcasting
        return x_new

    @classmethod
    def update_state(cls, dtsys_a, dtsys_b, state_array, u_seq):
        """
        This method updates an array of system states xt given an input sequence u_seq.
        x: (n,), (n,1), (1,n) array
        u_seq: (t,m) array for multiple steps, (m,), (m,1), (1,m) array for one step update
        """
        m = dtsys_b.shape[1]
        x_new = state_array
        duration, remainder = divmod(u_seq.size, m)
        if remainder != 0:
            raise ValueError(
                "Number of inputs divided by input size must be an integer"
            )
        else:
            x_old = state_array
            if duration == 1:
                x_new = cls.update_state_one_step(dtsys_a, dtsys_b, x_old, u_seq)
            else:
                for t in range(duration):
                    x_new = cls.update_state_one_step(
                        dtsys_a, dtsys_b, x_old, u_seq[t, :]
                    )
                    x_old = x_new
        return x_new

    @staticmethod
    def convert_ct_to_dt(Ac, Bc, Cc, Dc, ts):
        """
        From continuous time system (Ac, Bc, Cc, Dc) to discrete-time system (A,B,C,D) with ZOH discretization scheme
        """
        continuous_sys = ct.ss(Ac, Bc, Cc, Dc)
        discrete_sys = continuous_sys.sample(ts, method="zoh")
        return (
            discrete_sys.A,
            discrete_sys.B,
            discrete_sys.C,
            discrete_sys.D,
        )


class SecureStateReconstruct:
    """
    This class implements different SSR algorithms and yields possible states and corresponding sensors
    """

    def __init__(self, ss_problem: SSProblem, possible_comb=None) -> None:
        self.problem = ss_problem
        # 3d narray, shape (io_length, n, p)
        self.obser = self.construct_observability_matrices()

        # possible healthy sensor combinations
        if possible_comb is None:
            num_healthy_sensors = ss_problem.p - ss_problem.s
            self.possible_comb = nchoosek(
                [i for i in range(ss_problem.p)], num_healthy_sensors
            )
        else:
            self.possible_comb = possible_comb

        # check if the clean measurement exists
        if hasattr(ss_problem, "y_his"):
            self.y_his = ss_problem.y_his
        else:
            # 2d narray, shape (io_length, p). Definition per (5)
            self.y_his = self.construct_clean_measurement()

    def construct_observability_matrices(self):
        ss_problem = self.problem
        obser_matrix_array = np.zeros(
            (ss_problem.io_length, ss_problem.n, ss_problem.p)
        )

        A = ss_problem.A
        for i in range(ss_problem.p):
            Ci = ss_problem.C[i : i + 1, :]
            obser_i = Ci @ linalg.fractional_matrix_power(A, 0)
            for t in range(1, ss_problem.io_length):
                new_row = Ci @ linalg.fractional_matrix_power(A, t)
                obser_i = np.vstack((obser_i, new_row))
            obser_matrix_array[:, :, i : i + 1] = obser_i.reshape(
                ss_problem.io_length, ss_problem.n, 1
            )
        return obser_matrix_array

    def construct_clean_measurement(self):
        ss_problem = self.problem
        tilde_y = ss_problem.tilde_y_his
        u_seq = ss_problem.u_seq
        io_length = ss_problem.io_length
        # Check (5) for definition
        # u_list = [input at time t_now - io_length+1, input at time t_now -io_length+2, ..., input at time t_now]
        u_list = [u_seq[t, :] for t in range(io_length)]
        u_vec = np.vstack(u_list).reshape(-1, 1)  # 2d column array

        # yi
        yi_list = []
        for i in range(ss_problem.p):
            tilde_yi = tilde_y[:, i : i + 1]  # 2d array of dimension (io_length,1)
            fi = self.construct_fi(i)  # 2d array of dimension (io_length,io_length*m)
            yi = tilde_yi - fi @ u_vec  # 2d array of dimension (io_length,1)
            yi_list.append(yi)

        # y_his: shape (io_length, p) = [y1 y2 ... yp] for p sensors, and yi is a column 2d array [yi(0); yi(1); .... yi(io_length-1)]
        y_his = np.hstack(yi_list)
        return y_his

    def construct_fi(self, i):
        """
        construct Fi according to Eq. 5 in the paper
        """
        Ci = self.problem.C[i, :]
        A = self.problem.A
        B = self.problem.B
        io_length = self.problem.io_length
        m = self.problem.m

        fi = np.zeros((io_length, io_length * m))
        # note that row count of Fi  =  io_length. The first row is zeros.
        # see (5) for definition
        for t in range(1, io_length):
            fi[t : t + 1, :] = right_shift_row_array(fi[t - 1 : t, :], m)
            # here for t-th row, t = 1,...,io_length -1, add the left most element Ci A^t B.
            fi[t : t + 1, 0:m] = Ci @ linalg.fractional_matrix_power(A, t - 1) @ B
        return fi

    def solve_initial_state(self, error_bound: float = 1.0):
        """
        The method solves a given SSR problem and yields possible initial states, currently in a brute-force approach.
        """
        state_list = []
        sensor_list = []
        residual_list = []

        possible_states_list = []
        corresp_sensors_list = []
        possible_residuals_list = []
        for comb in self.possible_comb:
            # recall obser is in the shape of (io_length, n, p)
            # print(comb)
            obser_matrix = self.vstack_comb(self.obser, comb)
            # print(f'obser_matrix for comb {comb} is \n {obser_matrix}')
            # recall y_his is in the shape of (io_length, p)
            measure_vec = self.vstack_comb(self.y_his, comb)
            # print(f'corresponding measurement is \n {measure_vec}')

            # print(f'Observation matrix shape {obser_matrix.shape}, measure_vec shape {measure_vec.shape}')
            state, residuals, rank, _ = linalg.lstsq(obser_matrix, measure_vec)

            if len(residuals) < 1:
                # print(f'combinations: {comb}')
                residuals = (
                    linalg.norm(obser_matrix @ state - measure_vec, ord=2) ** 2
                )
            else:
                residuals = residuals.item()

            # if residuals < error_bound:
            #     possible_states_list.append(state)
            #     corresp_sensors_list.append(comb)
            #     possible_residuals_list.append(residuals)
            #     # print(f'residuals: {residuals}')
            #     if rank < obser_matrix.shape[1]:
            #         print(
            #             f"Warning: observation matrix for sensors in {comb} is of deficient rank {rank}."
            #         )
            state_list.append(state)
            sensor_list.append(comb)
            residual_list.append(residuals)


        # if len(possible_states_list) > 0:
        #     # here we remove the states that yields 100x smallest residual
        #     residual_min = min(possible_residuals_list)
        #     print(f"residual min is {residual_min}")
        #     # comb_list = [i for i in range(len(residuals_list)) if residuals_list[i]<10*residual_min]
        #     comb_list = [i for i in range(len(possible_residuals_list))]
        #     possible_states_list = [possible_states_list[index] for index in comb_list]
        #     corresp_sensors_list = [corresp_sensors_list[index] for index in comb_list]
        #     possible_states = np.hstack(possible_states_list)
        #     corresp_sensors = np.array(corresp_sensors_list)
        # else:
        #     possible_states = None
        #     corresp_sensors = None
        #     print("No possible state found. Consider relax the error bound")

        residual_min = min(residual_list)
        # print(f"residual min is {residual_min}")
        if residual_min<min(error_bound,10*residual_min):
            possible_states_list = [state for residual, state in zip(residual_list, state_list) if residual < error_bound]
            corresp_sensors_list = [sensors for residual, sensors in zip(residual_list, sensor_list) if residual < error_bound]
            possible_states = np.hstack(possible_states_list)
            corresp_sensors = np.array(corresp_sensors_list)
        else:
            print("No possible state found. SSR gives the state with smallest residual. Consider relax the error bound")
            residual_min_index =  residual_list.index( min(residual_list))
            possible_states = np.reshape(state_list[residual_min_index],(-1,1))
            corresp_sensors = np.array(sensor_list[residual_min_index])

        return possible_states, corresp_sensors, possible_residuals_list

    def solve(self, error_bound: float = 1.0):
        # Solves for current states
        possible_states, corresp_sensors, residuals_list = (
            self.solve_initial_state(error_bound)
        )
        if possible_states is None:
            return None, corresp_sensors, residuals_list

        current_states_list = []
        for ind in range(possible_states.shape[1]):
            init_state = possible_states[:, ind:ind+1]
            curr_state = self.problem.update_state(
                self.problem.A, self.problem.B, init_state, self.problem.u_seq
            )
            current_states_list.append(curr_state)
        current_states = np.hstack(current_states_list)
        return current_states, corresp_sensors, residuals_list

    @classmethod
    def vstack_comb(
        cls,
        array_to_stack: np.ndarray,
        comb: List[int] | None = None,
        axis: int | None = None,
    ):
        """
        vertically stack numpy array that is sliced along **axis** with some **combination**,
        default axis: last axis; default comb: full indices

        a = np.array([3, 1, 4, 1, 5, 9, 2, 6, 5, 3, 5,2])
        b = a.reshape(2,2,-1)
        c = SecureStateReconstruct.vstack_comb(b,comb = [1,2],axis = 2)
        """
        if axis is None:
            axis = array_to_stack.ndim - 1  # last dimension
        if comb is None:
            p = array_to_stack.shape[axis]
            comb = [i for i in range(p)]
        temp_list = []
        for i in comb:
            temp_i = cls.slice_along_axis(array_to_stack, i, axis)
            if temp_i.ndim == 1:
                temp_i = temp_i.reshape(-1, 1)  # so temp_i becomes 2d column array
            temp_list.append(temp_i)
        stacked_array = np.vstack(temp_list)
        return stacked_array

    @staticmethod
    def unvstack(array_to_unstack: np.ndarray, row_count_for_a_slice: int):
        """
        undo the vstack operation

        array_to_unstack: 2d or 3d array
        """
        rows = array_to_unstack.shape[0]
        quotient, remainder = divmod(rows, row_count_for_a_slice)
        if remainder != 0:
            Warning("unable to unvstack the array")
        temp_lst = []
        for i in range(quotient):
            temp_lst.append(
                array_to_unstack[
                    i * row_count_for_a_slice : (i + 1) * row_count_for_a_slice, :
                ]
            )
        if array_to_unstack.shape[1] == 1:
            array_unstacked = np.hstack(temp_lst)
        else:
            array_unstacked = np.dstack(temp_lst)

        return array_unstacked

    @staticmethod
    def slice_along_axis(array, j, k):
        """
        Extract the j-th slice along the k-th axis of a d-dimensional array.

        Parameters:
        array (np.ndarray): The input d-dimensional array.
        j (int): The index of the slice to extract.
        k (int): The axis along which to extract the slice.

        Returns:
        np.ndarray: The extracted slice.
        """
        # Create a full slice (:) for each dimension
        index = [slice(None)] * array.ndim

        # Replace the slice for the k-th axis with the specific index j
        index[k] = j

        # Convert the list to a tuple and use it to slice the array
        return array[tuple(index)]
    
    @staticmethod
    def is_same_state(st1:np.ndarray,st2:np.ndarray):
        error = st1.flatten() - st2.flatten()
        # print(f'error norm: {linalg.norm(error)}')
        if linalg.norm(error)<1e-6:
            return True
        return False

    @classmethod
    def remove_duplicate_states(cls,possible_states):
        state_lst = [possible_states[0]]
        for state in possible_states[1:]:
            if not any(cls.is_same_state(state,st) for st in state_lst):
                state_lst.append(state)
        return state_lst

class LinearInequalityConstr:
    """
    This class defines linear inequality constraints and methods including compare, visualize

    linear inequality constraints: a_mat x + b_vec >= 0 (or, -a_mat x >= b_vec)
    a: (M,N) array
    b: (M,1) array
    """

    def __init__(self, a: np.ndarray, b: np.ndarray) -> None:
        b = b.reshape(-1, 1)
        assert a.shape[0] == b.shape[0]
        self.a_mat = a
        self.b_vec = b

    def compare(self, linear_ineq_constr):
        pass

    def visualize(self, dim=(0, 1)):
        pass

    def combine(self, linear_ineq_constr):
        a_mat = np.vstack((self.a_mat, linear_ineq_constr.a_mat))
        b_vec = np.vstack((self.b_vec, linear_ineq_constr.b_vec))
        return LinearInequalityConstr(a_mat, b_vec)

    def is_satisfy(self, u):
        u = u.reshape(-1, 1)
        assert u.shape[0] == self.a_mat.shape[1]

        return all(self.a_mat @ u + self.b_vec + 1e-6 >= 0)


class SafeProblem:
    """
    This class defines data for safe problem
    """

    def __init__(self, A, B, h, q, gamma) -> None:
        assert gamma >= 0
        assert gamma <= 1

        self.A = A
        self.B = B
        self.u_seq = np.array([])
        self.n = A.shape[0]
        self.m = B.shape[1]
        self.io_length = self.n

        assert h.shape[0] == q.shape[0]
        assert h.shape[1] == self.A.shape[1]
        assert q.shape[1] == 1

        self.h = h
        self.q = q
        self.gamma = gamma

        # according to (7) of the note "Safety of linear systems under sensor attacks without state estimation
        self.k = (1 - gamma) * h @ np.linalg.matrix_power(
            self.A, self.io_length
        ) - h @ np.linalg.matrix_power(self.A, self.io_length + 1)

    def cal_cbf_condition_state(self, state) -> LinearInequalityConstr:
        state = state.reshape(-1, 1)
        a_mat = self.h @ self.B
        b_vec = (
            self.h @ self.A @ state
            + self.q
            - (1 - self.gamma) * (self.h @ state + self.q)
        )
        return LinearInequalityConstr(a_mat, b_vec)

    def merge_multiple_LIC(self, constr_list) -> LinearInequalityConstr:
        lic_full = constr_list[0]
        if len(constr_list) == 1:
            return lic_full

        for lic in constr_list[1:]:
            lic_full = LinearInequalityConstr.combine(lic_full, lic)
        return lic_full

    def cal_cbf_condition(self, possible_states) -> LinearInequalityConstr:
        """
        This method calculates CBF conditions over a set of states
        """
        cbf_conditions = []
        for state in possible_states:
            cbf_condition = self.cal_cbf_condition_state(state)
            cbf_conditions.append(cbf_condition)

        total_cbf_condition = self.merge_multiple_LIC(cbf_conditions)
        return total_cbf_condition

    def cal_Q_ut(self):
        """
        construct Q(u(t)) according to (7) of the note "Safety of linear systems under sensor attacks without state estimation

        """

        H = self.h
        q = self.q
        gamma = self.gamma

        A = self.A
        B = self.B
        io_length = self.io_length
        n = self.n
        m = self.m

        u_seq = self.u_seq

        left_mat = H @ ((1 - gamma) * np.identity(n) - A)
        right_vec = np.zeros((n, 1))
        for k in range(io_length - 1):
            ut = u_seq[k, :]
            ut = ut.reshape(-1, 1)
            temp = (
                linalg.fractional_matrix_power(A, io_length - 2 - k) @ B @ ut
            )  # note in the paper, io_length = t + 1
            right_vec = right_vec + temp

        return left_mat @ right_vec - gamma * q

    def cal_safe_input_constr_woSSR(
        self, initial_states_subssr
    ) -> LinearInequalityConstr:
        """
        This method implements our computationally efficient CBF conditions
        initial_states_subssr is a list of possible_initial_states, each entry correspnds to one subspace
        possible_initial_states = initial_states_subssr[j] is a n*x 2d array, each column corresponds to one state

        """
        Q_ut = self.cal_Q_ut()
        kv_maxsum = np.zeros(np.shape(self.q))
        count_subspace = len(initial_states_subssr)
        for j in range(count_subspace):
            possible_initial_states = initial_states_subssr[j]
            kv = self.k @ possible_initial_states
            kv_max = kv.max(axis=1)
            kv_max = kv_max.reshape(-1, 1)
            kv_maxsum = kv_maxsum + kv_max
        # a_mat x + b_vec \geq 0
        a_mat = self.h @ self.B
        b_vec = -kv_maxsum - Q_ut

        # print(f'a_mat: {a_mat.shape}')
        # print(f'b_vec: {b_vec.shape}')
        # print(f'kv_maxsum: {kv_maxsum.shape}')
        # print(f'Q_ut shape: {Q_ut}')
        return LinearInequalityConstr(a_mat, b_vec)

    def cal_safe_qp(self, u_nom, lic: LinearInequalityConstr):
        """
        sol_flag: -1 -- qp solver fails, 0 -- qp status "unknown", 1 -- qp status 'optimal'
        """
        flag = 0
        # Define QP parameters for qp solver (CVXOPT):
        # min 0.5 xT P x+ qT x s.t. Gx <= h
        qp_P = matrix(np.identity(u_nom.shape[0]))
        qp_q = matrix(-u_nom, (u_nom.shape[0], 1), "d")  #  qp_q is a m*1 matrix
        qp_G = matrix(-lic.a_mat)
        qp_h = matrix(lic.b_vec, (lic.b_vec.shape[0], 1), "d")  # qp_h is a x*1 matrix

        solvers.options["show_progress"] = False  # mute optimization output
        solvers.options["maxiters"] = 500  # increase max iteration number
        solvers.options["abstol"] = 1e-4
        solvers.options["reltol"] = 1e-5
        try:
            solv_sol = solvers.qp(qp_P, qp_q, qp_G, qp_h)
            if solv_sol["status"] == "unknown":
                print(
                    "warning: safe control is approximately computed. Use u_nom instead."
                )
                u = u_nom.flatten()
            else:
                flag = 1
                u = np.array(
                    solv_sol["x"]
                ).flatten()  # extract decision variables of optimization problem
        except Exception as _:
            # print('cvxopt solver failed:')
            # print(f'qp_P: {qp_P}')
            # print(f'qp_q: {qp_q}')
            # print(f'qp_G: {qp_G}')
            # print(f'qp_h: {qp_h}')

            # solvers.options['show_progress'] = True
            # solv_sol = solvers.qp(qp_P,qp_q,qp_G,qp_h)

            # raise TypeError('no control input found')
            print("warning: safe control fails. Use u_nom instead.")
            flag = -1
            u = u_nom.flatten()

        return u, flag

    def cal_safe_control(self, u_nom, possible_states):
        lic = self.cal_cbf_condition(possible_states)
        u, flag = self.cal_safe_qp(u_nom, lic)
        return u, lic, flag

    def solve_safe_control_by_brute_force(self, u_nom, possible_states):
        # solve ssr by brute-force
        possible_states = possible_states.transpose() # now possible_states[0] is one possible state
        possible_states = self.remove_duplicate_states(possible_states)
        u_safe1,lic1,flag1 = self.cal_safe_control(u_nom,possible_states)
        return u_safe1, lic1, flag1

    def remove_duplicate_states(self, possible_states):
        state_lst = [possible_states[0]]
        for state in possible_states[1:]:
            if not any(self.is_same_state(state,st) for st in state_lst):
                state_lst.append(state)
        return state_lst

    def is_same_state(self, st1:np.ndarray, st2:np.ndarray):
        error = st1.flatten() - st2.flatten()
        # print(f'error norm: {linalg.norm(error)}')
        if linalg.norm(error)<1e-6:
            return True
        return False
