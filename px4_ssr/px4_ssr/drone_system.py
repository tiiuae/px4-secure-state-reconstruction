import numpy as np
import scipy as sc
import control as ct
import itertools
from typing import List

EPS: float = 1e-6
TS: float = 0.01


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


class DroneSystem:
    def __init__(self, A=None, B=None, C=None, D=None, ts=0.01):
        self.ts = ts

        self._define_state(A, B, C, D)
        self.n = np.size(self.Ad, 0)
        self.m = np.size(self.Bd, 1)
        self.p = np.size(self.Cd, 0)

        self._define_barriers()
        self._define_auxiliary_matrices()

    def _define_state(self, A, B, C, D):
        if A is None:
            self.Ac = np.matrix(
                [
                    # [0, 0, 1, 0],
                    # [0, 0, 0, 1],
                    # [0, 0, 0, 0],
                    # [0, 0, 0, 0]
                    [0, 1],  # x xdot
                    [0, 0],
                ]
            )
        else:
            self.Ac = A

        if B is None:
            self.Bc = np.matrix(
                [
                    # [0, 0],
                    # [0, 0],
                    # [1, 0],
                    # [0, 1]
                    [0],
                    [1],
                ]
            )
        else:
            self.Bc = B

        if C is None:
            self.Cc = np.matrix(
                [
                    # [1, 0, 0, 0],
                    # [1, 0, 0, 0],
                    # [0, 1, 0, 0],
                    # [0, 1, 0, 0],
                    # [0, 0, 1, 0],
                    # [0, 0, 1, 0],
                    # [0, 0, 0, 1],
                    # [0, 0, 0, 1],
                    [1, 0],  # x1 xdot1 x2 xdot2
                    [0, 1],
                    [1, 0],
                    [0, 1],
                ]
            )
        else:
            self.Cc = C

        if D is None:
            self.Dc = np.matrix(
                [
                    # [0, 0],
                    # [0, 0],
                    # [0, 0],
                    # [0, 0],
                    # [0, 0],
                    # [0, 0],
                    # [0, 0],
                    # [0, 0]
                    [0],
                    [0],
                    [0],
                    [0],
                ]
            )
        else:
            self.Dc = D

        assert np.size(self.Ac, 0) == np.size(self.Cc, 1)
        assert np.size(self.Ac, 0) == np.size(self.Bc, 0)
        assert np.size(self.Ac, 1) == np.size(self.Cc, 1)
        assert np.size(self.Bc, 1) == np.size(self.Dc, 1)
        assert np.size(self.Cc, 0) == np.size(self.Dc, 0)

        self.system_c = ct.StateSpace(self.Ac, self.Bc, self.Cc, self.Dc)
        self.system_d = ct.c2d(self.system_c, self.ts, "zoh")

        self.Ad, self.Bd, self.Cd, self.Dd = (
            self.system_d.A,
            self.system_d.B,
            self.system_d.C,
            self.system_d.D,
        )

    def _define_barriers(self):
        # self.H = np.matrix([[ 1, 0, 0, 0],
        #                     [-1, 0, 0, 0],
        #                     [ 0, 1, 0, 0],
        #                     [ 0,-1, 0, 0],
        #                     [ 0, 0, 1, 0],
        #                     [ 0, 0,-1, 0],
        #                     [ 0, 0, 0, 1],
        #                     [ 0, 0, 0,-1]])
        # self.q = 4 * np.ones((8,1));
        self.H = np.matrix([[1, 0], [-1, 0], [0, 1], [0, -1]])
        self.q = 4 * np.ones((4, 1))

    def _define_auxiliary_matrices(self):
        self.O_cell = np.zeros((self.p, self.n, self.n))
        self.F_cell = np.zeros((self.p, self.n, self.n * self.m))
        for i in range(self.p):
            self.O_cell[i] = ct.obsv(self.Ad, self.Cd[i])
            Fi = np.zeros((self.n, self.n * self.m))

            for j in range(1, self.n):
                for k in range(j):
                    Fi[j, self.m * k : self.m * (k + 1)] = (
                        self.Cd[i]
                        @ np.linalg.matrix_power(self.Ad, j - k - 1)
                        @ self.Bd
                    )

            self.F_cell[i] = Fi


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
        self.io_length = np.array(output_sequence).shape[0]
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
        discrete_sys: ct.StateSpace = ct.ss(Ac, Bc, Cc, Dc, ts)
        # discrete_sys = sys.sample(ts, method="zoh")
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
            obser_i = Ci @ sc.linalg.fractional_matrix_power(A, 0)
            for t in range(1, ss_problem.io_length):
                new_row = Ci @ sc.linalg.fractional_matrix_power(A, t)
                obser_i = np.vstack((obser_i, new_row))
            obser_matrix_array[:, :, i : i + 1] = obser_i.reshape(
                ss_problem.io_length, ss_problem.n, 1
            )
        return obser_matrix_array

    def construct_clean_measurement(self):
        ss_problem = self.problem
        tilde_y = np.array(ss_problem.tilde_y_his)
        u_seq = np.array(ss_problem.u_seq)
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
            fi[t : t + 1, 0:m] = Ci @ sc.linalg.fractional_matrix_power(A, t - 1) @ B
        return fi

    # def construct_y_his_vec(self,comb):
    #     measure_vec_list = []
    #     for i in comb:
    #         measure_i = self.y_his[:,i:i+1]
    #         measure_vec_list.append(measure_i)
    #     measure_vec = np.vstack(measure_vec_list)
    #     return measure_vec

    def solve_initial_state(self, error_bound: float = 1.0):
        """
        The method solves a given SSR problem and yields possible initial states, currently in a brute-force approach.
        """
        possible_states_list = []
        corresp_sensors_list = []
        residuals_list = []
        for comb in self.possible_comb:
            # recall obser is in the shape of (io_length, n, p)
            obser_matrix = self.vstack_comb(self.obser, comb)
            # print(f'obser_matrix for comb {comb} is \n {obser_matrix}')
            # recall y_his is in the shape of (io_length, p)
            measure_vec = self.vstack_comb(self.y_his, comb)
            # print(f'corresponding measurement is \n {measure_vec}')

            # print(f'Observation matrix shape {obser_matrix.shape}, measure_vec shape {measure_vec.shape}')
            state, residuals, rank, _ = sc.linalg.lstsq(obser_matrix, measure_vec)

            if len(residuals) < 1:
                # print(f'combinations: {comb}')
                residuals = (
                    sc.linalg.norm(obser_matrix @ state - measure_vec, ord=2) ** 2
                )

            if residuals < error_bound:
                possible_states_list.append(state)
                corresp_sensors_list.append(comb)
                residuals_list.append(residuals)
                # print(f'residuals: {residuals}')
                if rank < obser_matrix.shape[1]:
                    print(
                        f"Warning: observation matrix for sensors in {comb} is of deficient rank {rank}."
                    )

        if len(possible_states_list) > 0:
            # here we remove the states that yields 100x smallest residual
            residual_min = min(residuals_list)
            print(f"residual min is {residual_min}")
            # comb_list = [i for i in range(len(residuals_list)) if residuals_list[i]<10*residual_min]
            comb_list = [i for i in range(len(residuals_list))]
            possible_states_list = [possible_states_list[index] for index in comb_list]
            corresp_sensors_list = [corresp_sensors_list[index] for index in comb_list]
            possible_states = np.hstack(possible_states_list)
            corresp_sensors = np.array(corresp_sensors_list)
        else:
            possible_states = None
            corresp_sensors = None
            print("No possible state found. Consider relax the error bound")

        return possible_states, corresp_sensors, corresp_sensors_list

    def solve(self, error_bound: float = 1.0):
        # Solves for current states
        possible_states, corresp_sensors, corresp_sensors_list = (
            self.solve_initial_state(error_bound)
        )
        if possible_states is None:
            return None, corresp_sensors, corresp_sensors_list

        current_states_list = []
        for ind in range(possible_states.shape[1]):
            init_state = possible_states[:, ind:ind+1]
            curr_state = self.problem.update_state(
                self.problem.A, self.problem.B, init_state, self.problem.u_seq
            )
            current_states_list.append(curr_state)
        current_states = np.hstack(current_states_list)
        return current_states, corresp_sensors, corresp_sensors_list

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
