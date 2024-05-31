import numpy as np
import control as ct


class DroneSystem:

    def __init__(self, A=None, B=None, C=None, D=None, ts=0.01):

        self.ts = ts

        self._define_state(A, B, C, D)
        self.n = np.size(self.Ad,0)
        self.m = np.size(self.Bd,1)
        self.p = np.size(self.Cd,0)

        self._define_barriers()
        self._define_auxiliary_matrices()


    def _define_state(self, A, B, C, D):

        if A == None:
            self.Ac = np.matrix(
                [
                    # [0, 0, 1, 0],
                    # [0, 0, 0, 1],
                    # [0, 0, 0, 0],
                    # [0, 0, 0, 0]
                    [0, 1], # x xdot
                    [0, 0],
                ]
            )
        else:
            self.Ac = A

        if B == None:
            self.Bc = np.matrix(
                [
                    # [0, 0],
                    # [0, 0],
                    # [1, 0],
                    # [0, 1]
                    [0],
                    [1]
                ]
            )
        else:
            self.Bc = B

        if C == None:
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
                    [1, 0], # x1 xdot1 x2 xdot2
                    [0, 1],
                    [1, 0],
                    [0, 1]

                ]
            )
        else:
            self.Cc = C

        if D == None:
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
                    [0]
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
        self.H = np.matrix([[ 1, 0],
                            [-1, 0],
                            [ 0, 1],
                            [ 0,-1]])
        self.q = 4 * np.ones((4,1));


    def _define_auxiliary_matrices(self):
        self.O_cell = np.zeros((self.p, self.n, self.n))
        self.F_cell = np.zeros((self.p, self.n, self.n * self.m))
        for i in range(self.p):
            self.O_cell[i] = ct.obsv(self.Ad, self.Cd[i])
            Fi = np.zeros((self.n, self.n * self.m))

            for j in range(1, self.n):
                for k in range(j):
                    Fi[j, self.m * k : self.m * (k + 1)] = self.Cd[i] @ np.linalg.matrix_power(self.Ad, j - k - 1) @ self.Bd

            self.F_cell[i] = Fi

