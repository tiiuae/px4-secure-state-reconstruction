import numpy as np
import control as ct


class DroneSystem:

    def __init__(self, A=None, B=None, C=None, D=None, ts=0.01):

        self.ts = ts

        if A == None:
            self.Ac = np.matrix(
                [
                    [0, 0, 1, 0],
                    [0, 0, 0, 1],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0]
                ]
            )
        else:
            self.Ac = A

        if B == None:
            self.Bc = np.matrix(
                [
                    [0, 0],
                    [0, 0],
                    [1, 0],
                    [0, 1]
                ]
            )
        else:
            self.Bc = B

        if C == None:
            self.Cc = np.matrix(
                [
                    [1, 0, 0, 0],
                    [1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1],
                    [0, 0, 0, 1],
                ]
            )
        else:
            self.Cc = C

        if D == None:
            self.Dc = np.matrix(
                [
                    [0, 0],
                    [0, 0],
                    [0, 0],
                    [0, 0],
                    [0, 0],
                    [0, 0],
                    [0, 0],
                    [0, 0]
                ]
            )
        else:
            self.Dc = D

        assert np.size(self.Ac, 0) == np.size(self.Cc, 1)
        assert np.size(self.Ac, 0) == np.size(self.Bc, 0)
        assert np.size(self.Ac, 1) == np.size(self.Cc, 1)
        assert np.size(self.Bc, 1) == np.size(self.Dc, 1)
        assert np.size(self.Cc, 0) == np.size(self.Dc, 0)
            
        self.system_c = ct.StateSpace()
        self.system_d = ct.c2d(self.system_c, self.ts, "zoh")

        self.Ad, self.Bd, self.Cd, self.Dd = (
            self.system_d.A,
            self.system_d.B,
            self.system_d.C,
            self.system_d.D,
        )
