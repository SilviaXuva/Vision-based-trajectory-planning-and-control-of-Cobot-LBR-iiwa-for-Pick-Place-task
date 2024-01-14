import numpy as np
from spatialmath import SE3

class Pose:
    def __new__(self, x: float, y: float, z: float, rpy: np.ndarray = None, r_xyz: np.ndarray = None) -> None:
        self.x = x
        self.y = y
        self.z = z
        if rpy is None:
            self.rx = r_xyz[0]
            self.ry = r_xyz[1]
            self.rz = r_xyz[2]
            self.T = SE3.Trans(
                self.x, self.y, self.z
            )*SE3.Rx(self.rx)*SE3.Ry(self.ry)*SE3.Rz(self.rz)
        else:
            self.rpy = rpy
            self.T = SE3.Trans(
                self.x, self.y, self.z
            )*SE3.RPY(self.rpy, order = 'zyx')

        return self.T
