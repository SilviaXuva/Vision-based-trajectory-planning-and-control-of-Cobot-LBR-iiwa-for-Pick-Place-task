from Data.pose import Pose

import numpy as np

class Marker:
    def __init__(
            self, 
            id: int, 
            color: np.ndarray,
            mass: float,
            T: Pose
        ) -> None:
        self.id = id
        self.color = color
        self.mass = mass
        self.T = T

deg = np.pi/180
green = Marker(id = 1, color = 'green', mass = 0.125, T = Pose(
    x = 0.700, y = 0.15, z = 0.225,
    r_xyz = [0, 0, 98.012*deg]
))

blue = Marker(id = 2, color = 'blue', mass = 5, T = Pose(
    x = 0.700, y = 0, z = 0.225, 
    r_xyz = [0, 0, -123.905*deg]
))

red = Marker(id = 3, color = 'red', mass = 7, T = Pose(
    x = 0.700, y = -0.15, z = 0.225,
    r_xyz = [0, 0, 60.13*deg]
))
