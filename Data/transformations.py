from Helpers.input import Motion

import numpy as np
from spatialmath import SE3, base

def PoseToCart(T: SE3):
    x = np.empty(6)
    # Translational
    x[:3] = T.A[:3, -1]
    # Angular
    x[3:] = base.tr2rpy(T.A, order='zyx', check=False)
    return np.array(x)

def CartToPose(x: np.ndarray):
    T = SE3.Trans(x[:3])*SE3.RPY(x[3:], order = 'zyx')
    return T

def GetDot(x: np.ndarray, x0: SE3|np.ndarray):
    dot = [
        (x[i] - x0)/Motion.ts if i == 0 
        else (x[i] - x[i-1])/Motion.ts 
        for i in range(len(x))
    ]
    return dot