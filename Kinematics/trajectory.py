from Data.transformations import PoseToCart, CartToPose, GetDot

from collections import namedtuple
import numpy as np
from roboticstoolbox import DHRobot, ERobot
from roboticstoolbox.tools.trajectory import jtraj, ctraj
from spatialmath import SE3

Trajectory = namedtuple('Trajectory', ('T', 
    'q', 'qDot', 'qDotDot', 
    'x', 'xDot', 'xDotDot'
))
def TrajectoryPlanning(type: str, source: str, robot: DHRobot|ERobot, q0: np.ndarray, T1: SE3, t: np.ndarray):
    T0 = robot.fkine(q0)
    x0 = PoseToCart(robot.fkine(q0))
    if type == 'cart':
        if source == 'rtb':
            traj = ctraj(T0, T1, t); arr = [arr for arr in traj.A if arr is not None]
            T = [SE3(pose) for pose in arr]
            xRef = [PoseToCart(SE3(pose)) for pose in traj.A]; xDotRef = GetDot(xRef, x0); xDotDotRef = GetDot(xDotRef, x0)
        elif source == 'custom':
            xRef, xDotRef, xDotDotRef = QuinticEndEffectorTraj(T0, T1, t)
            T = [CartToPose(x) for x in xRef]
        qRef = qDotRef = qDotDotRef = [None for i in range(len(T))]            
        traj = Trajectory([CartToPose(x) for x in xRef], qRef, qDotRef, qDotDotRef, xRef, xDotRef, xDotDotRef)
    else:
        q1 = robot.ikine_LMS(T1).q
        if source == 'rtb':
            traj = jtraj(q0, q1, t)
            T = [robot.fkine(q) for q in traj.q]
            qRef = traj.q; qDotRef = traj.qd; qDotDotRef = traj.qdd
        elif source == 'custom':
            qRef, qDotRef, qDotDotRef = QuinticJointTraj(q0, q1, t)
            T = [robot.fkine(q) for q in qRef]
        xRef = [PoseToCart(pose) for pose in T]; xDotRef = GetDot(xRef, x0); xDotDotRef = GetDot(xDotRef, x0)
        traj = Trajectory(T, qRef, qDotRef, qDotDotRef, xRef, xDotRef, xDotDotRef)
    return traj

def ApplyTrajectory():
    pass

def QuinticEndEffectorTraj(T0: SE3, T1: SE3, t: np.ndarray):
    x0 = PoseToCart(T0)
    x1 = PoseToCart(T1)
        
    # Matrix T
    T = np.array([
    # ---Initial---
        [1,     min(t),     min(t)**2,      min(t)**3,      min(t)**4,          min(t)**5       ],
        [0,     1,          2*min(t),       3*min(t)**2,    4*min(t)**3,        5*min(t)**4     ],
        [0,     0,          2,              6*min(t),       12*min(t)**2,       20*min(t)**3    ],
    # ---Final---
        [1,     max(t),     max(t)**2,      max(t)**3,      max(t)**4,          max(t)**5       ],
        [0,     1,          2*max(t),       3*max(t)**2,    4*max(t)**3,        5*max(t)**4     ],
        [0,     0,          2,              6*max(t),       12*max(t)**2,       20*max(t)**3    ]
    ])
    # Matrix of coefficients
    C = np.zeros(shape=[6, 0])
    for i in range(6):
        Q = np.array([
        # ---Initial---
            [x0[i]],
            [0],
            [0],
        # ---Final---
            [x1[i]],
            [0],
            [0]
        ])
        C = np.append(C, np.linalg.lstsq(T,Q, rcond=None)[0], axis=1)
        
    tt = np.array([np.ones(t.shape), t, t**2, t**3, t**4, t**5]).T
    
    xRef   = tt @ np.array([1*C[0], 1*C[1], 1*C[2], 1*C[3],  1*C[4],  1*C[5]])
    xDoTRef  = tt @ np.array([0*C[0], 1*C[1], 2*C[2], 3*C[3],  4*C[4],  5*C[5]])
    xDotDoTRef = tt @ np.array([0*C[0], 0*C[1], 2*C[2], 6*C[3], 12*C[4], 20*C[5]])

    return xRef, xDoTRef, xDotDoTRef

def QuinticJointTraj(q0: np.ndarray, q1: np.ndarray, t: np.ndarray):
    # Matrix T
    T = np.array([
    # ---Initial---
        [1,     min(t),     min(t)**2,      min(t)**3,      min(t)**4,          min(t)**5       ],
        [0,     1,          2*min(t),       3*min(t)**2,    4*min(t)**3,        5*min(t)**4     ],
        [0,     0,          2,              6*min(t),       12*min(t)**2,       20*min(t)**3    ],
    # ---Final---
        [1,     max(t),     max(t)**2,      max(t)**3,      max(t)**4,          max(t)**5       ],
        [0,     1,          2*max(t),       3*max(t)**2,    4*max(t)**3,        5*max(t)**4     ],
        [0,     0,          2,              6*max(t),       12*max(t)**2,       20*max(t)**3    ]
    ])
    # Matrix of coefficients
    C = np.zeros(shape=[6, 0])
    for i in range(len(q0)):
        Q = np.array([
        # ---Initial---
            [q0[i]],
            [0],
            [0],
        # ---Final---
            [q1[i]],
            [0],
            [0]
        ])

        C = np.append(C, np.linalg.lstsq(T,Q, rcond=None)[0], axis=1)

    tt = np.array([np.ones(t.shape), t, t**2, t**3, t**4, t**5]).T
    
    qRef   = tt @ np.array([1*C[0], 1*C[1], 1*C[2], 1*C[3],  1*C[4],  1*C[5]])
    qDoTRef  = tt @ np.array([0*C[0], 1*C[1], 2*C[2], 3*C[3],  4*C[4],  5*C[5]])
    qDotDoTRef = tt @ np.array([0*C[0], 0*C[1], 2*C[2], 6*C[3], 12*C[4], 20*C[5]])

    return qRef, qDoTRef, qDotDoTRef