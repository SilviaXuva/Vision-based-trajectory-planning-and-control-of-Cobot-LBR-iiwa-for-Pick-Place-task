from Data.pose import Pose
from Data.transformations import GetDot
from Helpers.measures import Real, Ref
from Helpers.input import Motion, Gripper
from Helpers.log import Paths
from Simulators.CoppeliaSim import Actuation
from VisionProcessing.aruco import Marker

import numpy as np
import os
import glob
import pandas as pd
from roboticstoolbox import DHRobot, ERobot
from spatialmath import SE3

deg = np.pi/180
rad = 180/np.pi

class Target():
    def __init__(self, 
            name: str, 
            T: SE3, 
            GripperActuation: Actuation, 
            tTot: np.ndarray, 
            success: bool = None
        ):
        self.name = name
        self.T = T
        self.GripperActuation = GripperActuation
        self.t = np.arange(0, tTot + Motion.ts, Motion.ts)
        self.intErr = 0
        self.measures = []
        self.success = success
        path = fr'{Paths.execution}\Motion\{name}'
        self.cartPath = fr'{path}\Cart Comparison\Data'
        self.jointPath = fr'{path}\Joint Comparison\Data'
        os.makedirs(self.cartPath, exist_ok=True); os.makedirs(self.jointPath, exist_ok=True) 

    def SaveData(self, robot: DHRobot|ERobot, traj: Ref = None):
        if traj is not None:
            self.Traj = traj
            dataTypes = ['Traj']
        else:
            self.Real = Real(**pd.DataFrame([m.__dict__ for m in np.array(self.measures)[:,0]]).to_dict(orient="list"))
            self.Ref = Ref(**pd.DataFrame([m.__dict__ for m in np.array(self.measures)[:,1]]).to_dict(orient="list"))
            dataTypes = ['Real', 'Ref']
            
        for m in [['q'], ['qDot', 'q'], ['qDotDot', 'qDot'], ['x'], ['xDot', 'x'], ['xDotDot', 'xDot']]:
            measure = m[0]
            for dataType in dataTypes:
                if len(m) == 2 and (getattr(getattr(self, dataType), m[0]) is None or any(elem is None for elem in getattr(getattr(self, dataType), m[0]))):
                    setattr(getattr(self, dataType), m[0], GetDot(getattr(getattr(self, dataType), m[1]), getattr(getattr(self, dataType), m[1])[0]))
                if dataType == 'Traj':
                    setattr(getattr(self, dataType), f'{measure}_df', pd.DataFrame(getattr(getattr(self, dataType), measure), index=np.array([i*Motion.ts for i in range(len(self.Traj.q))])))
                else:
                    setattr(getattr(self, dataType), f'{measure}_df', pd.DataFrame(getattr(getattr(self, dataType), measure), index=np.array([i*Motion.ts for i in range(len(self.measures))])))
                if 'x' in measure:
                    path = fr'{self.cartPath}\{measure}'
                    os.makedirs(path, exist_ok=True)
                    getattr(getattr(self, dataType), f'{measure}_df').columns = ['x', 'y', 'z', 'rx', 'ry', 'rz']
                    getattr(getattr(self, dataType),f'{measure}_df').to_csv(fr'{path}\{measure}_{dataType}.csv')
                else:
                    path = fr'{self.jointPath}\{measure}'
                    os.makedirs(path, exist_ok=True)
                    getattr(getattr(self, dataType), f'{measure}_df').columns = [f'q{i}' for i in range(robot.n)]
                    getattr(getattr(self, dataType),f'{measure}_df').to_csv(fr'{path}\{measure}_{dataType}.csv')

bins = {
    "green": Pose(
        x = -0.500, y = 0.600, z = 0.35, 
        r_xyz = [0,  0, 0]
    ),
    "blue": Pose(
        x = -0.500, y = 0, z = 0.35, 
        r_xyz = [0,  0, 0]
    ),
    "red": Pose(
        x = -0.500, y = -0.600, z = 0.35, 
        r_xyz = [0,  0, 0]
    )
}

def GetArucoPickPlace(robot: DHRobot|ERobot, marker: Marker, count: int):
    bin = bins[marker.color]
    # rot = marker.T.rpy()[-1]*rad
    # if rot <= -135:
    #     rotation = SE3.RPY(180*deg, 0, 0)
    # elif rot > -135 and rot <= -90:
    #     rotation = SE3.RPY(180*deg, 0, -90*deg)
    # elif rot > -90 and rot <= -45:
    #     rotation = SE3.RPY(180*deg, 0, 90*deg)
    # elif rot > -45 and rot < 0:
    #     rotation = SE3.RPY(180*deg, 0, 180*deg)
    # elif rot >= 0 and rot < 45:
    #     rotation = SE3.RPY(180*deg, 0, 0)
    # elif rot >= 45 and rot < 90:
    #     rotation = SE3.RPY(180*deg, 0, 90*deg)
    # elif rot >= 90 and rot < 135:
    #     rotation = SE3.RPY(180*deg, 0, -90*deg)
    # elif rot >= 135:
    #     rotation = SE3.RPY(180*deg, 0, 0)
    rotation = Gripper.rotation

    prefix = f'{count}.{marker.color}{marker.id}_{float("{:.3f}".format(marker.mass))}'
    pickPlace = [
        Target(
            name = fr'{prefix}\1.Align',
            T = Pose(
                x = marker.T.t[0],
                y = marker.T.t[1], 
                z = marker.T.t[2] + Gripper.increaseHeight, 
                rpy = marker.T.rpy()
            )*rotation,
            GripperActuation = Actuation(),
            tTot = Motion.tTot,
        ),
        Target(
            name = fr'{prefix}\2.Pick',
            T = Pose(
                x = marker.T.t[0],
                y = marker.T.t[1], 
                z = marker.T.t[2], 
                rpy = marker.T.rpy()
            )*rotation,
            GripperActuation = Actuation(
                actuation = 'close', 
                shapePath = f'./{marker.color}{marker.id}'
            ),
            tTot = Motion.tTot,
        ),
        Target(
            name = fr'{prefix}\3.Place',
            T = bin*Gripper.rotation,
            GripperActuation = Actuation(
                actuation = 'open', 
                shapePath = f'./{marker.color}{marker.id}'
            ),
            tTot = Motion.tTot,
        ),
        Target(
            name = fr'{prefix}\4.Ready',
            T = robot.Tr,
            GripperActuation = Actuation(),
            tTot = Motion.tTot,
        ),
        Target(
            name = fr'{prefix}\5.Initial',
            T = robot.Tz,
            GripperActuation = Actuation(),
            tTot = Motion.tTot,
        )
    ]
    return pickPlace