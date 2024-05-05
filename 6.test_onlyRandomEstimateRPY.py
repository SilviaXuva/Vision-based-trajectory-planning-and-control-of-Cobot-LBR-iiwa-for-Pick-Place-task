import numpy as np
deg = np.pi/180; rad = 180/np.pi

from Helpers.paths import Paths
import os
Paths.execution = fr'{Paths.output}\{os.path.splitext(os.path.basename(__file__))[0]}\{os.path.splitext(os.path.basename(Paths.execution))[0]}'
os.makedirs(Paths.execution, exist_ok=True)
from Helpers.log import Log

from Data.targets import GetArucoPickPlace
from Data.transformations import PoseToCart, GetDot
from Models import DH_LBR_iiwa
from Helpers.measures import Real, Ref
from Helpers.input import Motion, Cuboids as Cb, Aruco
from Helpers.Analysis.plotRobot import plotOutputs as plotOutputsRobot
from Helpers.Analysis.plotVision import plotOutputs as plotOutputsVision
from Kinematics.control import JointSpaceController, IsCloseToTarget
from Kinematics.trajectory import TrajectoryPlanning
from Simulators import CoppeliaSim
from Simulators.CoppeliaSim import Drawing, RobotiqGripper, Camera, Conveyor, Cuboids
from VisionProcessing.aruco import ArucoVision

Cb.Create.max = 5
Aruco.estimatedRpy = True

robot = DH_LBR_iiwa()
coppelia = CoppeliaSim(scene='3.test_onlyRandom.ttt')
coppelia.Drawing = Drawing()
coppelia.Gripper = RobotiqGripper()
coppelia.Camera = Camera()
coppelia.ArucoVision = ArucoVision(coppelia.Camera)
coppelia.Conveyor = Conveyor()
coppelia.Cuboids = Cuboids()

robot = coppelia.Start(robot)
count = 0
coppelia.Step()

while coppelia.Cuboids.CheckToHandle():
    if len(coppelia.ArucoVision.detected) == 0:
        coppelia.Step()
    else:
        marker = coppelia.ArucoVision.detected[0]
        if coppelia.ArucoVision.PrintEstimatedPose(marker):
            count += 1
            pickPlace = GetArucoPickPlace(robot, marker, count)
            align, pick, place, ready, initial = pickPlace
            for i, target in enumerate(pickPlace[:4]):
                Log(f'Target {target.name}', 'Position in m', target.T.t, 'Rotation in deg', target.T.rpy()*rad)
                if i > 0 and not pickPlace[i-1].success:
                    target = ready

                q0 = qRef0 = qDotRef0 = coppelia.GetJointsPosition(robot)
                traj = TrajectoryPlanning(
                    type = Motion.Trajectory.type, 
                    source = Motion.Trajectory.source, 
                    robot = robot, 
                    q0 = q0, 
                    T1 = target.T, 
                    t = target.t
                )
                target.SaveData(robot, Ref(traj.q, traj.qDot, traj.qDotDot, traj.x, traj.xDot, traj.xDotDot))
                for TRef, qRef, qDotRef, qDotDotRef, xRef, xDotRef in zip(traj.T, traj.q, traj.qDot, traj.qDotDot, traj.x, traj.xDot):
                    q = coppelia.GetJointsPosition(robot)
                    if qRef is None:
                        qRef = robot.ikine_LMS(TRef).q
                        qDotRef = GetDot([qRef], qRef0)[0]
                        qDotDotRef = GetDot([qDotRef], qDotRef0)[0]
                        qRef0 = qRef; qDotRef0 = qDotRef
                    
                    qDotControl, target.intErr = JointSpaceController(robot, Motion.Kp, Motion.Ki, target.intErr, q, qRef, qDotRef)
                    
                    qControl = q + qDotControl*Motion.ts
                    target.measures.append([
                        Real(qControl, qDotControl, None, PoseToCart(robot.fkine(qControl)), None, None),
                        Ref(qRef, None, None, xRef, None, None)
                    ])
                    target.SaveData(robot)
                    coppelia.SetJointsTargetVelocity(robot, qDotControl); coppelia.Step(xRef[:3])

                    # isCloseToTarget, e = IsCloseToTarget(robot.fkine(qControl), target.T, Motion.tol)
                    if target == pick: 
                        prox, _ = coppelia.Gripper.CheckProximity(target.GripperActuation)
                    else:
                        prox = False
                    if prox:
                        break

                coppelia.SetJointsTargetVelocity(robot, [0,0,0,0,0,0,0]); coppelia.Step(xRef[:3])
                target.success = coppelia.Gripper.HandleShape(target.GripperActuation, coppelia)
        else:
            coppelia.Step()

coppelia.Stop()

# plotOutputsRobot(Paths.execution)
# plotOutputsVision(Paths.execution)

import time
import glob
import shutil
import os
time.sleep(5)
try:
    recordingFile = glob.glob(r'*.avi')[0]
    shutil.move(recordingFile, fr'{Paths.execution}\recording.avi')
except Exception as e:
    print(e)