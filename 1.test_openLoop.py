import numpy as np
from Helpers.paths import Paths
import os
currentFile = os.path.splitext(os.path.basename(__file__))[0]
Paths.execution = fr'{Paths.output}\{currentFile}\{Paths.startTime}'
os.makedirs(Paths.execution, exist_ok=True)
from Helpers.log import Log
from Data.targets import GetArucoPickPlace
from Data.transformations import PoseToCart, GetDot
from Helpers.cleanup import CleanUp
from Helpers.input import Motion
from Helpers.measures import Real, Ref
from Kinematics.control import JointSpaceController, IsCloseToTarget
from Kinematics.trajectory import TrajectoryPlanning
from Models import DH_LBR_iiwa
from Simulators import CoppeliaSim
from Simulators.CoppeliaSim import Drawing, RobotiqGripper, Cuboids

from Data.openLoop import red, blue, green

deg = np.pi/180; rad = 180/np.pi

robot = DH_LBR_iiwa()
coppelia = CoppeliaSim(scene='1.test_openLoop.ttt')
coppelia.Drawing = Drawing()
coppelia.Gripper = RobotiqGripper()
coppelia.Cuboids = Cuboids(maxCreation=0)

robot = coppelia.Start(robot)
count = 0
coppelia.Step()

while coppelia.Cuboids.CheckToHandle():
    for marker in [red, green, blue]:
        pickPlace = GetArucoPickPlace(robot, marker, count); count += 1
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
            target.SaveData(robot)
            
coppelia.Stop()

CleanUp()