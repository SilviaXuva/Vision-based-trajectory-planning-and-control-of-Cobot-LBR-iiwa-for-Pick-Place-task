import numpy as np
deg = np.pi/180

from Data.targets import GetArucoPickPlace
from Data.transformations import PoseToCart, GetDot
from Models import DH_LBR_iiwa
from Helpers.measures import Real, Ref
from Helpers.input import Motion, Cuboids as Cb, Aruco
from Helpers.log import Log
from Helpers.Analysis.plotRobot import plotOutputs as plotOutputsRobot
from Helpers.Analysis.plotVision import plotOutputs as plotOutputsVision
from Kinematics.control import JointSpaceController, IsCloseToTarget
from Kinematics.trajectory import TrajectoryPlanning
from Simulators import CoppeliaSim
from Simulators.CoppeliaSim import Drawing, RobotiqGripper, Camera, Conveyor, Cuboids
from VisionProcessing.aruco import ArucoVision

robot = DH_LBR_iiwa()
coppelia = CoppeliaSim(scene='3.test_onlyRandom.ttt')
coppelia.Drawing = Drawing()
coppelia.Gripper = RobotiqGripper()
coppelia.Camera = Camera()
coppelia.ArucoVision = ArucoVision(coppelia.Camera)
coppelia.Conveyor = Conveyor()
coppelia.Cuboids = Cuboids()

Aruco.estimatedRpy = True

robot = coppelia.Start(robot)
count = 0
coppelia.Step()
stop = False
obj = 'green0'
rx = 0; ry = 0; rz = 60*deg

while coppelia.Cuboids.CheckToHandle():
    # try:
    #     handle = coppelia.sim.getObject(f'./{obj}')
    #     coppelia.sim.setObjectOrientation(handle, -1, [rx, ry, rz])
    #     coppelia.sim.step()
    #     coppelia.Step()
    # except:
    #     pass
    if len(coppelia.ArucoVision.detected) == 0:
        coppelia.Step()
    else:
        marker = coppelia.ArucoVision.detected[0]
        if coppelia.ArucoVision.PrintEstimatedPose(marker):
            count += 1
            pickPlace = GetArucoPickPlace(robot, marker, count)
            align, pick, place, ready, initial = pickPlace
            for i, target in enumerate(pickPlace[:4]):
                Log(f'Target {target.name}', target.T.t, target.T.rpy())
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

coppelia.Stop()

# plotOutputsRobot(Paths.execution)
# plotOutputsVision(Paths.execution)