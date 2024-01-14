from Data.targets import GetArucoPickPlace
from Data.transformations import PoseToCart, GetDot
from Models import DH_LBR_iiwa
from Helpers.measures import Real, Ref
from Helpers.input import Motion
from Helpers.log import Log, Paths
from Helpers.Analysis.plotRobot import plotOutputs
from Kinematics.control import JointSpaceController, IsCloseToTarget
from Kinematics.trajectory import TrajectoryPlanning
from Simulators import CoppeliaSim
from Simulators.CoppeliaSim import Drawing, RobotiqGripper, Cuboids
from Data.openLoop import red, blue, green

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

                isCloseToTarget, e = IsCloseToTarget(robot.fkine(qControl), target.T, Motion.tol)
                if target.GripperActuation.close: 
                    prox, value = coppelia.Gripper.CheckProximity(target.GripperActuation)
                else:
                    prox = isCloseToTarget
                    value = 0
                if isCloseToTarget or prox:
                    break

            coppelia.SetJointsTargetVelocity(robot, [0,0,0,0,0,0,0]); coppelia.Step(xRef[:3])
            target.success = coppelia.Gripper.HandleShape(target.GripperActuation, coppelia)

coppelia.Stop()

plotOutputs(Paths.execution)