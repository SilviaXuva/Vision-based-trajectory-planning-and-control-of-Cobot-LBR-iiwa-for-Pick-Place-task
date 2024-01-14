from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import os
from roboticstoolbox import DHRobot, ERobot

from Helpers.log import Log
from Simulators.CoppeliaSim import ClearDrawing
from Simulators.sim import Sim

scenesPath = os.path.abspath(r'.\Scenes')

class CoppeliaSim(Sim):
    def __init__(self,
            scene: str = 'main.ttt'
        ) -> None:
        Sim.__init__(self, 'Coppelia')

        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        
        while self.sim.getSimulationState() != self.sim.simulation_stopped:
            self.Stop()
        
        Log(f'Loading scene {scene}')
        self.sim.loadScene(fr'{scenesPath}\{scene}')

    def Start(self, robot: DHRobot | ERobot):
        Log(f'Starting {self.name} simulation')
        robot.handle = self.GetRobotHandle(robot)
        robot.joints = self.GetJoints(robot)
        self.LockJoints(robot)
        self.sim.setInt32Param(self.sim.intparam_idle_fps, 0)
        self.sim.setStepping(True)
        self.sim.startSimulation()
        self.Step()
        self.started = True
        Log(f'{self.name} simulation started')
        return robot

    def Stop(self):
        Log(f'Stopping {self.name} simulation')
        ClearDrawing(self.sim)
        self.sim.stopSimulation()
        self.sim.setInt32Param(self.sim.intparam_idle_fps, 8)
        self.started = False
        Log(f'{self.name} simulation stopped')

    def GetRobotHandle(self, robot: DHRobot | ERobot):
        return self.sim.getObject(f'./{robot.name}')

    def GetJoints(self, robot: DHRobot | ERobot):
        while True:
            try:
                joints = list()
                for i in range(0, robot.n):
                    joints.append(self.sim.getObject(f'./joint{i}'))
                break
            except:
                pass
        return joints

    def SetJointsControlMode(self, robot: DHRobot | ERobot, type):
        if type == 'position':
            param = self.sim.jointdynctrl_position
        elif type == 'velocity':
            param = self.sim.jointdynctrl_velocity
            self.LockJoints(robot)
        for joint in robot.joints:
            self.sim.setObjectInt32Param(joint, self.sim.jointintparam_dynctrlmode, param)

    def LockJoints(self, robot: DHRobot | ERobot):
        for joint in robot.joints:
            self.sim.setObjectInt32Param(joint, self.sim.jointintparam_velocity_lock, 1)

    def GetJointsPosition(self, robot: DHRobot | ERobot):
        q = []
        for joint in robot.joints:
            q.append(self.sim.getJointPosition(joint))
        return np.array(q)

    def SetJointsTargetPosition(self, robot: DHRobot | ERobot, pos: list):
        for i, joint in enumerate(robot.joints):
            self.sim.setJointTargetPosition(joint, np.float64(pos[i]))

    def SetJointsTargetVelocity(self, robot: DHRobot | ERobot, vel: list):
        for i, joint in enumerate(robot.joints):
            self.sim.setJointTargetVelocity(joint, np.float64(vel[i]))

    def Step(self, xRef: np.ndarray = None): 
        if hasattr(self, 'Drawing'):
            self.Drawing.refPos = xRef
            self.Drawing.event.wait()
            self.Drawing.event.clear()
        
        if hasattr(self, 'Gripper'):
            self.Gripper.Actuation()

        if hasattr(self, 'Camera'):
            self.Camera.GetImg()
            self.Camera.ShowImg()
            
        if hasattr(self, 'ArucoVision'):
            self.ArucoVision.Process()

        if hasattr(self, 'Cuboids'):
            self.Cuboids.CheckAvailable()
            if not self.Cuboids.CheckToHandle() and self.Cuboids.created < self.Cuboids.maxCreation:
                self.Cuboids.CreateCuboid()
    
        self.client.step()