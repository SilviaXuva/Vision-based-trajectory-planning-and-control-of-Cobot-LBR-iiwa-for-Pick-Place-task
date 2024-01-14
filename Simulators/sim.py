import numpy as np
from roboticstoolbox import DHRobot, ERobot

from Helpers.log import Log

class Sim:
    def __init__(self, name: str) -> None:
        self.name = name
        self.started = False
        self.moveRobot = True
    
    def Start(self, robot: DHRobot | ERobot):
        Log(f'Starting {self.name} simulation')
        self.launch()
        self.add(robot)
        self.started = True
        Log(f'{self.name} simulation started')

    def Stop(self):
        Log(f'Stopping {self.name} simulation')
        self.close()
        self.started = False
        Log(f'{self.name} simulation stopped')
        
    def GetJointsPosition(self, robot: DHRobot | ERobot):
        return robot.q

    def SetJointsTargetPosition(self, robot: DHRobot | ERobot, pos: np.ndarray):
        robot.q = pos

    def SetJointsTargetVelocity(self, robot: DHRobot | ERobot, vel: np.ndarray):
        robot.qd = vel

    def Step(self):
        pass