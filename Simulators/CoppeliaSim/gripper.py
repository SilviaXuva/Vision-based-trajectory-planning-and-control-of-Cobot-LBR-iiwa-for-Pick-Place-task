import numpy as np
import math

from Helpers.log import Log
from Helpers.input import Gripper as Grp, Motion
from Simulators.CoppeliaSim.objects import CoppeliaObj

class Actuation:
    def __init__(self, actuation: str = None, shapePath: str = None) -> None:
        if actuation == 'close':
            self.close = True
        else:
            self.close = False
        self.shapePath = shapePath
    
    def Log(self):
        return self.__dict__

class RobotiqGripper(CoppeliaObj):
    def __init__(self) -> None:
        super().__init__('RobotiqGripper', False)
        self.simIK = self.client.require('simIK')

        self.j1 = self.sim.getObject('./active1'); self.j1Opened = self.sim.getJointPosition(self.j1)
        self.j2 = self.sim.getObject('./active2'); self.j2Opened = self.sim.getJointPosition(self.j2)
        self.simBase = self.sim.getObject('./ROBOTIQ85')
        self.simTip1 = self.sim.getObject('./LclosureDummyA')
        self.simTarget1 = self.sim.getObject('./LclosureDummyB')
        self.simTip2 = self.sim.getObject('./RclosureDummyA')
        self.simTarget2 = self.sim.getObject('./RclosureDummyB')
        self.CreateIkEnv()

        self.connector = self.sim.getObject('./attachPoint')
        self.objectSensor = self.sim.getObject('./attachProxSensor')
        self.close = False

    def CreateIkEnv(self):
        self.ikEnv = self.simIK.createEnvironment()
        self.ikGroup1 = self.simIK.createGroup(self.ikEnv)
        self.simIK.addElementFromScene(
            self.ikEnv,
            self.ikGroup1,
            self.simBase,self.simTip1,self.simTarget1,
            self.simIK.constraint_x+self.simIK.constraint_z
        )
        self.ikGroup2 = self.simIK.createGroup(self.ikEnv)
        self.simIK.addElementFromScene(
            self.ikEnv,
            self.ikGroup2,
            self.simBase,self.simTip2,self.simTarget2,
            self.simIK.constraint_x+self.simIK.constraint_z
        )
    
    def CheckProximity(self, Actuation: Actuation):
        prox = self.sim.checkProximitySensor(self.objectSensor, self.sim.getObject(Actuation.shapePath))
        isProx = True if prox[0] == 1 else False
        isProxClose = math.isclose(prox[1], 0, abs_tol=Grp.Proximity.tol)
        j1Pos = self.sim.getJointPosition(self.j1); isJ1Opened = math.isclose(j1Pos, self.j1Opened, abs_tol=Grp.Proximity.JointsTol.first)
        j2Pos = self.sim.getJointPosition(self.j2); isJ2Opened = math.isclose(j2Pos, self.j2Opened, abs_tol=Grp.Proximity.JointsTol.second)
        return isProx and isProxClose and isJ1Opened and isJ2Opened, prox
        
    def Actuation(self):
        p1 = self.sim.getJointPosition(self.j1)
        p2 = self.sim.getJointPosition(self.j2)
        if self.close:
            if (p1<p2-Grp.Actuation.Close.FirstLowerSecond.decreaseSecond):
                self.sim.setJointTargetVelocity(self.j1, Grp.Actuation.Close.FirstLowerSecond.vel1)
                self.sim.setJointTargetVelocity(self.j2, Grp.Actuation.Close.FirstLowerSecond.vel2)
            else:
                self.sim.setJointTargetVelocity(self.j1, Grp.Actuation.Close.Other.vel1)
                self.sim.setJointTargetVelocity(self.j2, Grp.Actuation.Close.Other.vel2)
        else:
            if (p1<p2):
                self.sim.setJointTargetVelocity(self.j1, Grp.Actuation.Open.FirstLowerSecond.vel1)
                self.sim.setJointTargetVelocity(self.j2, Grp.Actuation.Open.FirstLowerSecond.vel2)
            else:
                self.sim.setJointTargetVelocity(self.j1, Grp.Actuation.Open.Other.vel1)
                self.sim.setJointTargetVelocity(self.j2, Grp.Actuation.Open.Other.vel2)
        self.ApplyIk()
    
    def ApplyIk(self):
        self.simIK.handleGroup(self.ikEnv, self.ikGroup1,{'syncWorlds': True})
        self.simIK.handleGroup(self.ikEnv, self.ikGroup2,{'syncWorlds': True})

    def Cleanup(self):
        self.simIK.eraseEnvironment(self.ikEnv)
    
    def HandleShape(self, Actuation: Actuation, coppelia):
        self.close = Actuation.close
        if Actuation.shapePath is None:
            return True
        else:
            shape = self.sim.getObject(Actuation.shapePath)
            if not Actuation.close:
                ret = True if self.sim.setObjectParent(shape, -1, True) == 1 else False
            else:
                _, value = self.CheckProximity(Actuation)
                ret = True if value[0] == 1 and self.sim.setObjectParent(shape, self.connector, True) == 1 else False
            if ret:
                for i in np.arange(0, Grp.Actuation.time + Motion.ts, Motion.ts):
                    self.Actuation()
                    coppelia.Step()
            self.Cleanup()
            self.CreateIkEnv()
            return ret

class GripperChildScript:
    def __init__(self, client, sim, gripper_name = './ROBOTIQ85'):
        Log('Init Gripper...')
        self.client = client
        self.sim = sim
        
        handle = self.sim.getObject(gripper_name)
        self.script_handle = self.sim.getScript(self.sim.scripttype_childscript, handle)
        self.connector = self.sim.getObject('./attachPoint')
        self.objectSensor = self.sim.getObject('./attachProxSensor')
    
    def actuation(self, close):
        self.sim.callScriptFunction('Actuation', self.script_handle, close)
        
    def open(self):
        self.sim.setObjectParent(self.shape, -1, True)
        self.actuation(False)
        self.client.step()
        
    def close(self, shape_name = './Cuboid'):
        self.actuation(True)
        self.shape = self.sim.getObject(shape_name)
        if self.sim.checkProximitySensor(self.objectSensor,self.shape)[0] == 1:
            self.sim.setObjectParent(self.shape, self.connector, True)
        self.client.step()
