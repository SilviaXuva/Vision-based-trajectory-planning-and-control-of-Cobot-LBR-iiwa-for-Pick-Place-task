from Helpers.log import Log
from Helpers.input import Camera as Cam
from Simulators.CoppeliaSim.objects import CoppeliaObj

import cv2
import numpy as np
from spatialmath import SE3

class Camera(CoppeliaObj):
    def __init__(self) -> None:
        CoppeliaObj.__init__(self, 'Camera', False)
        self.handle = self.sim.getObject(Cam.sensorPath)
        self.GetParameters()
        self.GetIntrinsicMatrix()
        self.GetExtrinsicMatrix()
        self.GetImg()
        self.ShowImg()

    def GetParameters(self):
        self.resX, self.resY = self.sim.getVisionSensorRes(self.handle)
        self.perspectiveAngle = self.sim.getObjectFloatParam(self.handle, self.sim.visionfloatparam_perspective_angle)
        self.fovX = self.fovY = self.perspectiveAngle
        self.resX, self.resY, self.fovX, self.fovY

    def GetIntrinsicMatrix(self):
        cx = self.resX/2
        cy = self.resY/2
        fx = cx/np.tan(self.fovX/2)
        fy = cy/np.tan(self.fovY/2)
        self.intrinsicMatrix = np.array([
            [fx, 0,  cx],
            [0,  fy, cy],
            [0,  0,  1 ]
        ])
        self.distortionCoefficients = Cam.distortionCoefficients
        Log('Camera intrinsic matrix:', self.intrinsicMatrix)
        Log('Distortion coefficients:', self.distortionCoefficients) 

    def GetExtrinsicMatrix(self):
        pos = self.sim.getObjectPosition(self.handle, self.sim.handle_world)
        abg = self.sim.getObjectOrientation(self.handle, self.sim.handle_world)
        ypr = self.sim.alphaBetaGammaToYawPitchRoll(abg[0], abg[1], abg[2])
        rpy = [ypr[0], ypr[1], ypr[2]]
        worldT = SE3.Trans(pos)*SE3.RPY(rpy, order='zyx')
        Log('Camera pose in world frame:', worldT.A)
        self.extrinsicMatrix = (worldT*Cam.frameRotation).A
        Log('Camera extrinsic matrix:', self.extrinsicMatrix)

    def GetImg(self):
        frame, resX, resY = self.sim.getVisionSensorCharImage(self.handle)
        frame = np.frombuffer(frame, dtype=np.uint8).reshape(resY, resX, 3)
        self.frame = cv2.flip(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), 0)

    def ShowImg(self):
        cv2.imshow('Frame', self.frame)
        cv2.waitKey(1)  
