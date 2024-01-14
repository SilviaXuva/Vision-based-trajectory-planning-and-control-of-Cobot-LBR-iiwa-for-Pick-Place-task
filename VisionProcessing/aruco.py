from Data.pose import Pose
from Helpers.meanSquareError import MeanSquareError
from Helpers.log import Log, Paths
from Helpers.input import Aruco as Ar
from VisionProcessing.colorBasedFilters import MaskRanges, GetGray
from VisionProcessing.guiFeatures import WriteText, DrawingFrame

import cv2
import numpy as np
from spatialmath import SE3
import os
import pandas as pd

rad = 180/np.pi

class Marker:
    def __init__(
            self, 
            id: int, 
            color: np.ndarray,            
            corners:np.ndarray 
        ) -> None:
        self.id = id
        self.color = color        
        self.corners = corners

class ArucoVision():
    def __init__(self, Camera) -> None:
        Log(f'Init Aruco Vision...')
        self.Camera = Camera
        self.intrinsicMatrix = Camera.intrinsicMatrix    
        self.distortionCoefficients = Camera.distortionCoefficients  
        self.extrinsicMatrix = Camera.extrinsicMatrix
        self.allDetected = [] # Detected in entire execution
        self.Process()

    def Process(self):
        self.detected = [] # Detected in current frame
        self.PreProcessing()
        self.Detect()
        self.Draw()
        self.EstimateArucoPose()
        self.ShowImg()
    
    def ShowImg(self):
        cv2.imshow('Processed', self.processedFrame)
        cv2.imshow('Draw', self.drawFrame)
        cv2.waitKey(1)  

    def GetDetected(self, id: int, color: str):
        for i, marker in enumerate(self.allDetected):
            if marker.id == id and marker.color == color:
                return i
        return None

    def UpdateDetected(self, marker: Marker):
        index = self.GetDetected(marker.id, marker.color)
        if index is None:
            self.allDetected.append(Marker(marker.id, marker.color, marker.corners))
        else:        
            for key, value in marker.__dict__.items():
                setattr(self.allDetected[index], key, value)
            if not hasattr(self.allDetected[index], 'objectWorldTList'):
                self.allDetected[index].objectWorldTList = []
                self.allDetected[index].realObjectWorldTList = []
            if hasattr(marker, 'objectWorldT'):
                self.allDetected[index].objectWorldTList.append(marker.objectWorldT)
                self.allDetected[index].realObjectWorldTList.append(marker.realObjectWorldT)

    def PreProcessing(self):
        self.processedFrame = self.Camera.frame.copy()
        self.processedFrame, _ = MaskRanges(self.processedFrame)
        self.processedFrame = GetGray(self.processedFrame)

    def Detect(self):
        corners, ids, rejectedCandidates = Ar.detector.detectMarkers(self.processedFrame)
        
        if len(corners) > 0: # If markers are detected
            for (markerCorners, markerId) in zip(corners, ids):
                min_y = int(min(markerCorners[0][:,1])); max_y = int(max(markerCorners[0][:,1]))
                min_x = int(min(markerCorners[0][:,0])); max_x = int(max(markerCorners[0][:,0]))
                roi = self.Camera.frame[min_y:max_y, min_x:max_x]
                _, colors = MaskRanges(roi)
                markerId = int(markerId[0]); markerColor = colors[0] if len(colors) > 0 else None
                self.detected.append(Marker(markerId, markerColor, markerCorners))

                self.UpdateDetected(self.detected[-1])
    
    def Draw(self):
        self.drawFrame = self.Camera.frame.copy()
        corners = [marker.corners for marker in self.detected]
        cv2.aruco.drawDetectedMarkers(self.drawFrame, corners) # Draw a square around the markers
        
        for marker in self.detected:
            min_x = int(min(marker.corners[0][:,1])); max_x = int(max(marker.corners[0][:,1]))
            min_y = int(min(marker.corners[0][:,0])); max_y = int(max(marker.corners[0][:,0]))                
            x = int(max_x + (max_x - min_x)/2)
            y = int(max_y)
            WriteText(self.drawFrame, marker.id, (y, x), thickness=3)
            
    def GetArucoRealPose(self, marker: Marker):
        def TransformCoppeliaMatrix(matrixArray):
            matrix = SE3(np.array(
                np.vstack([
                    np.array(matrixArray).reshape(3,4), 
                    np.array([0,0,0,1])
                ])))
            return matrix
        
        try:
            mass = self.Camera.sim.getShapeMass(self.Camera.sim.getObject(f'./{marker.color}{marker.id}'))
            objectCameraT = TransformCoppeliaMatrix(
                self.Camera.sim.getObjectMatrix(self.Camera.sim.getObject(f'./marker{marker.id}'), self.Camera.sim.getObject('./cameraFrame'))
            )   
            objectWorldT = TransformCoppeliaMatrix(
                self.Camera.sim.getObjectMatrix(self.Camera.sim.getObject(f'./marker{marker.id}'), self.Camera.sim.handle_world)
            )
            real = [mass, objectCameraT, objectWorldT]
        except:
            real = [None, None, None]
        return real
    
    def EstimateArucoPose(self):
        for i, marker in enumerate(self.detected):
            try:
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(
                    marker.corners, 
                    Ar.length, 
                    self.intrinsicMatrix, 
                    self.distortionCoefficients,
                    estimateParameters=Ar.estimateParam
                )
                DrawingFrame(
                    self.drawFrame, 
                    self.intrinsicMatrix,
                    self.distortionCoefficients,
                    rvec,
                    tvec,
                    Ar.length
                )

                rmat, _ = cv2.Rodrigues(rvec)

                # Marker pose to camera frame
                objectCameraT = SE3(np.vstack([np.hstack([rmat, np.reshape(tvec,(3,1))]), np.array([0, 0, 0, 1])]))

                # Marker pose to world frame
                objectWorldT = SE3(self.extrinsicMatrix@objectCameraT.A)
                
                T = Pose(
                    x = objectWorldT.t[0], 
                    y = objectWorldT.t[1], 
                    z = objectWorldT.t[2]-(Ar.length/2), 
                    rpy = objectWorldT.rpy() if Ar.estimatedRpy else [0,0,objectWorldT.rpy()[-1]]
                )
                
                real = self.GetArucoRealPose(marker)

                self.detected[i].T = T; 
                self.detected[i].objectCameraT = objectCameraT; self.detected[i].objectWorldT = objectWorldT
                self.detected[i].realObjectCameraT = real[1]; self.detected[i].realObjectWorldT = real[2]
                self.detected[i].mass = real[0]

                self.UpdateDetected(self.detected[i])
            except:
                pass

    def PrintEstimatedPose(self, marker: Marker):
        if hasattr(marker, 'T'):
            if not hasattr(marker, ''):
                real = self.GetArucoRealPose(marker)
                marker.mass = real[0]
                marker.objectWorldT = marker.T
                marker.realObjectWorldT = real[2]
            Log(f'========== ID: {marker.id}__Color: {marker.color} ============')
            Log('Shape mass:', marker.mass)
            Log('---------- Object to world ------------')
            Log('---------- r ------------')
            Log('Estimated:', marker.objectWorldT.rpy()*rad)
            Log('Real:', marker.realObjectWorldT.rpy()*rad)
            Log('Error:', MeanSquareError(marker.objectWorldT.rpy(), marker.realObjectWorldT.rpy()))
            Log('---------- t ------------')
            Log('Estimated:', marker.objectWorldT.t)
            Log('Real:', marker.realObjectWorldT.t)
            Log('Error:', MeanSquareError(marker.objectWorldT.t, marker.realObjectWorldT.t))
            if Ar.saveData:
                self.SaveData(self.allDetected[self.GetDetected(marker.id, marker.color)])
            return True
        else:
            Log(f'Could not estimate marker {marker.color}{marker.id} pose')
            return False

    def SaveData(self, marker: Marker):
        if hasattr(marker, 'objectWorldTList'):
            path = fr'{Paths.execution}\Vision\{Ar.estimateParam.useExtrinsicGuess}-{str(Ar.estimateParam.solvePnPMethod)}\{marker.color}{marker.id}_{marker.mass}\Data'
            os.makedirs(path, exist_ok=True)
            for dataType in [['Real', marker.objectWorldTList], ['Ref', marker.realObjectWorldTList]]:
                data = [np.block([T.t, T.rpy()*np.pi/180]) for T in dataType[1] if T is not None]
                df = pd.DataFrame(data)
                df.columns = ['x', 'y', 'z', 'rx', 'ry', 'rz']
                df.to_csv(fr'{path}\markerPose_{dataType[0]}.csv')
            data = [np.block([T_Ref.t, T_Ref.rpy()*np.pi/180]) - np.block([T_Real.t, T_Real.rpy()*np.pi/180]) for T_Real, T_Ref in zip(marker.objectWorldTList, marker.realObjectWorldTList) if T_Real is not None and T_Ref is not None]
            df = pd.DataFrame(data)
            df.columns = ['x', 'y', 'z', 'rx', 'ry', 'rz']
            df.to_csv(fr'{path}\markerPose_Err.csv')