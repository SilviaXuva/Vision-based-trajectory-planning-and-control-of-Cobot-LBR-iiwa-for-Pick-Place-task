import cv2
import numpy as np
import os
from spatialmath import SE3
import commentjson

inputJson = commentjson.load(open(os.path.abspath(r'.\Helpers\input.jsonc')))
deg = np.pi/180

class Motion:
    ts = inputJson["Motion"]["ts"]
    tTot = inputJson["Motion"]["tTot"]
    Kp = np.array(inputJson["Motion"]["Kp"])
    Ki = np.array(inputJson["Motion"]["Ki"])
    tol = np.block([np.array(inputJson["Motion"]["tol"])[:3]*0.01, np.array(inputJson["Motion"]["tol"])[3:]*deg])
    class Trajectory:
        type = inputJson["Motion"]["Trajectory"]["type"]
        source = inputJson["Motion"]["Trajectory"]["source"]
    controller = inputJson["Motion"]["controller"]

class Camera:
    sensorPath = inputJson["Camera"]["sensorPath"]
    distortionCoefficients = inputJson["Camera"]["distortionCoefficients"]
    frameRotation = SE3.RPY(
        inputJson["Camera"]["FrameRotation"]["roll"]*deg,
        inputJson["Camera"]["FrameRotation"]["pitch"]*deg,
        inputJson["Camera"]["FrameRotation"]["yaw"]*deg
    )

class Aruco:
    dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, inputJson["Aruco"]["dict"]))
    detectParam = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dict, detectParam)
    length = inputJson["Aruco"]["length"]
    estimateParam = cv2.aruco.EstimateParameters()
    estimateParam.pattern = cv2.aruco.ARUCO_CCW_CENTER
    estimateParam.useExtrinsicGuess = False
    estimateParam.solvePnPMethod = 8 
    points = np.array([
        [[-length/2, length/2, 0]],
        [[length/2, length/2, 0]],
        [[length/2, -length/2, 0]],
        [[-length/2, -length/2, 0]]
    ])
    estimatedRpy = inputJson["Aruco"]["estimatedRpy"]
    saveData = inputJson["Aruco"]["saveData"]

class Gripper:
    rotation = SE3.RPY(
        inputJson["Gripper"]["Rotation"]["roll"]*deg,
        inputJson["Gripper"]["Rotation"]["pitch"]*deg,
        inputJson["Gripper"]["Rotation"]["yaw"]*deg
    )
    increaseHeight = inputJson["Gripper"]["increaseHeight"]
    class Proximity:
        tol = inputJson["Gripper"]["Proximity"]["tol"]
        class JointsTol:
            first = inputJson["Gripper"]["Proximity"]["JointsTol"]["first"]
            second = inputJson["Gripper"]["Proximity"]["JointsTol"]["second"]
    class Actuation:
        time = inputJson["Gripper"]["Actuation"]["time"]
        class Close:
            class FirstLowerSecond:
                decreaseSecond = inputJson["Gripper"]["Actuation"]["Close"]["FirstLowerSecond"]["decreaseSecond"]
                vel1 = inputJson["Gripper"]["Actuation"]["Close"]["FirstLowerSecond"]["vel1"]
                vel2 = inputJson["Gripper"]["Actuation"]["Close"]["FirstLowerSecond"]["vel2"]
            class Other:
                vel1 = inputJson["Gripper"]["Actuation"]["Close"]["Other"]["vel1"]
                vel2 = inputJson["Gripper"]["Actuation"]["Close"]["Other"]["vel2"]
        class Open:
            class FirstLowerSecond:
                vel1 = inputJson["Gripper"]["Actuation"]["Open"]["FirstLowerSecond"]["vel1"]
                vel2 = inputJson["Gripper"]["Actuation"]["Open"]["FirstLowerSecond"]["vel2"]
            class Other:
                vel1 = inputJson["Gripper"]["Actuation"]["Open"]["Other"]["vel1"]
                vel2 = inputJson["Gripper"]["Actuation"]["Open"]["Other"]["vel2"]

class Drawing:
    ref = inputJson["Drawing"]["ref"]
    real = inputJson["Drawing"]["real"]
    cyclic = inputJson["Drawing"]["cyclic"]

class Conveyor:
    path = inputJson["Conveyor"]["path"]
    tol = inputJson["Conveyor"]["tol"]
    vel = inputJson["Conveyor"]["vel"]

class Cuboids:
    path = inputJson["Cuboids"]["path"]
    bodyPath = inputJson["Cuboids"]["bodyPath"]
    markerPath = inputJson["Cuboids"]["markerPath"]
    colors = inputJson["Cuboids"]["colors"]
    class ToHandle:
        z = inputJson["Cuboids"]["ToHandle"]["z"]
        tol = inputJson["Cuboids"]["ToHandle"]["tol"]
    class Create:
        max = inputJson["Cuboids"]["Create"]["max"]
        x = inputJson["Cuboids"]["Create"]["x"]
        z = inputJson["Cuboids"]["Create"]["z"]
        rx = inputJson["Cuboids"]["Create"]["rx"]
        ry = inputJson["Cuboids"]["Create"]["ry"]
        class Random:
            class id:
                min = inputJson["Cuboids"]["Create"]["Random"]["id"]["min"]
                max = inputJson["Cuboids"]["Create"]["Random"]["id"]["max"]
            class y:
                min = inputJson["Cuboids"]["Create"]["Random"]["y"]["min"]
                max = inputJson["Cuboids"]["Create"]["Random"]["y"]["max"]
            class rz:
                min = inputJson["Cuboids"]["Create"]["Random"]["rz"]["min"]*deg
                max = inputJson["Cuboids"]["Create"]["Random"]["rz"]["max"]*deg
            class mass:
                min = inputJson["Cuboids"]["Create"]["Random"]["mass"]["min"]
                max = inputJson["Cuboids"]["Create"]["Random"]["mass"]["max"]

class ProximitySensor:
    path = inputJson["ProximitySensor"]["path"]
