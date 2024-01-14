from Helpers.Analysis.plotVision import plotOutputs
from Helpers.log import Log, Paths
from Helpers.input import Aruco
from Models import DH_LBR_iiwa
from Simulators import CoppeliaSim
from Simulators.CoppeliaSim import Drawing, RobotiqGripper, Camera, Conveyor, Cuboids
from VisionProcessing.aruco import ArucoVision

robot = DH_LBR_iiwa()
coppelia = CoppeliaSim(scene='2.test_vision.ttt')
coppelia.Drawing = Drawing()
coppelia.Gripper = RobotiqGripper()
coppelia.Camera = Camera()
coppelia.ArucoVision = ArucoVision(coppelia.Camera)
coppelia.Conveyor = Conveyor()
coppelia.Cuboids = Cuboids()

robot = coppelia.Start(robot)

useExtrinsicGuessOptions = [True, False]
solvePnPMethodOptions = range(9)
for useExtrinsicGuess in useExtrinsicGuessOptions:
    Aruco.estimateParam.useExtrinsicGuess = useExtrinsicGuess
    for solvePnPMethod in solvePnPMethodOptions:
        Aruco.estimateParam.solvePnPMethod = solvePnPMethod
        Log('ArUco parameters: ')
        Log('Use Extrinsic Guess: ', useExtrinsicGuess)
        Log('Solve PNP Method: ', solvePnPMethod)
        for i in range(10):
            for marker in coppelia.ArucoVision.allDetected:
                coppelia.ArucoVision.PrintEstimatedPose(marker)
            coppelia.Step()

coppelia.Stop()

plotOutputs(Paths.execution)