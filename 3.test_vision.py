from Helpers.Analysis.plotVision import plotOutputs
from Helpers.log import Paths
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

for i in range(50):
    for marker in coppelia.ArucoVision.allDetected:
        coppelia.ArucoVision.PrintEstimatedPose(marker)
    coppelia.Step()

coppelia.Stop()

plotOutputs(Paths.execution)