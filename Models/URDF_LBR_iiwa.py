import numpy as np
from roboticstoolbox.models.URDF.LBR import LBR
from spatialmath import SE3

class LBR_iiwa(LBR):
    """Class that imports a Kuka LBR iiwa 14R 820 URDF model

    ``LBR()`` is a class which imports a Kuka LBR iiwa 14R 820 robot definition
    from a URDF file.  The model describes its kinematic and graphical
    characteristics.

    .. runblock:: pycon

        >>> from Models.URDF_LBR_iiwa import LBR_iiwa
        >>> robot = LBR_iiwa()

    Defined joint configurations are:

    - qz, zero joint angle configuration, 'L' shaped configuration
    - qr, vertical 'READY' configuration
    - qs, arm is stretched out in the x-direction
    - qn, arm is at a nominal non-singular configuration

    .. codeauthor:: Jesse Haviland
    .. sectionauthor:: Peter Corke
    """
    
    def __init__(self):
        """Init LBR_iiwa URDF model"""
        deg = np.pi/180
        mm = 1e-3
        flange = 230 * mm

        super().__init__()
        
        self.name = 'LBRiiwa14R820'
        self.qr = np.array([0,0,0,90*deg,0,-90*deg,0])
        self.Tr = self.fkine(self.qr)
        self.qz = np.zeros(self.n)
        self.Tz = self.fkine(self.qz)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)

        self.q = self.qz
        self.base = SE3.Rz(np.pi)