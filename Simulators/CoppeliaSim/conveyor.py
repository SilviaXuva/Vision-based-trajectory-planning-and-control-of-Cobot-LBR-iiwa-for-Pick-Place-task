from Helpers.input import Conveyor as Conv
from Simulators.CoppeliaSim.objects import CoppeliaObj

import math

class Conveyor(CoppeliaObj):
    def __init__(self) -> None:
        super().__init__('Conveyor', False)
        self.handle = self.sim.getObjectHandle(Conv.path)
        self.Move(Conv.vel)

    def Move(self, vel: float):
        self.sim.writeCustomTableData(self.handle,'__ctrl__',{"vel":vel})
    
    def GetVel(self):
        vel = self.sim.readCustomTableData(self.handle,'__state__')['vel']
        return vel

    def CheckStopped(self):
        vel = self.sim.readCustomTableData(self.handle,'__state__')['vel']
        if math.isclose(vel, 0, abs_tol=Conv.tol):
            return True
        return False