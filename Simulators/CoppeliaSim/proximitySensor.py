from Helpers.input import ProximitySensor as PS
from Simulators.CoppeliaSim.objects import CoppeliaObj

class ProximitySensor(CoppeliaObj):
    def __init__(self) -> None:
        super().__init__('Proximity Sensor', False)
        self.handle = self.sim.getObjectHandle(PS.path)
        self.lastProx = True
    
    def CheckProximity(self):
        prox = self.sim.readProximitySensor(self.handle)
        if prox[0] == 1:
            self.prox = True
            return self.prox
        self.prox = False
        return self.prox
    
    def CallCuboidCreation(self):
        if self.lastProx == True and self.prox == False:
            create = True
        else:
            create = False
        self.lastProx = self.prox
        return create
