from Helpers.input import Cuboids as Cb, Aruco
from Simulators.CoppeliaSim.objects import CoppeliaObj

import numpy as np
import random
import math

class Cuboids(CoppeliaObj):
    def __init__(self, maxCreation = None) -> None:
        super().__init__('Cuboids', False)
        self.handle = self.sim.getObjectHandle(Cb.path)
        self.bodyHandle = self.sim.getObjectHandle(Cb.bodyPath)
        self.colors= Cb.colors
        self.created = 0
        self.maxCreation = Cb.Create.max if maxCreation is None else maxCreation
        self.CheckAvailable()
        self.CheckToHandle()

    def CheckAvailable(self):
        self.available = []
        for color in ['red', 'blue', 'green']:
            for markerId in range(1,11):
                try:
                    obj = f'./{color}{markerId}'
                    self.sim.getObject(obj)
                    self.available.append(obj)
                except:
                    pass

    def CheckToHandle(self):
        self.toHandle = self.available.copy()
        def filterAvailable(obj):
            x, y, z = self.sim.getObjectPosition(self.sim.getObject(obj))
            if math.isclose(z, Cb.ToHandle.z, abs_tol=Cb.ToHandle.tol):
                return True
            else:
                return False
        self.toHandle = list(filter(filterAvailable, self.toHandle))
        return True if len(self.toHandle) > 0  else False

    def CreateCuboid(self):
        while True:
            markerId = random.randrange(Cb.Create.Random.id.min,Cb.Create.Random.id.max)
            colorName, color = random.choice(list(self.colors.items()))
            if not f'./{colorName}{markerId}' in self.available:
                break
        
        handle, marker, body = self.sim.copyPasteObjects(
            [self.handle, self.sim.getObjectHandle(Cb.markerPath.replace('{id}', str(markerId))), self.bodyHandle],0
        )
        self.sim.setObjectPosition(marker, handle, [0, 0, (Aruco.length/2)]); self.sim.setObjectPosition(body, handle, [0,0,0])
        self.sim.setObjectParent(marker, handle, True); self.sim.setObjectParent(body, handle)
        
        y = random.uniform(Cb.Create.Random.y.min, Cb.Create.Random.y.max); self.sim.setObjectPosition(handle, -1, [Cb.Create.x, y, Cb.Create.z])
        rz = random.uniform(Cb.Create.Random.rz.min, Cb.Create.Random.rz.max); self.sim.setObjectOrientation(handle, -1, [Cb.Create.rx, Cb.Create.ry, rz])
        mass = random.uniform(Cb.Create.Random.mass.min, Cb.Create.Random.mass.max); self.sim.setShapeMass(handle, mass)
    
        self.sim.setObjectAlias(marker, self.sim.getObjectAlias(marker).replace("ref_",""))
        self.sim.setShapeColor(marker, None, self.sim.colorcomponent_ambient_diffuse, color)

        self.sim.setObjectAlias(body, self.sim.getObjectAlias(body).replace("ref_",""))
        self.sim.setShapeColor(body, None, self.sim.colorcomponent_ambient_diffuse, color)

        self.sim.setObjectAlias(handle, colorName + str(markerId))
        self.created += 1
