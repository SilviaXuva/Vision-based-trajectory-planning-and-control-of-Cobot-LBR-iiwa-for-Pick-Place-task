from Helpers.log import Log
from Helpers.input import Drawing as Dr
from Simulators.CoppeliaSim.objects import CoppeliaObj

import numpy as np

class Drawing(CoppeliaObj):
    def __init__(self) -> None:
        super().__init__('Drawing')
        self.tipHandle = self.sim.getObject('./tip')
        self.Ref = self.Ref(self.sim, self.tipHandle)
        self.Real = self.Real(self.sim, self.tipHandle)
        self.refPos = None
        self.start()
    
    def Show(self):
        self.Real.UpdateLine()
        if self.refPos is not None:
            self.Ref.UpdateLine(self.refPos)
    
    def run(self):
        while True:
            try:
                self.Show()
                self.event.set()
            except:
                pass
            
    class DrawingObj():
        def __init__(self, sim, tipHandle: str, color: list) -> None:
            self.sim = sim
            self.tipHandle = tipHandle
            self.obj = self.sim.addDrawingObject(self.sim.drawing_lines|self.sim.drawing_cyclic, 2, 0, -1, Dr.cyclic, color)
            self.pos = np.array(self.sim.getObjectPosition(self.tipHandle, self.sim.handle_world))

        def UpdateLine(self, pos: np.ndarray):
            pos = np.array(pos)
            line = np.concatenate([self.pos, pos])
            self.sim.addDrawingObjectItem(self.obj, line.tolist())
            self.pos = pos
            
    class Ref(DrawingObj):
        def __init__(self, sim, tipHandle: str, color: list = Dr.ref) -> None:
            super().__init__(sim, tipHandle, color)

    class Real(DrawingObj):
        def __init__(self, sim, tipHandle: str, color: list = Dr.real) -> None:
            super().__init__(sim, tipHandle, color)

        def UpdateLine(self):
            super().UpdateLine(self.sim.getObjectPosition(self.tipHandle, self.sim.handle_world))

def ClearDrawing(sim):
    Log('Clear drawing...')
    for i in range(1, 100):
        try:
            sim.addDrawingObjectItem(i, None)
            sim.removeDrawingObject(i, None)
        except:
            pass