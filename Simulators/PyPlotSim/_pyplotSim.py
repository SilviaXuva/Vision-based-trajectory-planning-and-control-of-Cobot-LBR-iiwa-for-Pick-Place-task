import numpy as np
from roboticstoolbox.backends.PyPlot import PyPlot

from Helpers.input import Motion
from Simulators.sim import Sim

class PyPlotSim(Sim, PyPlot):
    def __init__(self) -> None:
        Sim.__init__(self, 'PyPlot')
        PyPlot.__init__(self)

    def Step(self, xRef: np.ndarray = None): 
        self.step(Motion.ts)
