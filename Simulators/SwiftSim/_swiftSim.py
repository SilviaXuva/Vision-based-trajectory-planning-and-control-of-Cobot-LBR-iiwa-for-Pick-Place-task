import numpy as np
from swift import Swift
import time

from Helpers.input import Motion
from Simulators.sim import Sim

class SwiftSim(Sim, Swift):
    def __init__(self) -> None:
        Sim.__init__(self, 'Swift')
        Swift.__init__(self)

    def Step(self, xRef: np.ndarray = None): 
        self.step(Motion.ts)
        time.sleep(Motion.ts)