import numpy as np
import random
import math
from entities.network_node import NetworkNode


class Antenna(NetworkNode):
    """
    Antenna implementation

    Antennas in the simulation are served as devices meant to send packets. Data can only be sent from then and are being forwarded by the drone entity. 
    """

    def __init__(self,
                 env,
                 node_id,
                 coords,
                 inbox,
                 simulator):
        
        super().__init__(
                 env,
                 node_id,
                 coords,
                 inbox,
                 simulator)
    

        self.rng_antenna = random.Random(self.identifier + self.simulator.seed)
        #self.can_relay = True
