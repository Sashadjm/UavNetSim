import numpy as np
import random
import math
from entities.network_node import NetworkNode


class User(NetworkNode):
    """
    User implementation

    users in the simulation are served as end devices. Each user can be selected as a potential source node, destination
    and relaying node. Each user needs to install the corresponding routing module, MAC module, mobility module and
    energy module, etc. At the same time, each user also has its own queue and can only send one packet at a time, so
    subsequent data packets need queuing for queue resources, which is used to reflect the queue delay in the user
    network

    Attributes:
        simulator: the simulation platform that contains everything
        env: simulation environment created by simpy
        identifier: used to uniquely represent a user
        coords: the 3-D position of the user
        start_coords: the initial position of user
        direction: current direction of the user
        pitch: current pitch of the user
        speed: current speed of the user
        velocity: velocity components in three directions
        direction_mean: mean direction
        pitch_mean: mean pitch
        velocity_mean: mean velocity


    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/11
    Updated at: 2025/4/16
    """

    def __init__(self,
                 env,
                 node_id,
                 coords,
                 speed,
                 inbox,
                 simulator):
        
        super().__init__(
                 env,
                 node_id,
                 coords,
                 inbox,
                 simulator)
    

        self.rng_user = random.Random(self.identifier + self.simulator.seed)

        self.direction = self.rng_user.uniform(0, 2 * np.pi)
        self.pitch = self.rng_user.uniform(-0.05, 0.05)
        self.speed = speed  # constant speed throughout the simulation
        self.velocity = [self.speed * math.cos(self.direction) * math.cos(self.pitch),
                         self.speed * math.sin(self.direction) * math.cos(self.pitch),
                         self.speed * math.sin(self.pitch)]

        self.direction_mean = self.direction
        self.pitch_mean = self.pitch
        self.velocity_mean = self.speed

        self.can_relay = False

        # dépend de si branché ou pas mais pas la même énergie que les users donc valeur à changer 