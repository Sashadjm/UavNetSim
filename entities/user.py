import simpy
import numpy as np
import random
import math
import queue
from entities.network_node import NetworkNode
from simulator.log import logger
from entities.packet import DataPacket
from routing.dsdv.dsdv import Dsdv
from mac.csma_ca import CsmaCa
from mobility.gauss_markov_3d import GaussMarkov3D
from energy.energy_model import EnergyModel
from allocation.channel_assignment import ChannelAssigner
from utils import config
from utils.util_function import has_intersection
from phy.large_scale_fading import sinr_calculator


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
        inbox: a "Store" in simpy, used to receive the packets from other users (calculate SINR)
        buffer: used to describe the queuing delay of sending packet
        transmitting_queue: when the next hop node receives the packet, it should first temporarily store the packet in
                    "transmitting_queue" instead of immediately yield "packet_coming" process. It can prevent the buffer
                    resource of the previous hop node from being occupied all the time
        waiting_list: for reactive routing protocol, if there is no available next hop, it will put the data packet into
                      "waiting_list". Once the routing information bound for a destination is obtained, user will get
                      the data packets related to this destination, and put them into "transmitting_queue"
        mac_protocol: installed mac protocol (CSMA/CA, ALOHA, etc.)
        mac_process_dict: a dictionary, used to store the mac_process that is launched each time
        mac_process_finish: a dictionary, used to indicate the completion of the process
        mac_process_count: used to distinguish between different "mac_send" processes
        enable_blocking: describe whether the process of waiting for an ACK blocks the delivery of subsequent packets
                         1: stop-and-wait protocol; 0: sliding window (need further implemented)
        routing_protocol: routing protocol installed (GPSR, DSDV, etc.)
        mobility_model: mobility model installed (3-D Gauss-markov, 3-D random waypoint, etc.)
        energy_model: energy consumption model installed
        residual_energy: the residual energy of user in Joule
        sleep: if the user is in a "sleep" state, it cannot perform packet sending and receiving operations
        channel_assigner: used to assign sub-channel for transmitting

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
        import simpy
import numpy as np
import random
import math
import queue
from entities.network_node import NetworkNode
from simulator.log import logger
from entities.packet import DataPacket
from routing.dsdv.dsdv import Dsdv
from mac.csma_ca import CsmaCa
from mobility.gauss_markov_3d import GaussMarkov3D
from energy.energy_model import EnergyModel
from allocation.channel_assignment import ChannelAssigner
from utils import config
from utils.util_function import has_intersection
from phy.large_scale_fading import sinr_calculator


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