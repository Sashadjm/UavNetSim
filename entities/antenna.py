import numpy as np
import random
import math
from entities.network_node import NetworkNode
from utils import config
from entities.packet import DataPacket
from simulator.log import logger


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
        self.can_relay = False

    def start_sim(self):
        """
        Start this node simulation
        """

        super().start_sim()

        self.env.process(self.generate_data_packet("Uniform"))


    def generate_data_packet(self, traffic_pattern="Poisson"):

        while True:
            if not self.sleep:
                if traffic_pattern == "Uniform":
                    # the node generates a data packet every 0.04 with jitter
                    yield self.env.timeout(self.rng_node.randint(40000, 45000))
                elif traffic_pattern == "Poisson":
                    """
                    The process of generating data packets by nodes follows Poisson distribution, thus the generation
                    interval of data packets follows exponential distribution
                    """

                    rate = 5  # on average, how many packets are generated in 1s
                    yield self.env.timeout(round(self.rng_node.expovariate(rate) * 1e6))

                config.GL_ID_DATA_PACKET += 1  # data packet id

                # randomly choose a destination
                destination = self.rng_node.choice(self.simulator.users)

                # data packet length
                # rien à vraiment changer car on peut garder la même logique pour les différents agents (différence entre data ou message ou quoi)
                if config.VARIABLE_PAYLOAD_LENGTH:
                    fluctuation = self.rng_node.randint(
                        -config.MAXIMUM_PAYLOAD_VARIATION,
                        config.MAXIMUM_PAYLOAD_VARIATION,
                    )
                    payload_length = config.AVERAGE_PAYLOAD_LENGTH + fluctuation
                else:
                    payload_length = config.AVERAGE_PAYLOAD_LENGTH  # in bit, 1024 bytes

                data_packet_length = (
                    config.IP_HEADER_LENGTH
                    + config.MAC_HEADER_LENGTH
                    + config.PHY_HEADER_LENGTH
                    + payload_length
                )

                # channel assignment
                channel_id = self.channel_assigner.channel_assign()

                pkd = DataPacket(
                    self,
                    dst_node=destination,
                    creation_time=self.env.now,
                    data_packet_id=config.GL_ID_DATA_PACKET,
                    data_packet_length=data_packet_length,
                    simulator=self.simulator,
                    channel_id=channel_id,
                )
                pkd.transmission_mode = (
                    0  # the default transmission mode of data packet is "unicast" (0)
                )

                self.simulator.metrics.datapacket_generated_num += 1

                logger.info(
                    "At time: %s (us) ++++ %s: %s generates a data packet (id: %s, dst: %s)",
                    self.env.now,
                    self.get_type_string(),
                    self.identifier,
                    pkd.packet_id,
                    destination.identifier,
                )

                pkd.waiting_start_time = self.env.now

                if self.transmitting_queue.qsize() < self.max_queue_size:
                    self.transmitting_queue.put(pkd)
                else:
                    # the node has no more room for new packets
                    pass
            else:  # cannot generate packets if "my_node" is in sleep state
                break
