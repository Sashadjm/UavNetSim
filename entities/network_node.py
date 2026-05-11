import simpy
import random
import queue
import entities
# from entities.drone import Drone
# from entities.user import User
# from entities.antenna import Antenna
from simulator.log import logger
from routing.dsdv.dsdv import Dsdv
from mac.csma_ca import CsmaCa
from energy.energy_model import EnergyModel
from entities.packet import DataPacket
from allocation.channel_assignment import ChannelAssigner
from utils import config
from utils.util_function import has_intersection
from phy.large_scale_fading import sinr_calculator

class NetworkNode:
    """
    NetworkdNode implementation

    NetworkdNodes in the simulation are served as routers. Each networkd node can be selected as a potential source node,
    destination and relaying node. Each network node needs to install the corresponding routing module, MAC module, mobility
    module and energy module, etc. At the same time, each network node also has its own queue and can only send one packet at
    a time, so subsequent data packets need queuing for queue resources, which is used to reflect the queue delay in the network.

    Attributes:
        simulator: the simulation platform that contains everything
        env: simulation environment created by simpy
        identifier: used to uniquely represent a network node
        coords: the 3-D position of the network node
        inbox: a "Store" in simpy, used to receive the packets from other network nodes (calculate SINR)
        buffer: used to describe the queuing delay of sending packet
        transmitting_queue: when the next hop node receives the packet, it should first temporarily store the packet in
                    "transmitting_queue" instead of immediately yield "packet_coming" process. It can prevent the buffer
                    resource of the previous hop node from being occupied all the time
        waiting_list: for reactive routing protocol, if there is no available next hop, it will put the data packet into
                      "waiting_list". Once the routing information bound for a destination is obtained, network node will get
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
        residual_energy: the residual energy of drone in Joule
        sleep: if the drone is in a "sleep" state, it cannot perform packet sending and receiving operations
        channel_assigner: used to assign sub-channel for transmitting
    """
    def __init__(self,
                 env,
                 node_id,
                 coords,
                 inbox,
                 simulator):
        self.simulator = simulator
        self.env = env
        self.identifier = node_id
        self.coords = coords

        self.rng_node = random.Random(self.identifier + self.simulator.seed)

        self.inbox = inbox

        self.buffer = simpy.Resource(env, capacity=1)
        self.max_queue_size = config.MAX_QUEUE_SIZE
        self.transmitting_queue = queue.Queue()  # queue in the real sense
        self.waiting_list = []

        self.mac_protocol = CsmaCa(self)
        self.mac_process_dict = dict()
        self.mac_process_finish = dict()
        self.mac_process_count = 0
        self.enable_blocking = 1  # enable "stop-and-wait" protocol

        self.routing_protocol = Dsdv(self.simulator, self)
        self.can_relay = True

        self.energy_model = EnergyModel(self)
        self.residual_energy = config.INITIAL_ENERGY
        self.sleep = False

        self.channel_assigner = ChannelAssigner(self.simulator, self)

        self.env.process(self.generate_data_packet())
        self.env.process(self.feed_packet())
        self.env.process(self.receive())

    def generate_data_packet(self, traffic_pattern='Poisson'):

        while True:
            if not self.sleep:
                if traffic_pattern == 'Uniform':
                    # the node generates a data packet every 0.5s with jitter
                    yield self.env.timeout(self.rng_node.randint(500000, 505000))
                elif traffic_pattern == 'Poisson':
                    """
                    The process of generating data packets by nodes follows Poisson distribution, thus the generation 
                    interval of data packets follows exponential distribution
                    """

                    rate = 5  # on average, how many packets are generated in 1s
                    yield self.env.timeout(round(self.rng_node.expovariate(rate) * 1e6))

                config.GL_ID_DATA_PACKET += 1  # data packet id

                # randomly choose a destination
                all_candidate_list = [i for i in range(self.simulator.n_nodes)]
                all_candidate_list.remove(self.identifier)
                dst_id = self.rng_node.choice(all_candidate_list)
                destination = self.simulator.network_nodes[dst_id]  # obtain the destination drone

                # data packet length
                # rien à vraiment changer car on peut garder la même logique pour les différents agents (différence entre data ou message ou quoi)
                if config.VARIABLE_PAYLOAD_LENGTH:
                    fluctuation = self.rng_node.randint(-config.MAXIMUM_PAYLOAD_VARIATION, config.MAXIMUM_PAYLOAD_VARIATION)
                    payload_length = config.AVERAGE_PAYLOAD_LENGTH + fluctuation
                else:
                    payload_length = config.AVERAGE_PAYLOAD_LENGTH  # in bit, 1024 bytes

                data_packet_length = (config.IP_HEADER_LENGTH + config.MAC_HEADER_LENGTH +
                                    config.PHY_HEADER_LENGTH + payload_length)

                # channel assignment
                channel_id = self.channel_assigner.channel_assign()

                pkd = DataPacket(self,
                                dst_node=destination,
                                creation_time=self.env.now,
                                data_packet_id=config.GL_ID_DATA_PACKET,
                                data_packet_length=data_packet_length,
                                simulator=self.simulator,
                                channel_id=channel_id)
                pkd.transmission_mode = 0  # the default transmission mode of data packet is "unicast" (0)

                self.simulator.metrics.datapacket_generated_num += 1

                logger.info('At time: %s (us) ++++ %s: %s generates a data packet (id: %s, dst: %s)',
                            self.env.now, self.get_type_string(), self.identifier, pkd.packet_id, destination.identifier)

                pkd.waiting_start_time = self.env.now

                if self.transmitting_queue.qsize() < self.max_queue_size:
                    self.transmitting_queue.put(pkd)
                else:
                    # the node has no more room for new packets
                    pass
            else:  # cannot generate packets if "my_node" is in sleep state
                break

    def blocking(self):
        """
        The process of waiting for an ACK will block subsequent incoming data packets to simulate the
        "head-of-line blocking problem"
        """

        if self.enable_blocking:
            if not self.mac_protocol.wait_ack_process_finish:
                flag = False  # there is currently no waiting process for ACK
            else:
                # get the latest process status
                final_indicator = list(self.mac_protocol.wait_ack_process_finish.items())[-1]

                if final_indicator[1] == 0:
                    flag = True  # indicates that the node is still waiting
                else:
                    flag = False  # there is currently no waiting process for ACK
        else:
            flag = False

        return flag

    def feed_packet(self):
        """
        It should be noted that this function is designed for those packets which need to compete for wireless channel

        Firstly, all packets received or generated will be put into the "transmitting_queue", every very short
        time, the node will read the packet in the head of the "transmitting_queue". Then the node will check
        if the packet is expired (exceed its maximum lifetime in the network), check the type of packet:
        1) data packet: check if the data packet exceeds its maximum re-transmission attempts. If the above inspection
           passes, routing protocol is executed to determine the next hop node. If next hop is found, then this data
           packet is ready to transmit, otherwise, it will be put into the "waiting_queue".
        2) control packet: no need to determine next hop, so it will directly start waiting for buffer
        """

        while True:
            if not self.sleep:  # if node still has enough energy to relay packets
                yield self.env.timeout(10)  # for speed up the simulation

                if not self.blocking():
                    if not self.transmitting_queue.empty():
                        packet = self.transmitting_queue.get()  # get the packet at the head of the queue

                        if self.env.now < packet.creation_time + packet.deadline:  # this packet has not expired
                            if isinstance(packet, DataPacket):
                                if packet.number_retransmission_attempt[self.identifier] < config.MAX_RETRANSMISSION_ATTEMPT:
                                    # it should be noted that "final_packet" may be the data packet itself or a control
                                    # packet, depending on whether the routing protocol can find an appropriate next hop
                                    has_route, final_packet, enquire = self.routing_protocol.next_hop_selection(packet)

                                    if has_route:
                                        logger.info('At time: %s (us) ---- %s: %s obtain the next hop: %s of data'
                                                    ' packet (id: %s)',
                                                    self.env.now, self.get_type_string(), self.identifier, packet.next_hop_id, packet.packet_id)

                                        # in this case, the "final_packet" is actually the data packet
                                        yield self.env.process(self.packet_coming(final_packet))
                                    else:
                                        self.waiting_list.append(packet)
                                        self.remove_from_queue(packet)

                                        if enquire:
                                            # in this case, the "final_packet" is actually the control packet
                                            yield self.env.process(self.packet_coming(final_packet))

                            else:  # control packet but not ack
                                yield self.env.process(self.packet_coming(packet))
                        else:
                            pass  # means dropping this data packet for expiration
            else:  # this node runs out of energy
                break  # it is important to break the while loop

    def packet_coming(self, pkd):
        """
        When node has a packet ready to transmit, yield it.

        The requirement of "ready" is:
            1) this packet is a control packet, or
            2) the valid next hop of this data packet is obtained

        Parameter:
            pkd: packet that waits to enter the buffer of node
        """

        if not self.sleep:
            arrival_time = self.env.now
            logger.info('At time: %s (us) ---- Packet: %s starts waiting for %s: %s buffer resource',
                        arrival_time, pkd.packet_id, self.get_type_string(), self.identifier)

            with self.buffer.request() as request:
                yield request  # wait to enter to buffer

                logger.info('At time: %s (us) ---- Packet: %s has been added to the buffer of %s: %s, '
                            'waiting time is: %s',
                            self.env.now, pkd.packet_id, self.get_type_string(), self.identifier, self.env.now - arrival_time)

                pkd.number_retransmission_attempt[self.identifier] += 1

                if pkd.number_retransmission_attempt[self.identifier] == 1:
                    pkd.time_transmitted_at_last_hop = self.env.now

                logger.info('At time: %s (us) ---- Re-transmission attempts of pkd: %s at %s: %s is: %s',
                            self.env.now, pkd.packet_id, self.get_type_string(), self.identifier,
                            pkd.number_retransmission_attempt[self.identifier])

                # every time the node initiates a data packet transmission, "mac_process_count" will be increased by 1
                self.mac_process_count += 1

                key=''.join(['mac_send', str(self.identifier), '_', str(pkd.packet_id)])

                mac_process = self.env.process(self.mac_protocol.mac_send(pkd))
                self.mac_process_dict[key] = mac_process
                self.mac_process_finish[key] = 0

                yield mac_process
        else:
            pass

    def remove_from_queue(self, data_pkd):
        """
        After receiving the ack packet, node should remove the data packet that has been acked from its queue

        Parameter:
            data_pkd: the acked data packet
        """
        temp_queue = queue.Queue()

        while not self.transmitting_queue.empty():
            pkd_entry = self.transmitting_queue.get()
            if pkd_entry != data_pkd:
                temp_queue.put(pkd_entry)

        while not temp_queue.empty():
            self.transmitting_queue.put(temp_queue.get())

    def receive(self):
        """
        Core receiving function of node
        1. the node checks its "inbox" to see if there is incoming packet every 5 units (in us) from the time it is
           instantiated to the end of the simulation
        2. update the "inbox" by deleting the inconsequential data packet
        3. then the node will detect if it receives a (or multiple) complete data packet(s)
        4. SINR calculation
        """

        while True:
            if not self.sleep:
                # delete packets that have been processed and do not interfere with
                # the transmission and reception of all current packets
                self.update_inbox()

                flag, all_nodes_send_to_me, time_span, potential_packet = self.trigger()

                if flag:
                    # find the transmitters of all packets currently transmitted on the channel
                    transmitting_node_list = []
                    for node in self.simulator.network_nodes:
                        for item in node.inbox:
                            packet = item[0]
                            insertion_time = item[1]
                            transmitter = item[2]
                            channel_used = item[4]
                            transmitting_time = packet.packet_length / config.BIT_RATE * 1e6
                            interval = [insertion_time, insertion_time + transmitting_time]

                            for interval2 in time_span:
                                if has_intersection(interval, interval2):
                                    transmitting_node_list.append([transmitter, channel_used])

                    # remove duplicates
                    transmitting_node_list = [list(x) for x in {tuple(i) for i in transmitting_node_list}]

                    sinr_list = sinr_calculator(self, all_nodes_send_to_me, transmitting_node_list)

                    # receive the packet of the transmitting node corresponding to the maximum SINR
                    max_sinr = max(sinr_list)
                    if max_sinr >= config.SNR_THRESHOLD:
                        which_one = sinr_list.index(max_sinr)

                        pkd = potential_packet[which_one]

                        if pkd.get_current_ttl() < config.MAX_TTL:
                            sender = all_nodes_send_to_me[which_one][0]

                            logger.info('At time: %s (us) ---- Packet %s from NODE: %s is received by %s: %s, sinr is: %s',
                                        self.env.now, pkd.packet_id, sender, self.get_type_string(), self.identifier, max_sinr)

                            yield self.env.process(self.routing_protocol.packet_reception(pkd, sender))
                        else:
                            logger.info('At time: %s (us) ---- Packet %s is dropped due to exceeding max TTL',
                                        self.env.now, pkd.packet_id)
                    else:  # sinr is lower than threshold
                        pass

                yield self.env.timeout(5)
            else:
                break
    
    #No specific changes because its for both users and drones
    def update_inbox(self):
        """
        Clear the packets that have been processed.
                                           ↓ (current time step)
                              |==========|←- (current incoming packet p1)
                       |==========|←- (packet p2 that has been processed, but also can affect p1, so reserve it)
        |==========|←- (packet p3 that has been processed, no impact on p1, can be deleted)
        --------------------------------------------------------> time
        """

        if config.VARIABLE_PAYLOAD_LENGTH:
            max_transmission_time = ((config.AVERAGE_PAYLOAD_LENGTH + config.MAXIMUM_PAYLOAD_VARIATION)
                                     / config.BIT_RATE) * 1e6  # for a single data packet
        else:
            max_transmission_time = (config.AVERAGE_PAYLOAD_LENGTH / config.BIT_RATE) * 1e6  # for a single data packet

        for item in self.inbox:
            insertion_time = item[1]  # the moment that this packet begins to be sent to the channel
            received = item[3]  # used to indicate if this packet has been processed (1: processed, 0: unprocessed)
            if insertion_time + 2 * max_transmission_time < self.env.now:  # no impact on the current packet
                if received:
                    self.inbox.remove(item)

    def trigger(self):
        """
        Detects whether the node has received a complete data packet

        Returns:
            flag: bool variable, "1" means a complete data packet has been received by this node and vice versa
            all_nodes_send_to_me: a nested list, whose element is a list including sender id and the channel id
            time_span: a nested list, whose element is a list including the time when the packet is transmitted and the
                time when the packet reached
            potential_packet: a list, including all the instances of the received complete data packet
        """

        flag = 0  # used to indicate if I receive a complete packet
        all_nodes_send_to_me = []
        time_span = []
        potential_packet = []

        for item in self.inbox:
            packet = item[0]  # not sure yet whether it has been completely transmitted
            insertion_time = item[1]  # transmission start time
            transmitter = item[2]
            processed = item[3]  # indicate if this packet has been processed
            channel_used = item[4]  # indicate the sub-channel that used to transmit this packet

            transmitting_time = packet.packet_length / config.BIT_RATE * 1e6  # expected transmission time

            if not processed:  # this packet has not been processed yet
                if self.env.now >= insertion_time + transmitting_time:  # it has been transmitted completely
                    flag = 1
                    all_nodes_send_to_me.append([transmitter, channel_used])
                    time_span.append([insertion_time, insertion_time + transmitting_time])
                    potential_packet.append(packet)
                    item[3] = 1
                else:
                    pass
            else:
                pass

        return flag, all_nodes_send_to_me, time_span, potential_packet


    def get_type_string(self):
        """
        Returns a string representation of this node's type.
        """
        if isinstance(self, entities.drone.Drone):
            return "UAV"
        if isinstance(self, entities.user.User):
            return "USER"
        if isinstance(self, entities.antenna.Antenna):
            return "ANTENNA"
        else:
            return "NODE"



    # TODO 
    # - Copier les fonctions relatives au réseau de drone.py dans cette classe
    # - Modifier les fonctions de drone.py pour qu'elles puissent être utilisées dans cette classe
    # - Modifier les implémentations dans /phy, /mac, /routing, etc. pour qu'elles fonctionnent avec
    #   NetworkNode au lieu de Drone.
    # - Modifier Drone pour qu'elle hérite de NetworkNode, et ne contienne que les fonctions spécifiques à Drone
    #   (par exemple, les fonctions de mobilité, etc.)
    # - Ajouter une nouvelle entité, "User", qui hérite de NetworkNode.