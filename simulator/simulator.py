import random
import numpy as np
import matplotlib.pyplot as plt
import simpy
from entities.network_node import NetworkNode
from phy.channel import Channel
import entities
from entities.drone import Drone
from entities.user import User
from entities.antenna import Antenna
from entities.obstacle import Obstacle
from simulator.metrics import Metrics
from mobility.start_coords import random_2d, random_3d
from path_planning.astar import astar
from utils import config
from utils.util_function import grid_map
from allocation.central_controller import CentralController
from visualization.static_drawing import scatter_plot

def get_drone_speed():
    if config.HETEROGENEOUS:
        return random.randint(5, 60)
    else:
        return 10

def get_user_speed():
    return 0

class Simulator:
    """
    Description: simulation environment

    Attributes:
        env: simpy environment
        total_simulation_time: discrete time steps, in nanosecond
        n_nodes: number of network nodes
        n_drones: number of the drones
        n_users: number of the users
        n_antennas: number of antennas
        channel_states: a dictionary, used to describe the channel usage
        channel: wireless channel
        metrics: Metrics class, used to record the network performance
        drones: a list, contains all drone instances
        users: a list, contains all user instances
        antennas: a list, contains all antenna instances
        network_nodes: a list, contains all network nodes (i.e. drones + users)
        grid: obstacles in the environment

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/11
    Updated at: 2025/7/8
    """

    def __init__(self,
                 seed,
                 env,
                 total_simulation_time=config.SIM_TIME):

        self.env = env
        self.seed = seed
        self.total_simulation_time = total_simulation_time  # total simulation time (ns)

        self.channel_states = {}
        self.channel = Channel(self.env)

        self.metrics = Metrics(self)  # use to record the network performance

        # Whether we started the simulation or not
        self.is_sim_started = False

        # NOTE: if distributed optimization is adopted, remember to comment this to speed up simulation
        # self.central_controller = CentralController(self)

        self.n_drones = 0
        self.n_users = 0
        self.n_antennas = 0

        self.n_nodes = 0

        self.drones: list[Drone] = []
        self.users: list[User] = []
        self.antennas: list[Antenna] = []

        self.network_nodes: list[NetworkNode] = []

        # Create the (for now empty) grid
        self.grid = np.zeros((config.GRID_RESOLUTION, config.GRID_RESOLUTION, config.GRID_RESOLUTION))
        self.obstacles: list[Obstacle] = []
        self.obstacle_type = []

        print('Seed is: ', self.seed)

    def add_obstacle(self, obstacle: Obstacle):
        obstacle.add_to_grid(self.grid)

        if obstacle.id not in self.obstacle_type:
            self.obstacle_type.append(obstacle.id)

        self.obstacles.append(obstacle)

    def add_node(self, node: NetworkNode):
        """
        Add an already built node to the nodes array
        """
        self.channel_states[self.n_nodes] = simpy.Resource(self.env, capacity=1)
        self.network_nodes.append(node)
        self.n_nodes += 1

    def add_drone(self, coords=None, speed=None) -> Drone:
        """
        Construct and add a drone to the simulation

        Keyword arguments:
        coords -- Start position of the drone. Default to a random 3D value
        speed -- Speed of the drone. Default based on config.HETEROGENEOUS
        """
        if coords is None:
            coords = random_3d()
        if speed is None:
            speed = get_drone_speed()

        drone = Drone(
            self.env,
            self.n_nodes,
            coords,
            speed,
            self.channel.create_inbox_for_receiver(self.n_nodes),
            self
        )

        print(f"Creating UAV {self.n_nodes} at position {coords}")

        self.add_node(drone)
        self.drones.append(drone)
        self.n_drones += 1

        return drone

    def add_user(self, coords=None, speed=None) -> User:
        """
        Construct and add a user to the simulation

        Keyword arguments:
        coords -- Start position of the user. Default to a random 2D value
        speed -- Speed of the user. Default 0
        """
        if coords is None:
            coords = random_2d()
        if speed is None:
            speed = get_user_speed()

        user = User(
            self.env,
            self.n_nodes,
            coords,
            speed,
            self.channel.create_inbox_for_receiver(self.n_nodes),
            self
        )

        print(f"Creating USER {self.n_nodes} at position {coords}")

        self.add_node(user)
        self.users.append(user)
        self.n_users += 1

        return user

    def add_antenna(self, coords=None) -> Antenna:
        """
        Construct and add an antenna to the simulation.

        Keyword arguments:
        coords -- Start position of the antenna. Default to a random 2D value
        """
        if coords is None:
            coords = random_2d()

        antenna = Antenna(
            self.env,
            self.n_nodes,
            coords,
            self.channel.create_inbox_for_receiver(self.n_nodes),
            self
        )

        print(f"Creating ANTENNA {self.n_nodes} at position {coords}")

        self.add_node(antenna)
        self.antennas.append(antenna)
        self.n_antennas += 1

        return antenna

    def start_sim(self):
        """
        Start the simulation
        """
        self.is_sim_started = True

        scatter_plot(self)
        #scatter_plot_with_obstacles(self, self.grid, [])

        for node in self.network_nodes:
            node.start_sim()

        self.env.process(self.show_performance())
        self.env.process(self.show_time())

    def show_time(self):
        while True:
            print('At time: ', self.env.now / 1e6, ' s.')

            # the simulation process is displayed every 0.5s
            yield self.env.timeout(0.5*1e6)

    def show_performance(self):
        yield self.env.timeout(self.total_simulation_time - 1)

        scatter_plot(self)

        #self.metrics.print_metrics()
