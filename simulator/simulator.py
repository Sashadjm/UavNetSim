import random
import numpy as np
import matplotlib.pyplot as plt
from phy.channel import Channel
from entities.drone import Drone
from entities.user import User
from entities.obstacle import SphericalObstacle, CubeObstacle
from simulator.metrics import Metrics
from mobility import start_coords
from path_planning.astar import astar
from utils import config
from utils.util_function import grid_map
from allocation.central_controller import CentralController
from visualization.static_drawing import scatter_plot, scatter_plot_with_obstacles


class Simulator:
    """
    Description: simulation environment

    Attributes:
        env: simpy environment
        total_simulation_time: discrete time steps, in nanosecond
        n_drones: number of the drones
        n_users: number of the users
        channel_states: a dictionary, used to describe the channel usage
        channel: wireless channel
        metrics: Metrics class, used to record the network performance
        drones: a list, contains all drone instances
        users: a list, contains all user instances
        network_nodes: a list, contains all network nodes (i.e. drones + users)

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/11
    Updated at: 2025/7/8
    """

    def __init__(self,
                 seed,
                 env,
                 channel_states,
                 n_drones, 
                 n_users,
                 total_simulation_time=config.SIM_TIME):

        self.env = env
        self.seed = seed
        self.total_simulation_time = total_simulation_time  # total simulation time (ns)

        self.n_drones = n_drones  # total number of drones in the simulation
        self.n_users = n_users  # total number of users in the simulation
        self.n_nodes = self.n_drones + self.n_users  # total number of nodes in the simulation, including drones and users
        self.channel_states = channel_states
        self.channel = Channel(self.env)

        self.metrics = Metrics(self)  # use to record the network performance

        # NOTE: if distributed optimization is adopted, remember to comment this to speed up simulation
        # self.central_controller = CentralController(self)

        start_position = start_coords.get_random_start_point_3d(seed)
        # start_position = start_coords.get_customized_start_point_3d()

        current_id = 0

        self.drones = []
        print('Seed is: ', self.seed)
        for i in range(n_drones):
            if config.HETEROGENEOUS:
                speed = random.randint(5, 60)
            else:
                speed = 10

            print('UAV: ', current_id, ' initial location is at: ', start_position[i], ' speed is: ', speed)
            drone = Drone(env=env,
                          node_id=current_id,
                          coords=start_position[i],
                          speed=speed,
                          inbox=self.channel.create_inbox_for_receiver(i),
                          simulator=self)

            self.drones.append(drone)
            current_id += 1

        start_position_user = start_coords.get_random_start_point_3d(seed)
        # start_position = start_coords.get_customized_start_point_3d()

        self.users = []
        for i in range(n_users):
            if config.HETEROGENEOUS:
                speed = random.randint(5, 60)
            else:
                speed = 10

            print('User: ', current_id, ' initial location is at: ', start_position_user[i], ' speed is: ', speed)
            user = User(env=env,
                          node_id=current_id,
                          coords=start_position[i],
                          speed=speed,
                          inbox=self.channel.create_inbox_for_receiver(i),
                          simulator=self)

            self.users.append(user)
            current_id += 1

        self.network_nodes = self.drones + self.users

        # scatter_plot_with_spherical_obstacles(self)
        scatter_plot(self)

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

        self.metrics.print_metrics()
