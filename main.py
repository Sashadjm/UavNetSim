import simpy
from utils import config
from entities.obstacle import RectangularObstacle
from simulator.simulator import Simulator
from visualization.visualizer import SimulationVisualizer

"""
  _   _                   _   _          _     ____    _
 | | | |   __ _  __   __ | \ | |   ___  | |_  / ___|  (_)  _ __ ___
 | | | |  / _` | \ \ / / |  \| |  / _ \ | __| \___ \  | | | '_ ` _ \
 | |_| | | (_| |  \ V /  | |\  | |  __/ | |_   ___) | | | | | | | | |
  \___/   \__,_|   \_/   |_| \_|  \___|  \__| |____/  |_| |_| |_| |_|

"""

def setup_scenario_1(sim: Simulator):
    """
    Create a scenario where :
    - A wall separates the world in 2
    - There is an antenna on one side
    - There is a user on the other side
    - There is a UAV on top of the wall
    """
    sim.add_obstacle(RectangularObstacle([10, 0, 0], [10, 19, 15])) # Grid coordinates

    sim.add_antenna((150, 300, 0))

    sim.add_user((450, 300, 0))

    sim.add_drone((300, 300, 90))


if __name__ == "__main__":
    # Simulation setup
    env = simpy.Environment()

    sim = Simulator(seed=2025, env=env)

    setup_scenario_1(sim)

    # # Add obstacles
    # sim.add_obstacle(RectangularObstacle([10, 0, 0], [10, 19, 19]))

    # # Create entities
    # for i in range(0, config.NUMBER_OF_DRONES):
    #     sim.add_drone()

    #     # # Pour ajouter un module (ex: mobilité)
    #     # drone = sim.add_drone()
    #     # PathFollowing3D(drone, path)

    # for i in range(0, config.NUMBER_OF_USERS):
    #     sim.add_user()

    # for i in range(0, config.NUMBER_OF_ANTENNAS):
    #     sim.add_antenna()

    # Finish simulation setup
    sim.start_sim()

    # Add the visualizer to the simulator
    # Use 20000 microseconds (0.02s) as the visualization frame interval
    visualizer = SimulationVisualizer(sim, output_dir=".", vis_frame_interval=20000)
    visualizer.run_visualization()

    # Run simulation
    env.run(until=config.SIM_TIME)

    # Finalize visualization
    visualizer.finalize()
