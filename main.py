import simpy
from utils import config
from entities.obstacle import SphericalObstacle
from simulator.simulator import Simulator
from visualization.visualizer import SimulationVisualizer

"""
  _   _                   _   _          _     ____    _
 | | | |   __ _  __   __ | \ | |   ___  | |_  / ___|  (_)  _ __ ___
 | | | |  / _` | \ \ / / |  \| |  / _ \ | __| \___ \  | | | '_ ` _ \
 | |_| | | (_| |  \ V /  | |\  | |  __/ | |_   ___) | | | | | | | | |
  \___/   \__,_|   \_/   |_| \_|  \___|  \__| |____/  |_| |_| |_| |_|

"""

if __name__ == "__main__":
    # Simulation setup
    env = simpy.Environment()

    sim = Simulator(seed=2025, env=env)

    print(sim.grid)

    # Add obstacles
    sim.add_obstacle(SphericalObstacle([0, 0, 0], 10))

    # Create entities
    for i in range(0, config.NUMBER_OF_DRONES):
        sim.add_drone()

        # # Pour ajouter un module (ex: mobilité)
        # drone = sim.add_drone()
        # PathFollowing3D(drone, path)

    for i in range(0, config.NUMBER_OF_USERS):
        sim.add_user()

    for i in range(0, config.NUMBER_OF_ANTENNAS):
        sim.add_antenna()

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
