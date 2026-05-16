import random
from utils import config

def random_3d() -> tuple[int, int, int]:
    position_x = random.uniform(1, config.MAP_LENGTH - 1)
    position_y = random.uniform(1, config.MAP_WIDTH - 1)
    position_z = random.uniform(1, config.MAP_HEIGHT - 1)
    return (position_x, position_y, position_z)

def random_2d() -> tuple[int, int, int]:
    position_x = random.uniform(1, config.MAP_LENGTH - 1)
    position_y = random.uniform(1, config.MAP_WIDTH - 1)
    return (position_x, position_y, 0)

def get_random_start_point_2d(sim_seed, n_points):
    res = get_random_start_point_3d(sim_seed, n_points)
    for i in range(n_points):
        res[i] = tuple([res[i][0], res[i][1], 0])
    return res

def get_random_start_point_3d(sim_seed, n_points):
    start_positions = []
    for i in range (n_points) :
        random.seed(sim_seed + i)
        start_positions.append(random_3d())

    return start_positions

def get_customized_start_point_3d():
    start_positions = []
    for i in range(config.NUMBER_OF_DRONES):
        input_str = input('Please input the coordinates of drone, e.g., 10, 20, 1')
        position_x, position_y, position_z = map(float, input_str.split(','))

        start_positions.append(tuple([position_x, position_y, position_z]))

    return start_positions
