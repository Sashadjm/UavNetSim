import math
from simulator.log import logger
from utils import config
from utils.util_function import euclidean_distance_3d, euclidean_distance_2d
import numpy as np


def sinr_calculator(my_node, main_nodes_list, all_transmitting_nodes_list):
    """
    calculate signal to signal-to-interference-plus-noise ratio

    Parameters:
        my_node: receiver node
        main_nodes_list: list of nodes that wants to transmit packet to receiver
        all_transmitting_nodes_list: list of all nodes currently transmitting packet

    Returns:
        List of sinr of each main node
    """

    simulator = my_node.simulator
    transmit_power = config.TRANSMITTING_POWER
    noise_power = config.NOISE_POWER

    sinr_list = []  # record the sinr of all transmitter
    receiver = my_node

    for pair in main_nodes_list:  # each pair includes the main node id and the channel id
        main_node_id = pair[0]  # node id of main transmitter
        channel_id = pair[1]  # channel id of main transmitter
        transmitter = simulator.network_nodes[main_node_id]

        interference_list = [x[0] for x in all_transmitting_nodes_list]
        channel_list = [x[1] for x in all_transmitting_nodes_list]

        main_link_path_loss = general_path_loss(receiver, transmitter)
        receive_power = transmit_power * main_link_path_loss
        interference_power = 0

        real_interference_nodes = []

        for i in range(0, len(interference_list)):
            if interference_list[i] != main_node_id:  # possible interference
                if my_node.channel_assigner.adjacent_channel_interference_check(channel_id, channel_list[i]):
                    interference = simulator.network_nodes[interference_list[i]]
                    real_interference_nodes.append(interference_list[i])

                    interference_link_path_loss = general_path_loss(receiver, interference)
                    interference_power += transmit_power * interference_link_path_loss
                else:
                    # it means that two sub-channel is non-overlapping
                    pass

        sinr = 10 * math.log10(receive_power / (noise_power + interference_power))

        if real_interference_nodes:
            logger.info('At time: %s (us) ---- Packets collision: Main node is: %s, interference node is: %s, ',
                        simulator.env.now, main_node_id, real_interference_nodes)

            simulator.metrics.collision_num += 1
        else:
            pass

        logger.info('At time: %s (us) ---- The SINR of main link between %s (Tx) %s and %s (Rx) %s is: %s',
                    simulator.env.now, simulator.network_nodes[main_node_id].get_type_string(), main_node_id, receiver.get_type_string(), receiver.identifier, sinr)

        sinr_list.append(sinr)

    return sinr_list

def distance(a: np.ndarray, b: np.ndarray) -> float:
    """
    Compute the Euclidean distance between two points*.

    Args:
        a (np.array): The first point.
        b (np.array): The second point.

    Returns:
        float: The Euclidean distance between the two points.
    """

    return float(np.linalg.norm(a - b))


def _distance_in_one_obstacle(
    a: np.ndarray, b: np.ndarray, obs_min: np.ndarray, obs_max: np.ndarray
) -> float:
    """
    Compute the distance between a line segment and a rectangular obstacle.

    This function uses the `distance` function to calculate the distances.
    The end point can be inside the obstacle, but not the start point.

    Args:
        a (np.array): The start point of the line segment.
        b (np.array): The end point of the line segment.
        obs_min (np.array): The minimum corner of the obstacle.
        obs_max (np.array): The maximum corner of the obstacle.

    Returns:
        float: The distance between the line segment and the obstacle.
    """

    direction = b - a
    tmin, tmax = 0.0, 1.0

    # Check for each axis
    for i in range(3):

        # If the line is not parallel to the axis
        if direction[i] != 0:
            # Calculate intersection points with the planes of the obstacle
            t1 = (obs_min[i] - a[i]) / direction[i]
            t2 = (obs_max[i] - a[i]) / direction[i]

            # Update the interval parameter
            tmin = max(tmin, min(t1, t2))
            tmax = min(tmax, max(t1, t2))

        # If the line is parallel to the axis and outside the obstacle
        elif a[i] < obs_min[i] or a[i] > obs_max[i]:
            return 0.0

    # If the line segment intersects the obstacle
    if tmin <= tmax:
        # Calculate the entry and exit points of the line segment
        entry_point = a + tmin * direction
        exit_point = a + tmax * direction
        if np.all(obs_min <= b) and np.all(b <= obs_max):
            exit_point = b

        # Return the distance between the entry and exit points divide by 5 because obstacles are
        # buildings and buildings are not completely solid
        return distance(entry_point, exit_point) / 5.0

    return 0.0

def distance_in_obstacles(
    a: np.ndarray, b: np.ndarray, obstacles: list[tuple[np.ndarray, np.ndarray]]
) -> float:
    """
    Compute the distance between a line segment and all obstacles.

    Args:
        a (np.array): The start point of the line segment.
        b (np.array): The end point of the line segment.
        obstacles (List[Tuple[np.array, np.array]]):
            The list of obstacles, each represented by a tuple of minimum and maximum corners.

    Returns:
        float: The total distance between the line segment and all obstacles.
    """

    total_distance = 0.0
    for obs in obstacles:
        total_distance += _distance_in_one_obstacle(a, b, obs[0], obs[1])
    return total_distance



def grid_cell_to_obstacle(cell_x: int, cell_y: int, cell_z: int) -> tuple[np.ndarray, np.ndarray]:
    """
    Converts a cell in the grid, defined by its coordinates (ex: cell (1, 2, 0)) to
    a minimum-maximum defined rectangle (ex: [(10, 20, 0), (20, 30, 10)]).
    The "minimum" is the corner of the grid cell which has the three lowest components.
    The "maximum" is the corner of the grid cell which has the three highest components.

    Args:
        cell_x (int): X coordinate of the cell.
        cell_y (int): Y coordinate of the cell.
        cell_z (int): Z coordinate of the cell.

    Returns:
        tuple[np.ndarray, np.ndarray]: A tuple (minimum, maximum) representing the cell as a rectangle.
    """

    # Compute the size of one cell
    cell_length = config.MAP_LENGTH / config.GRID_RESOLUTION
    cell_width = config.MAP_WIDTH / config.GRID_RESOLUTION
    cell_height = config.MAP_HEIGHT / config.GRID_RESOLUTION

    min = np.asarray([cell_x * cell_length, cell_y * cell_width, cell_z * cell_height])
    max = min + np.asarray([cell_length, cell_width, cell_height])

    return (min, max)


def grid_to_obstacles(grid: np.ndarray) -> list[tuple[np.ndarray, np.ndarray]]:
    """
    The world is divided by a grid of size config.GRID_RESOLUTION ** 3.
    Cells of this grid can either be air or rectangular obstacles filling the whole grid.
    We want to convert 
    """
    res = []
    for x in range(0, config.GRID_RESOLUTION):
        for y in range(0, config.GRID_RESOLUTION):
            for z in range(0, config.GRID_RESOLUTION):
                if grid[x, y, z] != 0:
                    res.append(grid_cell_to_obstacle(x, y, z))

    return res


def _path_loss_obs(distance_obs: float) -> float:
    """
    Calculate the path loss in an obstacle for an observed distance.

    Args:
        distance_obs (float): The observed distance in meters.

    Returns:
        float: The calculated path loss in decibels (dB).
    """

    if distance_obs <= 0:
        return 0.0

    return 10.0 * config.PATH_LOSS_OBSTACLE_EXPONENT * np.log10(distance_obs)


def _db_to_linear(db_path_loss: float) -> float:
    """
    Convert a logarithmic path loss to a linear path loss.
    The formula is : linear_path_loss = 10 ^ (db_path_loss / 10)
    """
    return 10 ** (db_path_loss / 10)


def path_loss_obstacles(receiver, transmitter) -> float:
    """
    Compute the path loss due to obstacles.

    Parameters:
        receiver: the node that receives the packet
        transmitter: the node that sends the packet

    Returns:
        path loss due to obstacles
    """
    start = np.asarray(transmitter.coords)
    end = np.asarray(receiver.coords)
    obstacles = grid_to_obstacles(transmitter.simulator.grid)
    distance_obs = distance_in_obstacles(start, end, obstacles) 
    return _db_to_linear(_path_loss_obs(distance_obs))


def general_path_loss(receiver, transmitter):
    """
    General path loss model of line-of-sight (LoS) channels without system loss

    References:
        [1] J. Sabzehali, et al., "Optimizing number, placement, and backhaul connectivity of multi-UAV networks," in
            IEEE Internet of Things Journal, vol. 9, no. 21, pp. 21548-21560, 2022.

    Parameters:
        receiver: the node that receives the packet
        transmitter: the node that sends the packet

    Returns:
        path loss
    """

    c = config.LIGHT_SPEED
    fc = config.CARRIER_FREQUENCY
    alpha = config.PATH_LOSS_EXPONENT

    distance = euclidean_distance_3d(receiver.coords, transmitter.coords)

    if distance != 0:
        path_loss = (c / (4 * math.pi * fc * distance)) ** alpha

        obs_path_loss = path_loss_obstacles(receiver, transmitter)
        path_loss /= obs_path_loss
    else:
        path_loss = 1

    return path_loss

def probabilistic_los_path_loss(receiver, transmitter):
    """
    probabilistic loss mode

    References:
        [1] A. Al-Hourani, S. Kandeepan and S. Lardner, "Optimal LAP Altitude for Maximum Coverage," in IEEE Wireless
            Communications Letters, vol. 3, no. 6, pp. 569-572, 2014.
        [2] J. Sabzehali, et al., "Optimizing number, placement, and backhaul connectivity of multi-UAV networks," in
            IEEE Internet of Things Journal, vol. 9, no. 21, pp. 21548-21560, 2022.

    Parameters:
        receiver: the node that receives the packet
        transmitter: the node that sends the packet

    Returns:
        path loss
    """

    c = config.LIGHT_SPEED
    fc = config.CARRIER_FREQUENCY
    alpha = config.PATH_LOSS_EXPONENT
    eta_los = 0.1
    eta_nlos = 21
    a = 4.88
    b = 0.429

    distance = euclidean_distance_3d(receiver.coords, transmitter.coords)
    horizontal_dist = euclidean_distance_2d(receiver, transmitter)
    vertical_dist = max(receiver.coords[2], transmitter.coords[2])

    elevation_angle = math.atan(horizontal_dist / vertical_dist) * 180 / math.pi

    los_prob = 1 / (1 + a * math.exp(-b * (elevation_angle - a)))
    nlos_prob = 1 - los_prob

    if distance != 0:
        path_loss_los = ((c / (4 * math.pi * fc * distance)) ** alpha) * (10 ** (eta_los / 10))
        path_loss_nlos = ((c / (4 * math.pi * fc * distance)) ** alpha) * (10 ** (eta_nlos / 10))
    else:
        path_loss_los = 1
        path_loss_nlos = 1

    path_loss = los_prob * path_loss_los + nlos_prob * path_loss_nlos
    obs_path_loss = path_loss_obstacles(receiver, transmitter)
    path_loss /= obs_path_loss
    return path_loss


def maximum_communication_range():
    c = config.LIGHT_SPEED
    fc = config.CARRIER_FREQUENCY
    alpha = config.PATH_LOSS_EXPONENT  # path loss exponent
    transmit_power_db = 10 * math.log10(config.TRANSMITTING_POWER)
    noise_power_db = 10 * math.log10(config.NOISE_POWER)
    snr_threshold_db = config.SNR_THRESHOLD

    path_loss_db = transmit_power_db - noise_power_db - snr_threshold_db

    max_comm_range = (c * (10 ** (path_loss_db / (alpha * 10)))) / (4 * math.pi * fc)

    return max_comm_range
