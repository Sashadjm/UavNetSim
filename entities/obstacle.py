from utils.util_function import euclidean_distance_3d
from utils import config

class Obstacle:
    def add_to_grid(self, grid):
        pass

class SphericalObstacle(Obstacle):
    def __init__(self, center, radius, obstacle_id=1):
        self.center = center  # in meter
        self.radius = radius  # in meter
        self.id = obstacle_id

    def add_to_grid(self, grid):
        x0, y0, z0 = self.center
        x0, y0, z0 = int(round(x0)), int(round(y0)), int(round(z0))

        for x in range(max(0, x0 - self.radius), min(config.MAP_LENGTH, x0 + self.radius)):
            for y in range(max(0, y0 - self.radius), min(config.MAP_WIDTH, y0 + self.radius)):
                for z in range(max(0, z0 - self.radius), min(config.MAP_HEIGHT, z0 + self.radius)):
                    if euclidean_distance_3d([x, y, z], self.center) <= self.radius:
                        grid[int(x / config.GRID_RESOLUTION),
                             int(y / config.GRID_RESOLUTION),
                             int(z / config.GRID_RESOLUTION)] = self.id


class CubeObstacle(Obstacle):
    def __init__(self, center, length, width, height, obstacle_id=2):
        self.center = center
        self.length = length
        self.width = width
        self.height = height
        self.id = obstacle_id

    def add_to_grid(self, grid):
        x0, y0, z0 = self.center
        x0, y0, z0 = int(round(x0)), int(round(y0)), int(round(z0))

        for x in range(max(0, x0 - int(self.length / 2)), min(config.MAP_LENGTH, x0 + int(self.length / 2))):
            for y in range(max(0, y0 - int(self.width / 2)), min(config.MAP_WIDTH, y0 + int(self.width / 2))):
                for z in range(max(0, z0 - int(self.height / 2)), min(config.MAP_HEIGHT, z0 + int(self.height / 2))):
                    grid[int(x / config.GRID_RESOLUTION),
                         int(y / config.GRID_RESOLUTION),
                         int(z / config.GRID_RESOLUTION)] = self.id


class RectangularObstacle(Obstacle):
    def __init__(self, ext1, ext2, obstacle_id=3):
        self.lower = [min(ext1[0], ext2[0]), min(ext1[1], ext2[1]), min(ext1[2], ext2[2])]
        self.upper = [max(ext1[0], ext2[0]), max(ext1[1], ext2[1]), max(ext1[2], ext2[2])]
        self.id = obstacle_id

    def add_to_grid(self, grid):
        for x in range(self.lower[0], self.upper[0]+1):
            for y in range(self.lower[1], self.upper[1]+1):
                for z in range(self.lower[2], self.upper[2]+1):
                    grid[x, y, z] = self.id
