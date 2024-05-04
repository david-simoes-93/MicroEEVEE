import numpy as np
import math
from pygame.locals import *
import Utils

CM_PER_CELL = 3
MAP_SIZE = int(240 / CM_PER_CELL) * 2 + 2  # amount of cells in map
HALF_MAP_SIZE = int(MAP_SIZE / 2)



class Maze(object):
    """
    map is represented as a graph, every 3cm?
    do we always start at some edge of map?
    """

    def __init__(self):
        self.maze = [[Cell(x, y) for y in range(MAP_SIZE)]
                     for x in range(MAP_SIZE)]
        for x in range(0, MAP_SIZE):
            for y in range(0, MAP_SIZE):
                if y > 0:
                    self.maze[x][y].neighbor_north = self.maze[x][y - 1]
                if y < MAP_SIZE - 1:
                    self.maze[x][y].neighbor_south = self.maze[x][y + 1]
                if x > 0:
                    self.maze[x][y].neighbor_west = self.maze[x - 1][y]
                if x < MAP_SIZE - 1:
                    self.maze[x][y].neighbor_east = self.maze[x + 1][y]

        self.my_cell_coords = self.get_gps_coords_from_cell_coords([0, 0])
        self.my_cell = self.maze[HALF_MAP_SIZE][HALF_MAP_SIZE]

        self.target = None
        self.home = self.my_cell

    def pick_exploration_target(self, path_planner, radian_theta):
        degree_theta = Utils.to_degree(radian_theta)
        if -45 <= degree_theta <= 45:
            prev_cell = self.my_cell.neighbor_west
        elif -135 <= degree_theta < -45:
            prev_cell = self.my_cell.neighbor_south
        elif 45 < degree_theta <= 135:
            prev_cell = self.my_cell.neighbor_north
        else:
            prev_cell = self.my_cell.neighbor_east
        is_my_cell_explored = self.my_cell.explored
        self.my_cell.explored = True
        to_be_explored = [[prev_cell, self.my_cell, 0]]
        already_explored = []

        while len(to_be_explored) > 0 and to_be_explored[0][1].explored:
            [prev_cell, curr_cell, curr_dist] = to_be_explored.pop(0)
            already_explored.append(curr_cell)

            neighbors = []
            for neighbor in path_planner.neighbors(curr_cell):
                if neighbor not in already_explored:
                    neighbors.append([curr_cell, neighbor, curr_dist +
                                      path_planner.distance_between(prev_cell, curr_cell, neighbor)])

            # sort to_be_explored by dist
            to_be_explored = sorted(
                to_be_explored + neighbors, key=lambda x: x[2])
        self.my_cell.explored = is_my_cell_explored

        if len(to_be_explored) > 0:
            return to_be_explored[0][1]
        else:
            for x in range(0, MAP_SIZE):
                for y in range(0, MAP_SIZE):
                    self.maze[x][y].explored = False
            self.my_cell.explored = True
            return self.pick_exploration_target(path_planner, degree_theta)

    def get_gps_coords_from_cell_coords(self, cell_cords):
        # gps_coords are measured in cm, [0,0] is initial robot position
        return [(cell_cords[0] - HALF_MAP_SIZE) * CM_PER_CELL, (cell_cords[1] - HALF_MAP_SIZE) * CM_PER_CELL]

    def get_cell_coords_from_gps_coords(self, gps_coords):
        # cell_coords are measured in cells (1 cell = 3cm), [half_width, half_height] is initial robot position
        return [gps_coords[0] / CM_PER_CELL + HALF_MAP_SIZE, gps_coords[1] / CM_PER_CELL + HALF_MAP_SIZE]

    def get_cell_indexes_from_cell_coords(self, cell_coords):
        return [int(round(cell_coords[0])), int(round(cell_coords[1]))]

    def get_cell_indexes_from_gps_coords(self, gps_coords):
        return self.get_cell_indexes_from_cell_coords(self.get_cell_coords_from_gps_coords(gps_coords))

    def get_ground_sensor_cells(self, my_x, my_y, compass):
        GROUND_SENSOR_DISTANCE = 6  # cm

        # 45º left
        far_left_sensor_gps_coords = [my_x + GROUND_SENSOR_DISTANCE * math.cos(compass - math.pi / 4),
                                      my_y + GROUND_SENSOR_DISTANCE * math.sin(compass - math.pi / 4)]
        far_left_cell_x, far_left_cell_y = self.get_cell_indexes_from_gps_coords(
            far_left_sensor_gps_coords)
        print(far_left_sensor_gps_coords)
        far_left_cell = self.maze[far_left_cell_x][far_left_cell_y]
        print(far_left_cell)

        # 25º left
        left_sensor_gps_coords = [my_x + GROUND_SENSOR_DISTANCE * math.cos(compass - math.pi / 6),
                                  my_y + GROUND_SENSOR_DISTANCE * math.sin(compass - math.pi / 6)]
        left_cell_x, left_cell_y = self.get_cell_indexes_from_gps_coords(
            left_sensor_gps_coords)
        left_cell = self.maze[left_cell_x][left_cell_y]

        # 0º front
        front_sensor_gps_coords = [my_x + GROUND_SENSOR_DISTANCE * math.cos(compass),
                                   my_y + GROUND_SENSOR_DISTANCE * math.sin(compass)]
        front_cell_x, front_cell_y = self.get_cell_indexes_from_gps_coords(
            front_sensor_gps_coords)
        front_cell = self.maze[front_cell_x][front_cell_y]

        # 25º right
        right_sensor_gps_coords = [my_x + GROUND_SENSOR_DISTANCE * math.cos(compass + math.pi / 6),
                                   my_y + GROUND_SENSOR_DISTANCE * math.sin(compass + math.pi / 6)]
        right_cell_x, right_cell_y = self.get_cell_indexes_from_gps_coords(
            right_sensor_gps_coords)
        right_cell = self.maze[right_cell_x][right_cell_y]

        # 45º right
        far_right_sensor_gps_coords = [my_x + GROUND_SENSOR_DISTANCE * math.cos(compass + math.pi / 4),
                                       my_y + GROUND_SENSOR_DISTANCE * math.sin(compass + math.pi / 4)]
        far_right_cell_x, far_right_cell_y = self.get_cell_indexes_from_gps_coords(
            far_right_sensor_gps_coords)
        print(far_right_sensor_gps_coords)
        far_right_cell = self.maze[far_right_cell_x][far_right_cell_y]
        print(far_right_cell)

        return [far_left_cell, left_cell, front_cell, right_cell, far_right_cell]

    def update(self, my_x, my_y, compass, ground_sensors):
        my_cell_x, my_cell_y = self.get_cell_indexes_from_gps_coords([
                                                                     my_x, my_y])
        if my_cell_x < 1 or my_cell_x >= MAP_SIZE - 1 or my_cell_y < 1 or my_cell_y >= MAP_SIZE - 1:
            print("OUT OF BOUNDS!")
            return
        self.my_cell = self.maze[my_cell_x][my_cell_y]
        self.my_cell_coords = [my_cell_x, my_cell_y]

        ground_sensor_cells = self.get_ground_sensor_cells(my_x, my_y, compass)

        ground_sensor_cells[0].mark_as_line(
            ground_sensors[0] == 1)    # far left
        ground_sensor_cells[1].mark_as_line(ground_sensors[1] == 1)  # mid left
        ground_sensor_cells[2].mark_as_line(ground_sensors[2] == 1)  # mid
        ground_sensor_cells[3].mark_as_line(
            ground_sensors[3] == 1)  # mid right
        ground_sensor_cells[4].mark_as_line(
            ground_sensors[4] == 1)  # far right


MAX_VAL, MIN_VAL = 155, -155
SENSOR_WEIGHT = 25


class Cell(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.neighbor_north, self.neighbor_south, self.neighbor_east, self.neighbor_west = None, None, None, None
        self.line_weight = 0

    def mark_as_line(self, is_line: bool):
        if is_line:
            self.line_weight = min(self.line_weight+SENSOR_WEIGHT, MAX_VAL)
        else:
            self.line_weight = max(self.line_weight-SENSOR_WEIGHT, MIN_VAL)

    @property
    def is_line(self):
        return self.line_weight > 0

    @property
    def explored(self):
        return self.line_weight > MAX_VAL/2 or self.line_weight < MIN_VAL/2

    def __str__(self):
        return f"{self.x} {self.y}"
