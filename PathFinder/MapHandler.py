import numpy as np
import math
from pygame.locals import *
import Utils

CM_PER_CELL = 12.5
MAP_SIZE = 20 * 2 + 4  # amount of cells in map + 4 for safety
HALF_MAP_SIZE = int(MAP_SIZE / 2)

class Maze(object):
    # map is a group of cells, the center of which is an intersection
    # each cell can connect up, down, left, or right with a neighbor
    # if a cell connects left, the left neighbor must connect right
    def __init__(self):
        self.maze = [[Cell(x, y) for y in range(MAP_SIZE)]
                     for x in range(MAP_SIZE)]
        for x in range(0, MAP_SIZE):
            for y in range(0, MAP_SIZE):
                if y > 0:
                    self.maze[x][y].neighbor_up = self.maze[x][y - 1]
                if y < MAP_SIZE - 1:
                    self.maze[x][y].neighbor_down = self.maze[x][y + 1]
                if x > 0:
                    self.maze[x][y].neighbor_left = self.maze[x - 1][y]
                if x < MAP_SIZE - 1:
                    self.maze[x][y].neighbor_right = self.maze[x + 1][y]

        self.my_cell_coords = Maze.get_cell_coords_from_gps_coords([0, 0])
        self.my_theta = 0
        self.my_cell.w_left = MAX_WEIGHT
        self.my_cell.w_right = MAX_WEIGHT
        # self.my_cell = self.maze[HALF_MAP_SIZE][HALF_MAP_SIZE]

        self.planned_path = []
        # self.home = self.my_cell

    @property
    def my_cell(self):
        [x, y] = Maze.get_cell_indexes_from_cell_coords(self.my_cell_coords)
        return self.maze[x][y]

    """
    def pick_exploration_target(self, path_planner, radian_theta):
        degree_theta = Utils.to_degree(radian_theta)
        if -45 <= degree_theta <= 45:
            prev_cell = self.my_cell.neighbor_left
        elif -135 <= degree_theta < -45:
            prev_cell = self.my_cell.neighbor_down
        elif 45 < degree_theta <= 135:
            prev_cell = self.my_cell.neighbor_up
        else:
            prev_cell = self.my_cell.neighbor_right
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
    """

    @classmethod
    def get_gps_coords_from_cell_coords(cls, cell_cords):
        # gps_coords are measured in cm, [0,0] is initial robot position
        return [(cell_cords[0] - HALF_MAP_SIZE) * CM_PER_CELL, (cell_cords[1] - HALF_MAP_SIZE) * CM_PER_CELL]

    @classmethod
    def get_cell_coords_from_gps_coords(cls, gps_coords):
        # cell_coords are measured in cells (1 cell = 12.5cm), [half_width, half_height] is initial robot position
        return [gps_coords[0] / CM_PER_CELL + HALF_MAP_SIZE, gps_coords[1] / CM_PER_CELL + HALF_MAP_SIZE]

    @classmethod
    def get_cell_indexes_from_cell_coords(cls, cell_coords):
        return [round(cell_coords[0]), round(cell_coords[1])]

    @classmethod
    def get_cell_indexes_from_gps_coords(cls, gps_coords):
        return Maze.get_cell_indexes_from_cell_coords(Maze.get_cell_coords_from_gps_coords(gps_coords))

    """
    def get_ground_sensor_cells(self, gps_x, gps_y, compass):
        # 40º left
        far_left_sensor_gps_coords = [gps_x + GROUND_SENSOR_DISTANCE * math.cos(compass - FAR_SENSOR_ANGLE),
                                      gps_y + GROUND_SENSOR_DISTANCE * math.sin(compass - FAR_SENSOR_ANGLE)]
        far_left_cell_x, far_left_cell_y = Maze.get_cell_indexes_from_gps_coords(far_left_sensor_gps_coords)
        far_left_cell = self.maze[far_left_cell_x][far_left_cell_y]

        # 10º left
        left_sensor_gps_coords = [gps_x + GROUND_SENSOR_DISTANCE * math.cos(compass - NEAR_SENSOR_ANGLE),
                                  gps_y + GROUND_SENSOR_DISTANCE * math.sin(compass - NEAR_SENSOR_ANGLE)]
        left_cell_x, left_cell_y = Maze.get_cell_indexes_from_gps_coords(left_sensor_gps_coords)
        left_cell = self.maze[left_cell_x][left_cell_y]

        # 0º front
        front_sensor_gps_coords = [gps_x + GROUND_SENSOR_DISTANCE * math.cos(compass),
                                   gps_y + GROUND_SENSOR_DISTANCE * math.sin(compass)]
        front_cell_x, front_cell_y = Maze.get_cell_indexes_from_gps_coords(front_sensor_gps_coords)
        front_cell = self.maze[front_cell_x][front_cell_y]

        # 10º right
        right_sensor_gps_coords = [gps_x + GROUND_SENSOR_DISTANCE * math.cos(compass + NEAR_SENSOR_ANGLE),
                                   gps_y + GROUND_SENSOR_DISTANCE * math.sin(compass + NEAR_SENSOR_ANGLE)]
        right_cell_x, right_cell_y = Maze.get_cell_indexes_from_gps_coords(right_sensor_gps_coords)
        right_cell = self.maze[right_cell_x][right_cell_y]

        # 40º right
        far_right_sensor_gps_coords = [gps_x + GROUND_SENSOR_DISTANCE * math.cos(compass + FAR_SENSOR_ANGLE),
                                       gps_y + GROUND_SENSOR_DISTANCE * math.sin(compass + FAR_SENSOR_ANGLE)]
        far_right_cell_x, far_right_cell_y = Maze.get_cell_indexes_from_gps_coords(far_right_sensor_gps_coords)
        far_right_cell = self.maze[far_right_cell_x][far_right_cell_y]
        
        return [far_left_cell, left_cell, front_cell, right_cell, far_right_cell]

    def update(self, gps_x, gps_y, compass, ground_sensors):
        my_cell_x, my_cell_y = Maze.get_cell_indexes_from_gps_coords([
                                                                     gps_x, gps_y])
        if my_cell_x < 1 or my_cell_x >= MAP_SIZE - 1 or my_cell_y < 1 or my_cell_y >= MAP_SIZE - 1:
            print("OUT OF BOUNDS!")
            return
        self.my_cell = self.maze[my_cell_x][my_cell_y]
        self.my_cell_coords = Maze.get_cell_coords_from_gps_coords([gps_x, gps_y])

        ground_sensor_cells = self.get_ground_sensor_cells(gps_x, gps_y, compass)

        ground_sensor_cells[0].mark_as_line(ground_sensors[0])  # far left
        ground_sensor_cells[1].mark_as_line(ground_sensors[1])  # mid left
        ground_sensor_cells[2].mark_as_line(ground_sensors[2])  # mid
        ground_sensor_cells[3].mark_as_line(ground_sensors[3])  # mid right
        ground_sensor_cells[4].mark_as_line(ground_sensors[4])  # far right
    """

    def update_rigid(self, gps_x, gps_y, compass, ground_sensors):
        # this method assumes robot is always at some 90º angle and that it is always centered on a line
        compass = Utils.get_rigid_compass(compass)
        my_cell_coords = Maze.get_cell_coords_from_gps_coords([gps_x, gps_y])
        my_cell_indices = Maze.get_cell_indexes_from_cell_coords(my_cell_coords)

        offset_from_cell_center = [my_cell_coords[0]-my_cell_indices[0], my_cell_coords[1]-my_cell_indices[1]]
        if abs(offset_from_cell_center[0]) > abs(offset_from_cell_center[1]):
            my_cell_coords[1] = my_cell_indices[1]
        else:
            my_cell_coords[0] = my_cell_indices[0]
        
        [gps_x, gps_y] = Maze.get_gps_coords_from_cell_coords(my_cell_coords)
        self.update(gps_x, gps_y, compass, ground_sensors)
        

    def update(self, gps_x, gps_y, compass, ground_sensors):
        self.my_theta = compass
        my_cell_coords = Maze.get_cell_coords_from_gps_coords([gps_x, gps_y])
        my_cell_indices = Maze.get_cell_indexes_from_cell_coords(my_cell_coords)
        if my_cell_indices[0] < 1 or my_cell_indices[0] >= MAP_SIZE - 1 or my_cell_indices[1] < 1 or my_cell_indices[1] >= MAP_SIZE - 1:
            print("OUT OF BOUNDS!")
            return
        self.my_cell_coords = my_cell_coords

        left_sensor = ground_sensors[0]
        front_sensor = ground_sensors[1] or ground_sensors[2] or ground_sensors[3]
        right_sensor = ground_sensors[4]

        far_left_sensor_gps_coords = Utils.far_left_sensor_gps(gps_x, gps_y, compass)
        front_sensor_gps_coords = Utils.front_sensor_gps(gps_x, gps_y, compass)
        far_right_sensor_gps_coords = Utils.far_right_sensor_gps(gps_x, gps_y, compass)
        
        sensor_cell_coords = [Maze.get_cell_coords_from_gps_coords(far_left_sensor_gps_coords), 
                              Maze.get_cell_coords_from_gps_coords(front_sensor_gps_coords), 
                              Maze.get_cell_coords_from_gps_coords(far_right_sensor_gps_coords)]                                    # [ [1.1, 0.9], ... ]
        sensor_cell_indices = [Maze.get_cell_indexes_from_cell_coords(coords) for coords in sensor_cell_coords]                     # [   [1, 1],   ... ]
        rel_sensor_coords = [[coords[0]-cell[0],coords[1]-cell[1]] for coords,cell in zip(sensor_cell_coords, sensor_cell_indices)] # [ [0.1,-0.1], ... ]
        sensor_cells = [self.maze[x][y] for x,y in sensor_cell_indices]  

        sensor_cells[0].add_sensor_reading(rel_sensor_coords[0], left_sensor)
        sensor_cells[1].add_sensor_reading(rel_sensor_coords[1], front_sensor)
        sensor_cells[2].add_sensor_reading(rel_sensor_coords[2], right_sensor)


MAX_WEIGHT, MIN_WEIGHT = 150, -150
THRESHOLD_WEIGHT = 75
SENSOR_WEIGHT = 25



class Cell:
    def __init__(self, x, y):
        self.indices = [x,y]

        self.w_up = 0 if y != 0 else MIN_WEIGHT
        self.w_down = 0 if y != MAP_SIZE - 1 else MIN_WEIGHT
        self.w_left = 0 if x != 0 else MIN_WEIGHT
        self.w_right = 0 if x != MAP_SIZE - 1 else MIN_WEIGHT
        self.goal = False

        self.neighbor_up = None
        self.neighbor_down = None
        self.neighbor_left = None
        self.neighbor_right = None

    @property
    def up_line(self):
        return self.w_up >= THRESHOLD_WEIGHT
    
    @property
    def down_line(self):
        return self.w_down >= THRESHOLD_WEIGHT
    
    @property
    def left_line(self):
        return self.w_left >= THRESHOLD_WEIGHT
    
    @property
    def right_line(self):
        return self.w_right >= THRESHOLD_WEIGHT
    
    @property
    def up_free(self):
        return self.w_up <= -THRESHOLD_WEIGHT
    
    @property
    def down_free(self):
        return self.w_down <= -THRESHOLD_WEIGHT
    
    @property
    def left_free(self):
        return self.w_left <= -THRESHOLD_WEIGHT
    
    @property
    def right_free(self):
        return self.w_right <= -THRESHOLD_WEIGHT

    def add_sensor_reading(self, rel_coords, sensor_val):
        # if sensor is positive, we try to find a wider line and add a weight to it
        if sensor_val:
            if -0.15 <= rel_coords[0] <= 0.15 and -0.5 <= rel_coords[1] <= -0.15:
                self.w_up = min(self.w_up + SENSOR_WEIGHT, MAX_WEIGHT)
                self.neighbor_up.w_down = self.w_up
            if -0.15 <= rel_coords[0] <= 0.15 and 0.15 <= rel_coords[1] <= 0.5:
                self.w_down = min(self.w_down + SENSOR_WEIGHT, MAX_WEIGHT)
                self.neighbor_down.w_up = self.w_down
            if -0.5 <= rel_coords[0] <= -0.15 and -0.15 <= rel_coords[1] <= 0.15:
                self.w_left = min(self.w_left + SENSOR_WEIGHT, MAX_WEIGHT)
                self.neighbor_left.w_right = self.w_left
            if 0.15 <= rel_coords[0] <= 0.5 and -0.15 <= rel_coords[1] <= 0.15:
                self.w_right = min(self.w_right + SENSOR_WEIGHT, MAX_WEIGHT)
                self.neighbor_right.w_left = self.w_right
        # if sensor is negative, we try to find a narrower line and remove weight from it
        if not sensor_val:
            if -0.10 <= rel_coords[0] <= 0.10 and -0.5 <= rel_coords[1] <= -0.10:
                self.w_up = max(self.w_up - SENSOR_WEIGHT, MIN_WEIGHT)
                self.neighbor_up.w_down = self.w_up
            if -0.10 <= rel_coords[0] <= 0.10 and 0.10 <= rel_coords[1] <= 0.5:
                self.w_down = max(self.w_down - SENSOR_WEIGHT, MIN_WEIGHT)
                self.neighbor_down.w_up = self.w_down
            if -0.5 <= rel_coords[0] <= -0.10 and -0.10 <= rel_coords[1] <= 0.10:
                self.w_left = max(self.w_left - SENSOR_WEIGHT, MIN_WEIGHT)
                self.neighbor_left.w_right = self.w_left
            if 0.10 <= rel_coords[0] <= 0.5 and -0.10 <= rel_coords[1] <= 0.10:
                self.w_right = max(self.w_right - SENSOR_WEIGHT, MIN_WEIGHT)
                self.neighbor_right.w_left = self.w_right
    
    def __str__(self) -> str:
        return f"{self.indices}"
