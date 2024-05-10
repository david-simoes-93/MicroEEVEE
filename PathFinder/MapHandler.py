import numpy as np
import math
from pygame.locals import *
import Utils
from Utils import Location

CM_PER_CELL = 12.5
MAP_SIZE = 20 * 2 + 4  # amount of cells in map + 4 for safety
HALF_MAP_SIZE = int(MAP_SIZE / 2)

HALF_LINE_WIDTH_PER_CELL = 1.25 / CM_PER_CELL
HALF_LINE_WIDTH_PER_CELL_THICK = HALF_LINE_WIDTH_PER_CELL * 1.5

EXPLORED_AREA_THRESHOLD = 5

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

        self.my_cell_coords = Maze.get_cell_coords_from_gps_coords(Location(0, 0))
        self.my_theta = 0
        self.my_cell.w_left = MAX_WEIGHT
        self.my_cell.neighbor_left.w_right = MAX_WEIGHT
        self.my_cell.w_right = MAX_WEIGHT
        self.my_cell.neighbor_right.w_left = MAX_WEIGHT

        self.max_explored_indices = Maze.get_cell_indexes_from_cell_coords(self.my_cell_coords)
        self.min_explored_indices = Maze.get_cell_indexes_from_cell_coords(self.my_cell_coords)

        self.planned_path = []

    @property
    def my_cell(self):
        cell_indices = Maze.get_cell_indexes_from_cell_coords(self.my_cell_coords)
        return self.maze[cell_indices.x][cell_indices.y]


    def pick_exploration_target(self, path_planner, radian_theta):
        explored_area = self.max_explored_indices - self.min_explored_indices
        if explored_area.x >= EXPLORED_AREA_THRESHOLD and explored_area.y >= EXPLORED_AREA_THRESHOLD:
            map_center_cell_estimate = self.maze[self.min_explored_indices.x + round(explored_area.x/2)][self.min_explored_indices.y + round(explored_area.y/2)]
            print(f"map center estimate: {map_center_cell_estimate}")
            return self.pick_closest_unexplored_cell(path_planner, radian_theta, map_center_cell_estimate, map_center_cell_estimate)
        
        degree_theta = Utils.to_degree(radian_theta)
        if -45 <= degree_theta <= 45:
            prev_cell = self.my_cell.neighbor_left
        elif -135 <= degree_theta < -45:
            prev_cell = self.my_cell.neighbor_down
        elif 45 < degree_theta <= 135:
            prev_cell = self.my_cell.neighbor_up
        else:
            prev_cell = self.my_cell.neighbor_right
        return self.pick_closest_unexplored_cell(path_planner, radian_theta, prev_cell, self.my_cell)

    def pick_closest_unexplored_cell(self, path_planner, radian_theta, prev_cell, my_cell):
        """ picks closest unexplored cell """
        to_be_explored = [[prev_cell, my_cell, 0]]
        already_explored = []

        while len(to_be_explored) > 0 and (to_be_explored[0][1].explored or to_be_explored[0][1] == my_cell):
            [prev_cell, curr_cell, curr_dist] = to_be_explored.pop(0)
            already_explored.append(curr_cell)

            neighbors = []
            for neighbor in curr_cell.neighbors:
                if neighbor not in already_explored:
                    neighbors.append([curr_cell, neighbor, curr_dist +
                                      path_planner.distance_between(prev_cell, curr_cell, neighbor)])

            # sort to_be_explored by dist
            to_be_explored = sorted(to_be_explored + neighbors, key=lambda x: x[2])

        if len(to_be_explored) > 0:
            # closest unexplored cell
            return to_be_explored[0][1]
        else:
            return prev_cell
    

    @classmethod
    def get_gps_coords_from_cell_coords(cls, cell_cords):
        # gps_coords are measured in cm, [0,0] is initial robot position
        return Location((cell_cords.x - HALF_MAP_SIZE) * CM_PER_CELL, (cell_cords.y - HALF_MAP_SIZE) * CM_PER_CELL)

    @classmethod
    def get_cell_coords_from_gps_coords(cls, gps_coords):
        # cell_coords are measured in cells (1 cell = 12.5cm), [half_width, half_height] is initial robot position
        return Location(gps_coords.x / CM_PER_CELL + HALF_MAP_SIZE, gps_coords.y / CM_PER_CELL + HALF_MAP_SIZE)

    @classmethod
    def get_cell_indexes_from_cell_coords(cls, cell_coords):
        return Location(round(cell_coords.x), round(cell_coords.y))

    @classmethod
    def get_cell_indexes_from_gps_coords(cls, gps_coords):
        return Maze.get_cell_indexes_from_cell_coords(Maze.get_cell_coords_from_gps_coords(gps_coords))

    def set_traversal_weights(self, new_cell_indices):
        prev_cell = self.my_cell
        if new_cell_indices.x < prev_cell.indices.x:
            prev_cell.set_left_weight(TRAVERSED_WEIGHT)
        elif new_cell_indices.x > prev_cell.indices.x:
            prev_cell.set_right_weight(TRAVERSED_WEIGHT)
        elif new_cell_indices.y < prev_cell.indices.y:
            prev_cell.set_up_weight(TRAVERSED_WEIGHT)
        elif new_cell_indices.y > prev_cell.indices.y:
            prev_cell.set_down_weight(TRAVERSED_WEIGHT)

        self.max_explored_indices.x = max(self.max_explored_indices.x, new_cell_indices.x)
        self.max_explored_indices.y = max(self.max_explored_indices.y, new_cell_indices.y)
        self.min_explored_indices.x = min(self.min_explored_indices.x, new_cell_indices.x)
        self.min_explored_indices.y = min(self.min_explored_indices.y, new_cell_indices.y)

    def update(self, eevee, ground_sensors):
        self.my_theta = eevee.theta
        my_cell_coords = Maze.get_cell_coords_from_gps_coords(eevee.gps)
        my_cell_indices = Maze.get_cell_indexes_from_cell_coords(my_cell_coords)
        if my_cell_indices.x < 1 or my_cell_indices.x >= MAP_SIZE - 1 or my_cell_indices.y < 1 or my_cell_indices.y >= MAP_SIZE - 1:
            print("OUT OF BOUNDS!")
            return
        self.set_traversal_weights(my_cell_indices)
        self.my_cell_coords = my_cell_coords

        sensor_cell_coords = [Maze.get_cell_coords_from_gps_coords(sensor_gps) for sensor_gps in eevee.sensor_positions]                                    # [ [1.1, 0.9], ... ]
        sensor_cell_indices = [Maze.get_cell_indexes_from_cell_coords(coords) for coords in sensor_cell_coords]                     # [   [1, 1],   ... ]
        rel_sensor_coords = [coords-cell for coords,cell in zip(sensor_cell_coords, sensor_cell_indices)] # [ [0.1,-0.1], ... ]
        sensor_cells = [self.maze[cell_index.x][cell_index.y] for cell_index in sensor_cell_indices]  

        for i in range(len(ground_sensors)):
            sensor_cells[i].add_sensor_reading(rel_sensor_coords[i], ground_sensors[i])



MAX_WEIGHT= 350
MIN_WEIGHT = -MAX_WEIGHT
THRESHOLD_WEIGHT = 50
SENSOR_WEIGHT = 10
TRAVERSED_WEIGHT = MAX_WEIGHT * 1000


class Cell:
    def __init__(self, x, y):
        self.indices = Location(x,y)
        self.coords = Maze.get_gps_coords_from_cell_coords(self.indices)

        self.w_up = 0 if y != 0 else -TRAVERSED_WEIGHT
        self.w_down = 0 if y != MAP_SIZE - 1 else -TRAVERSED_WEIGHT
        self.w_left = 0 if x != 0 else -TRAVERSED_WEIGHT
        self.w_right = 0 if x != MAP_SIZE - 1 else -TRAVERSED_WEIGHT
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
    
    @property
    def is_intersection(self):
        return (self.down_line or self.up_line) and (self.left_line or self.right_line)
    
    @property
    def explored(self):
        return abs(self.w_up) >= THRESHOLD_WEIGHT and abs(self.w_down) >= THRESHOLD_WEIGHT and abs(self.w_left) >= THRESHOLD_WEIGHT and abs(self.w_right) >= THRESHOLD_WEIGHT

    def set_up_weight(self, w):
        if abs(self.w_up) == TRAVERSED_WEIGHT:
            return
        self.w_up = w
        self.neighbor_up.w_down = w
        self.update_intersection_neighbors()

    def set_down_weight(self, w):
        if abs(self.w_down) == TRAVERSED_WEIGHT:
            return
        self.w_down = w
        self.neighbor_down.w_up = w
        self.update_intersection_neighbors()

    def set_left_weight(self, w):
        if abs(self.w_left) == TRAVERSED_WEIGHT:
            return
        self.w_left = w
        self.neighbor_left.w_right = w
        self.update_intersection_neighbors()

    def set_right_weight(self, w):
        if abs(self.w_right) == TRAVERSED_WEIGHT:
            return
        self.w_right = w
        self.neighbor_right.w_left = w
        self.update_intersection_neighbors()

    def update_intersection_neighbors(self):
        if not self.is_intersection:
            return
        #print(f"intersection at {self}")
        if self.up_line and self.neighbor_up and not self.neighbor_up.is_intersection:
            #print("up")
            self.neighbor_up.set_left_weight(min(self.neighbor_up.w_left, -THRESHOLD_WEIGHT))
            self.neighbor_up.set_right_weight(min(self.neighbor_up.w_right, -THRESHOLD_WEIGHT))
        if self.down_line and self.neighbor_down and not self.neighbor_down.is_intersection:
            #print("down")
            self.neighbor_down.set_left_weight(min(self.neighbor_down.w_left, -THRESHOLD_WEIGHT))
            self.neighbor_down.set_right_weight(min(self.neighbor_down.w_right, -THRESHOLD_WEIGHT))
        if self.left_line and self.neighbor_left and not self.neighbor_left.is_intersection:
            #print("left")
            self.neighbor_left.set_up_weight(min(self.neighbor_left.w_up, -THRESHOLD_WEIGHT))
            self.neighbor_left.set_down_weight(min(self.neighbor_left.w_down, -THRESHOLD_WEIGHT))
        if self.right_line and self.neighbor_right and not self.neighbor_right.is_intersection:
            #print("right")
            self.neighbor_right.set_up_weight(min(self.neighbor_right.w_up, -THRESHOLD_WEIGHT))
            self.neighbor_right.set_down_weight(min(self.neighbor_right.w_down, -THRESHOLD_WEIGHT))


    def add_sensor_reading(self, rel_coords, sensor_val):
        # if sensor is positive, we try to find a wider line and add a weight to it
        if sensor_val:
            if -HALF_LINE_WIDTH_PER_CELL_THICK <= rel_coords.x <= HALF_LINE_WIDTH_PER_CELL_THICK and -0.5 <= rel_coords.y <= -HALF_LINE_WIDTH_PER_CELL_THICK:
                self.set_up_weight(min(self.w_up + SENSOR_WEIGHT, MAX_WEIGHT))
            if -HALF_LINE_WIDTH_PER_CELL_THICK <= rel_coords.x <= HALF_LINE_WIDTH_PER_CELL_THICK and HALF_LINE_WIDTH_PER_CELL_THICK <= rel_coords.y <= 0.5:
                self.set_down_weight(min(self.w_down + SENSOR_WEIGHT, MAX_WEIGHT))
            if -0.5 <= rel_coords.x <= -HALF_LINE_WIDTH_PER_CELL_THICK and -HALF_LINE_WIDTH_PER_CELL_THICK <= rel_coords.y <= HALF_LINE_WIDTH_PER_CELL_THICK:
                self.set_left_weight(min(self.w_left + SENSOR_WEIGHT, MAX_WEIGHT))
            if HALF_LINE_WIDTH_PER_CELL_THICK <= rel_coords.x <= 0.5 and -HALF_LINE_WIDTH_PER_CELL_THICK <= rel_coords.y <= HALF_LINE_WIDTH_PER_CELL_THICK:
                self.set_right_weight(min(self.w_right + SENSOR_WEIGHT, MAX_WEIGHT))
        # if sensor is negative, we try to find a narrower line and remove weight from it
        if not sensor_val:
            if -HALF_LINE_WIDTH_PER_CELL <= rel_coords.x <= HALF_LINE_WIDTH_PER_CELL and -0.5 <= rel_coords.y <= -HALF_LINE_WIDTH_PER_CELL_THICK:
                self.set_up_weight(max(self.w_up - SENSOR_WEIGHT, MIN_WEIGHT))
            if -HALF_LINE_WIDTH_PER_CELL <= rel_coords.x <= HALF_LINE_WIDTH_PER_CELL and HALF_LINE_WIDTH_PER_CELL_THICK <= rel_coords.y <= 0.5:
                self.set_down_weight(max(self.w_down - SENSOR_WEIGHT, MIN_WEIGHT))
            if -0.5 <= rel_coords.x <= -HALF_LINE_WIDTH_PER_CELL_THICK and -HALF_LINE_WIDTH_PER_CELL <= rel_coords.y <= HALF_LINE_WIDTH_PER_CELL:
                self.set_left_weight(max(self.w_left - SENSOR_WEIGHT, MIN_WEIGHT))
            if HALF_LINE_WIDTH_PER_CELL_THICK <= rel_coords.x <= 0.5 and -HALF_LINE_WIDTH_PER_CELL <= rel_coords.y <= HALF_LINE_WIDTH_PER_CELL:
                self.set_right_weight(max(self.w_right - SENSOR_WEIGHT, MIN_WEIGHT))
    
    def __str__(self) -> str:
        return f"{self.indices}"
    
    @property
    def neighbors(self):
        neighbors_arr = []
        if not self.up_free and self.neighbor_up is not None:
            neighbors_arr.append(self.neighbor_up)
        if not self.down_free and self.neighbor_down is not None:
            neighbors_arr.append(self.neighbor_down)
        if not self.left_free and self.neighbor_left is not None:
            neighbors_arr.append(self.neighbor_left)
        if not self.right_free and self.neighbor_right is not None:
            neighbors_arr.append(self.neighbor_right)
        return neighbors_arr
