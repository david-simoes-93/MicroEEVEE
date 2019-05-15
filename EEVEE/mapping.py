import numpy as np
import math
from pygame.locals import *
import pygame
from EEVEE.Utils import *

cell_resolution = 16
half_cell_resolution = int(cell_resolution / 2)
cm_per_cell = 45


class Maze(object):
    """docstring for Maze"""

    def __init__(self):
        self.width = 21
        self.height = 21
        self.half_map_width = 10

        self.maze = [[Cell(x, y) for y in range(self.height)] for x in range(self.width)]
        for x in range(0, self.width):
            for y in range(0, self.height):
                if y > 0:
                    self.maze[x][y].neighbor_north = self.maze[x][y - 1]
                if y < self.height - 1:
                    self.maze[x][y].neighbor_south = self.maze[x][y + 1]
                if x > 0:
                    self.maze[x][y].neighbor_west = self.maze[x - 1][y]
                if x < self.width - 1:
                    self.maze[x][y].neighbor_east = self.maze[x + 1][y]
        self.screen = None
        self.screen_res = [self.width * cell_resolution * 2, self.height * cell_resolution * 2]
        self.max_dist_threshold = 1.5  # maximum sensor distance measured
        self.sensor_cutoff_point = 0.75  # we ignore wall measures beyond half a cell

        self.eevee = [self.width / 2, self.height / 2]  # pos in cell
        self.my_cell = self.maze[int(self.width / 2)][int(self.height / 2)]
        self.sensor_dots, self.wall_dots, self.debug_dots = [], [], []
        self.trust_val = 10
        self.prev_side_odometry_reset_cell = None

        self.cheese = None
        self.home = self.my_cell

    def pick_exploration_target(self, path_planner, radian_theta):
        degree_theta = to_degree(radian_theta)
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
            to_be_explored = sorted(to_be_explored + neighbors, key=lambda x: x[2])
        self.my_cell.explored = is_my_cell_explored

        if len(to_be_explored) > 0:
            return to_be_explored[0][1]
        else:
            for x in range(0, self.width):
                for y in range(0, self.height):
                    self.maze[x][y].explored = False
            self.my_cell.explored = True
            return self.pick_exploration_target(path_planner, degree_theta)

    def render(self, close=False):
        if close:
            pygame.quit()
            self.screen = None
            return

        if self.screen is None:
            pygame.init()
            self.screen = pygame.surface.Surface(
                (self.width * cell_resolution, self.height * cell_resolution))  # original GF size
            self.gui_window = pygame.display.set_mode(self.screen_res, pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE)
            pygame.display.set_caption("Eevee Map")

        self.screen.fill((255, 255, 255))

        # Draw cheese, home
        pygame.draw.circle(self.screen, (0, 255, 0),
                           [round(self.home.coords[0] * cell_resolution),
                            round(self.home.coords[1] * cell_resolution)],
                           int(half_cell_resolution / 2))
        if self.cheese is not None:
            pygame.draw.circle(self.screen, (255, 255, 0),
                               [round(self.cheese.coords[0] * cell_resolution),
                                round(self.cheese.coords[1] * cell_resolution)],
                               half_cell_resolution)

        # Draw obstacles
        for row in self.maze:
            for cell in row:
                for wall in cell.walls:
                    wall_color = get_wall_color(wall)
                    pygame.draw.rect(self.screen, wall_color, wall.rect)

        # Draw eevee
        pygame.draw.circle(self.screen, (0, 0, 255),
                           [int(round(self.eevee[0] * cell_resolution)),
                            int(round(self.eevee[1] * cell_resolution))],
                           int(half_cell_resolution / 2))

        for sensor_dot in self.sensor_dots:
            pygame.draw.rect(self.screen, (0, 255, 0),
                             [round(sensor_dot[0] * cell_resolution), round(sensor_dot[1] * cell_resolution),
                              1, 1])
        for wall_dot in self.wall_dots:
            pygame.draw.rect(self.screen, (255, 0, 0),
                             [round(wall_dot[0] * cell_resolution), round(wall_dot[1] * cell_resolution),
                              1, 1])

        # for debug_dot in self.debug_dots:
        #     pygame.draw.rect(self.screen, (255, 0, 255),
        #                      [round(debug_dot[0] * cell_resolution), round(debug_dot[1] * cell_resolution),
        #                       1, 1])

        self.gui_window.blit(pygame.transform.scale(self.screen, self.screen_res), (0, 0))
        pygame.display.flip()

    # def trust_based_on_distance(self, val):
    #    val = np.min([self.max_dist_threshold, np.max([val, 0])])
    #    return (1 - val) * 5  # 1 / val

    def get_gps_coords_from_cell_coords(self, cell_cords):
        return [(cell_cords[0] - self.half_map_width) * 45, (cell_cords[1] - self.half_map_width) * 45]

    def get_cell_coords_from_gps_coords(self, gps_coords):
        # gps_coords are measured in cm, [0,0] is initial robot position
        # cell_coords is measured in cells (1 cell = 45cm),
        #       [half_width, half_height] is initial robot position
        return [gps_coords[0] / 45 + self.half_map_width, gps_coords[1] / 45 + self.half_map_width]

    def reset_debug_dots(self):
        self.debug_dots = []

    def add_debug_dot(self, dot):
        self.debug_dots.append(dot)

    def update(self, my_x, my_y, left_sensor, front_sensor, right_sensor, back_sensor, ir_left_sensor, ir_right_sensor,
               compass, ground):
        #print("mappign", my_x, my_y, left_sensor, front_sensor, right_sensor, back_sensor, ir_left_sensor, ir_right_sensor,
        #       compass, ground)
        left_sensor /= cm_per_cell
        front_sensor /= cm_per_cell
        right_sensor /= cm_per_cell
        back_sensor /= cm_per_cell
        ir_left_sensor /= cm_per_cell
        ir_right_sensor /= cm_per_cell

        self.eevee = self.get_cell_coords_from_gps_coords([my_x, my_y])
        my_cell = self.maze[int(round(self.eevee[0]))][int(round(self.eevee[1]))]
        self.my_cell = my_cell

        # dist_to_cell_center = dist(self.eevee, my_cell.coords)
        # # [1/4 cm da cell] : o robo tem 20cm e a celula 45cm
        # if dist_to_cell_center < 0.25:
        #    self.my_cell.explored = True

        # if we're within cheese's boundaries
        if ground > 2:  # ground average (3 or 4 ground sensors)
            self.cheese = my_cell

        # compass = compass * math.pi / 180  #already in radians
        self.sensor_dots, self.wall_dots = [], []

        nearby_cells = [my_cell,
                        self.maze[my_cell.coords[0] + 1][my_cell.coords[1]],
                        self.maze[my_cell.coords[0] - 1][my_cell.coords[1]],
                        self.maze[my_cell.coords[0]][my_cell.coords[1] + 1],
                        self.maze[my_cell.coords[0]][my_cell.coords[1] - 1]]

        # US sensors
        min_left_sensor = np.min([self.sensor_cutoff_point, left_sensor])
        min_front_sensor = np.min([self.sensor_cutoff_point, front_sensor])
        min_right_sensor = np.min([self.sensor_cutoff_point, right_sensor])
        min_back_sensor = np.min([self.sensor_cutoff_point, back_sensor])

        # IR sensors
        min_ir_left_sensor = np.min([self.sensor_cutoff_point, ir_left_sensor])
        min_ir_right_sensor = np.min([self.sensor_cutoff_point, ir_right_sensor])

        # save points for front of sensor and 30º to left and right of each sensor (they're cone sensors)
        dist_sensor_from_robot_center = 10/45
        dist_ir_from_robot_center = 2/45 # [cm]
        # all sensors > max_dist_threshold do not computam
        if min_front_sensor < self.max_dist_threshold: #abs(compass) < math.pi / 6:
            # 0º
            front_sensor_pos_in_eevee = [self.eevee[0] + dist_sensor_from_robot_center * math.cos(compass),
                                         self.eevee[1] + dist_sensor_from_robot_center * math.sin(compass)]
            front_sensor_wall_pos = [front_sensor_pos_in_eevee[0] + min_front_sensor * math.cos(compass),
                                     front_sensor_pos_in_eevee[1] + min_front_sensor * math.sin(compass)]
            self.update_single_sensor(front_sensor, nearby_cells, [front_sensor_wall_pos], front_sensor_pos_in_eevee)



        """# -45º
        if min_left_sensor < self.max_dist_threshold: #abs(compass) > math.pi/6:
            left45_sensor_pos_in_eevee = [self.eevee[0] + dist_sensor_from_robot_center * math.cos(compass - math.pi / 4),
                                          self.eevee[1] + dist_sensor_from_robot_center * math.sin(compass - math.pi / 4)]
            left45_sensor_wall_pos = [left45_sensor_pos_in_eevee[0] + min_left_sensor * math.cos(compass - math.pi / 4),
                                      left45_sensor_pos_in_eevee[1] + min_left_sensor * math.sin(compass - math.pi / 4)]
            self.update_single_sensor(left_sensor, nearby_cells, [left45_sensor_wall_pos], left45_sensor_pos_in_eevee)"""

        # -90º --> IR
        left_sensor_pos_in_eevee = [self.eevee[0] + dist_ir_from_robot_center * math.cos(compass - math.pi / 2),
                                    self.eevee[1] + dist_ir_from_robot_center * math.sin(compass - math.pi / 2)]
        left_sensor_wall_pos = [left_sensor_pos_in_eevee[0] + min_ir_left_sensor * math.cos(compass - math.pi / 2),
                                left_sensor_pos_in_eevee[1] + min_ir_left_sensor * math.sin(compass - math.pi / 2)]
        self.update_single_sensor(ir_left_sensor, nearby_cells, [left_sensor_wall_pos], left_sensor_pos_in_eevee)

        """# 45º
        if min_right_sensor < self.max_dist_threshold: #abs(compass) > math.pi / 6:
            right45_sensor_pos_in_eevee = [self.eevee[0] + dist_sensor_from_robot_center * math.cos(compass + math.pi / 4),
                                           self.eevee[1] + dist_sensor_from_robot_center * math.sin(compass + math.pi / 4)]
            right45_sensor_wall_pos = [
                right45_sensor_pos_in_eevee[0] + min_right_sensor * math.cos(compass + math.pi / 4),
                right45_sensor_pos_in_eevee[1] + min_right_sensor * math.sin(compass + math.pi / 4)]
            self.update_single_sensor(right_sensor, nearby_cells, [right45_sensor_wall_pos], right45_sensor_pos_in_eevee)"""

        # 90º --> IR
        right_sensor_pos_in_eevee = [self.eevee[0] + dist_ir_from_robot_center * math.cos(compass + math.pi / 2),
                                     self.eevee[1] + dist_ir_from_robot_center * math.sin(compass + math.pi / 2)]
        right_sensor_wall_pos = [
            right_sensor_pos_in_eevee[0] + min_ir_right_sensor * math.cos(compass + math.pi / 2),
            right_sensor_pos_in_eevee[1] + min_ir_right_sensor * math.sin(compass + math.pi / 2)]
        self.update_single_sensor(ir_right_sensor, nearby_cells, [right_sensor_wall_pos], right_sensor_pos_in_eevee)

        """ Removed porque nao serve para nada
        # 180º
        back_sensor_pos_in_eevee = [self.eevee[0] + dist_sensor_from_robot_center * math.cos(compass + math.pi),
                                    self.eevee[1] + dist_sensor_from_robot_center * math.sin(compass + math.pi)]
        back_sensor_wall_pos = [back_sensor_pos_in_eevee[0] + min_back_sensor * math.cos(compass + math.pi),
                                back_sensor_pos_in_eevee[1] + min_back_sensor * math.sin(compass + math.pi)]
        self.update_single_sensor(back_sensor, nearby_cells, [back_sensor_wall_pos], back_sensor_pos_in_eevee)
        """
        # # confirm no walls where we are moving through
        # colliding_possible_walls = []
        # for cell in nearby_cells:
        #     for wall in cell.walls:
        #         dist = dist_to_line_segment(self.eevee, wall.line[0], wall.line[1])
        #         if dist < 0.25 * 1.1 and colliding:
        #             # append possible walls we're colliding with
        #             if not wall.confirmed_no_wall:
        #                 colliding_possible_walls.append(wall)
        #         elif dist < 0.25 * 0.5 and not colliding:
        #             wall.confirm_no_wall()
        #             wall.get_adjacent_wall().confirm_no_wall()
        # # but we only confirm them if there's only 1 possible wall
        # # otherwise we could be confirming multiple walls when we're only colliding with one
        # if len(colliding_possible_walls) == 1:
        #     colliding_possible_walls[0].confirm_wall()
        #     colliding_possible_walls[0].get_adjacent_wall().confirm_wall()

    def update_single_sensor(self, sensor_val, nearby_cells, sensor_positions, sensor_pos_in_eevee):
        #print("Sensor @", sensor_pos_in_eevee, "reading point", sensor_positions, "value", sensor_val)

        # if obstacle found within threshold
        weighted_walls = []
        #só faz isto se estiver no range de uma parede
        if sensor_val < self.sensor_cutoff_point:  # self.max_dist_threshold:
            # check closest ray to any wall
            ray_dists = self.max_dist_threshold  # [, self.max_dist_threshold, self.max_dist_threshold]
            ray_walls = None

            # for index, dot in enumerate(sensor_positions):
            for cell in nearby_cells:
                for wall in cell.walls:
                    d = dist_to_line_segment(sensor_positions[0], wall.line[0], wall.line[1])
                    if d < ray_dists:
                        ray_dists = d
                        ray_walls = wall

            # if that ray is actually close to a wall
            # closest_dot_to_wall_distance = ray_dists[min_index]
            closest_dot_to_wall = sensor_positions[0]
            closest_dot_cell_wall = ray_walls
            self.wall_dots.append(closest_dot_to_wall)

            # trust_val = 5 #self.trust_based_on_distance(
            #    dist_to_line_segment(sensor_pos_in_eevee, closest_dot_cell_wall.line[0], closest_dot_cell_wall.line[1]))
            # print("found obs", closest_dot_cell_wall, trust_val)

            closest_dot_cell_wall.weigh(self.trust_val)
            closest_dot_cell_wall.get_adjacent_wall().weigh(self.trust_val)
            weighted_walls = [closest_dot_cell_wall, closest_dot_cell_wall.get_adjacent_wall()]
            #print("Cell", str(closest_dot_cell_wall.cell), "Wall", str(closest_dot_cell_wall), "Weighted",
            #      closest_dot_cell_wall.weight)
            # print("wall", closest_dot_cell_wall)

        # decrease all other wall intersections score
        for cell in nearby_cells:
            for wall in cell.walls:
                if wall not in weighted_walls and intersects(wall.line[0], wall.line[1],
                                                             sensor_pos_in_eevee, sensor_positions[0]):
                    wall.weigh(-self.trust_val)
                    wall.get_adjacent_wall().weigh(-self.trust_val)
                    weighted_walls.append(wall)
                    #print("Cell", str(wall.cell), "Wall", str(wall), "Cleared",
                    #      wall.weight)

        self.sensor_dots.append(sensor_positions[0])


max_val, min_val = 155, -155


class Cell(object):
    def __init__(self, x, y):
        self.center_in_pixels = [x * cell_resolution, y * cell_resolution]
        self.coords = [x, y]
        self.wall_north = Wall([[x - .4375, y - .4375], [x + .4375, y - .4375]], self, "north")
        self.wall_south = Wall([[x - .4375, y + .4375], [x + .4375, y + .4375]], self, "south")
        self.wall_west = Wall([[x - .4375, y - .4375], [x - .4375, y + .4375]], self, "west")
        self.wall_east = Wall([[x + .4375, y - .4375], [x + .4375, y + .4375]], self, "east")
        self.walls = [self.wall_north, self.wall_south, self.wall_west, self.wall_east]
        self.neighbor_north, self.neighbor_south, self.neighbor_east, self.neighbor_west = None, None, None, None

        self.explored = False

    def __str__(self):
        return str(self.coords)


class Wall(object):
    def __init__(self, line_coords, cell, wall_type):
        self.weight = 0
        self.wall, self.no_wall, self.confirmed_no_wall, self.confirmed_wall = False, False, False, False
        self.line = line_coords
        self.rect = [line_coords[0][0] * cell_resolution, line_coords[0][1] * cell_resolution,
                     cell_resolution if line_coords[1][0] != line_coords[0][0] else 1,
                     cell_resolution if line_coords[1][1] != line_coords[0][1] else 1]
        self.cell = cell
        self.wall_type = wall_type

    def weigh(self, val):
        if not self.confirmed_no_wall and not self.confirmed_wall:
            self.weight = np.min([np.max([self.weight + val, min_val]), max_val])
            self.wall = self.weight > 32
            self.no_wall = self.weight < -32

    def confirm_no_wall(self):
        self.confirmed_wall = False
        self.confirmed_no_wall = True
        self.no_wall = True
        self.wall = False

    def confirm_wall(self):
        self.confirmed_wall = True
        self.confirmed_no_wall = False
        self.no_wall = False
        self.wall = True

    def get_adjacent_wall(self):
        if self.wall_type == "north":
            return self.cell.neighbor_north.wall_south
        if self.wall_type == "south":
            return self.cell.neighbor_south.wall_north
        if self.wall_type == "west":
            return self.cell.neighbor_west.wall_east
        if self.wall_type == "east":
            return self.cell.neighbor_east.wall_west

    def __str__(self):
        return str(self.cell.coords[0]) + "/" + str(self.cell.coords[1]) + " " + self.wall_type
