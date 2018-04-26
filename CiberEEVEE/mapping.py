import numpy as np
import math
from pygame.locals import *
import pygame
from CiberEEVEE.utils import *

cell_resolution = 16
half_cell_resolution = int(cell_resolution / 2)


class Maze(object):
    """docstring for Maze"""

    def __init__(self):
        self.width = 32
        self.height = 18
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
        self.max_dist_threshold = 2  # maximum sensor distance measured
        self.sensor_cutoff_point = 1.3  # we ignore sensor measures beyond this point

        self.cheese = None
        self.home = None
        self.eevee = [self.width / 2, self.height / 2]
        self.my_cell = self.maze[int(self.width / 2)][int(self.height / 2)]
        self.target = None
        self.sensor_dots, self.wall_dots, self.debug_dots = [], [], []
        self.trust_val = 6
        self.prev_side_odometry_reset_cell = None

    def pick_exploration_target(self, path_planner, dir):
        if -45 <= dir <= 45:
            prev_cell = self.my_cell.neighbor_west
        elif -135 <= dir < -45:
            prev_cell = self.my_cell.neighbor_south
        elif 45 < dir <= 135:
            prev_cell = self.my_cell.neighbor_north
        else:
            prev_cell = self.my_cell.neighbor_east
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

        if len(to_be_explored) > 0:
            return to_be_explored[0][1]
        else:
            for x in range(0, self.width):
                for y in range(0, self.height):
                    self.maze[x][y].explored = False
            self.my_cell.explored = True
            return self.pick_exploration_target(path_planner, dir)

    def reset_side_odometry(self, my_x, my_y, left_sensor, right_sensor, compass):
        if self.prev_side_odometry_reset_cell == self.my_cell:
            return my_x, my_y
        self.prev_side_odometry_reset_cell = self.my_cell

        compass_rad_left = compass * math.pi / 180 + math.pi/2
        compass_rad_right = compass * math.pi / 180 - math.pi/2

        print("pos:", my_x, my_y, self.my_cell, "sensors", left_sensor, right_sensor, compass)

        # find wall we're hitting
        if -15 <= compass <= 15:
            wall_left = self.my_cell.wall_north
            wall_right = self.my_cell.wall_south
            wall_left_coord_y = self.get_gps_coords_from_cell_coords(wall_left.line[0])[1]
            wall_right_coord_y = self.get_gps_coords_from_cell_coords(wall_right.line[0])[1]
            left_sensor_pos_in_eevee_y = wall_left_coord_y + left_sensor
            right_sensor_pos_in_eevee_y = wall_right_coord_y - right_sensor
            my_y = 0.5 * (left_sensor_pos_in_eevee_y - 0.5 * math.sin(compass_rad_left))\
                + 0.5 * (right_sensor_pos_in_eevee_y - 0.5 * math.sin(compass_rad_right))
        elif -105 <= compass < -75:
            wall_left = self.my_cell.wall_west
            wall_right = self.my_cell.wall_east
            wall_left_coord_x = self.get_gps_coords_from_cell_coords(wall_left.line[0])[0]
            wall_right_coord_x = self.get_gps_coords_from_cell_coords(wall_right.line[0])[0]
            left_sensor_pos_in_eevee_x = wall_left_coord_x + left_sensor
            right_sensor_pos_in_eevee_x = wall_right_coord_x - right_sensor
            my_x = 0.5 * (left_sensor_pos_in_eevee_x - 0.5 * math.cos(compass_rad_left)) \
                   + 0.5 * (right_sensor_pos_in_eevee_x - 0.5 * math.cos(compass_rad_right))
        elif 75 < compass <= 105:
            wall_left = self.my_cell.wall_east
            wall_right = self.my_cell.wall_west
            wall_left_coord_x = self.get_gps_coords_from_cell_coords(wall_left.line[0])[0]
            wall_right_coord_x = self.get_gps_coords_from_cell_coords(wall_right.line[0])[0]
            left_sensor_pos_in_eevee_x = wall_left_coord_x - left_sensor
            right_sensor_pos_in_eevee_x = wall_right_coord_x + right_sensor
            my_x = 0.5 * (left_sensor_pos_in_eevee_x - 0.5 * math.cos(compass_rad_left)) \
                   + 0.5 * (right_sensor_pos_in_eevee_x - 0.5 * math.cos(compass_rad_right))
        elif compass >= 165 or compass <= -165:
            wall_left = self.my_cell.wall_south
            wall_right = self.my_cell.wall_north
            wall_left_coord_y = self.get_gps_coords_from_cell_coords(wall_left.line[0])[1]
            wall_right_coord_y = self.get_gps_coords_from_cell_coords(wall_right.line[0])[1]
            left_sensor_pos_in_eevee_y = wall_left_coord_y - left_sensor
            right_sensor_pos_in_eevee_y = wall_right_coord_y + right_sensor
            my_y = 0.5 * (left_sensor_pos_in_eevee_y - 0.5 * math.sin(compass_rad_left)) \
                   + 0.5 * (right_sensor_pos_in_eevee_y - 0.5 * math.sin(compass_rad_right))

        print("new pos:", my_x, my_y)

        return my_x, my_y

    def reset_odometry(self, my_x, my_y, front_sensor, compass, trust_turns):
        compass_rad = compass * math.pi / 180

        print("pos:", my_x, my_y, self.my_cell, "sensors", front_sensor, compass)
        wall = None
        # front_sensor_pos_in_eevee = [my_x + 0.5 * math.cos(compass), my_y + 0.5 * math.sin(compass)]

        # find wall we're hitting
        if -15 <= compass <= 15:
            wall = self.my_cell.wall_east
            wall_coord_x = self.get_gps_coords_from_cell_coords(wall.line[0])[0]
            front_sensor_pos_in_eevee_x = wall_coord_x - front_sensor
            my_x = front_sensor_pos_in_eevee_x - 0.5 * math.cos(compass_rad)
        elif -105 <= compass < -75:
            wall = self.my_cell.wall_north
            wall_coord_y = self.get_gps_coords_from_cell_coords(wall.line[0])[1]
            front_sensor_pos_in_eevee_y = wall_coord_y + front_sensor
            my_y = front_sensor_pos_in_eevee_y - 0.5 * math.sin(compass_rad)
        elif 75 < compass <= 105:
            wall = self.my_cell.wall_south
            wall_coord_y = self.get_gps_coords_from_cell_coords(wall.line[0])[1]
            front_sensor_pos_in_eevee_y = wall_coord_y - front_sensor
            my_y = front_sensor_pos_in_eevee_y - 0.5 * math.sin(compass_rad)
        elif compass >= 165 or compass <= -165:
            wall = self.my_cell.wall_west
            wall_coord_x = self.get_gps_coords_from_cell_coords(wall.line[0])[0]
            front_sensor_pos_in_eevee_x = wall_coord_x + front_sensor
            my_x = front_sensor_pos_in_eevee_x - 0.5 * math.cos(compass_rad)

        if wall is not None:
            wall.confirm_wall()
            wall.get_adjacent_wall().confirm_wall()
            print("new pos:", my_x, my_y)

        return my_x, my_y

    def render(self, close=False):
        if close:
            pygame.quit()
            self.screen = None
            return

        if self.screen is None:
            pygame.init()
            self.screen = pygame.surface.Surface(
                (self.width * cell_resolution, self.height * cell_resolution))  # original GF size
            self.gui_window = pygame.display.set_mode(self.screen_res, HWSURFACE | DOUBLEBUF | RESIZABLE)
            pygame.display.set_caption("Eevee Map")

        self.screen.fill((255, 255, 255))

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

        for debug_dot in self.debug_dots:
            pygame.draw.rect(self.screen, (255, 0, 255),
                             [round(debug_dot[0] * cell_resolution), round(debug_dot[1] * cell_resolution),
                              1, 1])

        # Draw cheese, target, home
        # TODO

        self.gui_window.blit(pygame.transform.scale(self.screen, self.screen_res), (0, 0))
        pygame.display.flip()

    # def trust_based_on_distance(self, val):
    #    val = np.min([self.max_dist_threshold, np.max([val, 0])])
    #    return (1 - val) * 5  # 1 / val

    def get_gps_coords_from_cell_coords(self, cell_cords):
        return [(cell_cords[0] - self.width / 2) * 2, (cell_cords[1] - self.height / 2) * 2]

    def get_cell_coords_from_gps_coords(self, gps_coords):
        # gps_coords are measured in robot diameters, [0,0] is initial robot position
        # cell_coords is measured in cells (1 cell = 2 robot diameters),
        #       [half_width, half_height] is initial robot position
        return [gps_coords[0] / 2 + self.width / 2, gps_coords[1] / 2 + self.height / 2]

    def reset_debug_dots(self):
        self.debug_dots = []

    def add_debug_dot(self, dot):
        self.debug_dots.append(dot)

    def update(self, my_x, my_y, left_sensor, front_sensor, right_sensor, back_sensor,
               compass, ground, colliding):
        self.eevee = self.get_cell_coords_from_gps_coords([my_x, my_y])
        my_cell = self.maze[int(round(self.eevee[0]))][int(round(self.eevee[1]))]
        self.my_cell = my_cell
        self.my_cell.explored = True
        compass = compass * math.pi / 180
        self.sensor_dots, self.wall_dots = [], []

        nearby_cells = [my_cell,
                        self.maze[my_cell.coords[0] + 1][my_cell.coords[1]],
                        self.maze[my_cell.coords[0] - 1][my_cell.coords[1]],
                        self.maze[my_cell.coords[0]][my_cell.coords[1] + 1],
                        self.maze[my_cell.coords[0]][my_cell.coords[1] - 1],
                        self.maze[my_cell.coords[0] + 1][my_cell.coords[1] + 1],
                        self.maze[my_cell.coords[0] - 1][my_cell.coords[1] - 1],
                        self.maze[my_cell.coords[0] - 1][my_cell.coords[1] + 1],
                        self.maze[my_cell.coords[0] + 1][my_cell.coords[1] - 1]]
        min_left_sensor = np.min([self.sensor_cutoff_point, left_sensor])
        min_front_sensor = np.min([self.sensor_cutoff_point, front_sensor])
        min_right_sensor = np.min([self.sensor_cutoff_point, right_sensor])
        min_back_sensor = np.min([self.sensor_cutoff_point, back_sensor])

        # save points for front of sensor and 30º to left and right of each sensor (they're cone sensors)
        front_sensor_pos_in_eevee = [self.eevee[0] + 0.25 * math.cos(compass),
                                     self.eevee[1] + 0.25 * math.sin(compass)]
        front_sensor_wall_pos = [front_sensor_pos_in_eevee[0] + min_front_sensor / 2 * math.cos(compass),
                                 front_sensor_pos_in_eevee[1] + min_front_sensor / 2 * math.sin(compass)]
        front_sensor_l_wall_pos = [
            front_sensor_pos_in_eevee[0] + min_front_sensor / 2 * math.cos(compass + math.pi / 6),
            front_sensor_pos_in_eevee[1] + min_front_sensor / 2 * math.sin(compass + math.pi / 6)]
        front_sensor_r_wall_pos = [
            front_sensor_pos_in_eevee[0] + min_front_sensor / 2 * math.cos(compass - math.pi / 6),
            front_sensor_pos_in_eevee[1] + min_front_sensor / 2 * math.sin(compass - math.pi / 6)]
        self.update_single_sensor(front_sensor, nearby_cells,
                                  [front_sensor_wall_pos, front_sensor_l_wall_pos, front_sensor_r_wall_pos],
                                  front_sensor_pos_in_eevee)

        left_sensor_pos_in_eevee = [self.eevee[0] + 0.25 * math.cos(compass - math.pi / 2),
                                    self.eevee[1] + 0.25 * math.sin(compass - math.pi / 2)]
        left_sensor_wall_pos = [left_sensor_pos_in_eevee[0] + min_left_sensor / 2 * math.cos(compass - math.pi / 2),
                                left_sensor_pos_in_eevee[1] + min_left_sensor / 2 * math.sin(compass - math.pi / 2)]
        left_sensor_l_wall_pos = [
            left_sensor_pos_in_eevee[0] + min_left_sensor / 2 * math.cos(compass - math.pi / 2 + math.pi / 6),
            left_sensor_pos_in_eevee[1] + min_left_sensor / 2 * math.sin(compass - math.pi / 2 + math.pi / 6)]
        left_sensor_r_wall_pos = [
            left_sensor_pos_in_eevee[0] + min_left_sensor / 2 * math.cos(compass - math.pi / 2 - math.pi / 6),
            left_sensor_pos_in_eevee[1] + min_left_sensor / 2 * math.sin(compass - math.pi / 2 - math.pi / 6)]
        self.update_single_sensor(left_sensor, nearby_cells,
                                  [left_sensor_wall_pos, left_sensor_l_wall_pos, left_sensor_r_wall_pos],
                                  left_sensor_pos_in_eevee)

        right_sensor_pos_in_eevee = [self.eevee[0] + 0.25 * math.cos(compass + math.pi / 2),
                                     self.eevee[1] + 0.25 * math.sin(compass + math.pi / 2)]
        right_sensor_wall_pos = [right_sensor_pos_in_eevee[0] + min_right_sensor / 2 * math.cos(compass + math.pi / 2),
                                 right_sensor_pos_in_eevee[1] + min_right_sensor / 2 * math.sin(compass + math.pi / 2)]
        right_sensor_l_wall_pos = [
            right_sensor_pos_in_eevee[0] + min_right_sensor / 2 * math.cos(compass + math.pi / 2 + math.pi / 6),
            right_sensor_pos_in_eevee[1] + min_right_sensor / 2 * math.sin(compass + math.pi / 2 + math.pi / 6)]
        right_sensor_r_wall_pos = [
            right_sensor_pos_in_eevee[0] + min_right_sensor / 2 * math.cos(compass + math.pi / 2 - math.pi / 6),
            right_sensor_pos_in_eevee[1] + min_right_sensor / 2 * math.sin(compass + math.pi / 2 - math.pi / 6)]
        self.update_single_sensor(right_sensor, nearby_cells,
                                  [right_sensor_wall_pos, right_sensor_l_wall_pos, right_sensor_r_wall_pos],
                                  right_sensor_pos_in_eevee)

        back_sensor_pos_in_eevee = [self.eevee[0] + 0.25 * math.cos(compass + math.pi),
                                    self.eevee[1] + 0.25 * math.sin(compass + math.pi)]
        back_sensor_wall_pos = [back_sensor_pos_in_eevee[0] + min_back_sensor / 2 * math.cos(compass + math.pi),
                                back_sensor_pos_in_eevee[1] + min_back_sensor / 2 * math.sin(compass + math.pi)]
        back_sensor_l_wall_pos = [
            back_sensor_pos_in_eevee[0] + min_back_sensor / 2 * math.cos(compass + math.pi + math.pi / 6),
            back_sensor_pos_in_eevee[1] + min_back_sensor / 2 * math.sin(compass + math.pi + math.pi / 6)]
        back_sensor_r_wall_pos = [
            back_sensor_pos_in_eevee[0] + min_back_sensor / 2 * math.cos(compass + math.pi - math.pi / 6),
            back_sensor_pos_in_eevee[1] + min_back_sensor / 2 * math.sin(compass + math.pi - math.pi / 6)]
        self.update_single_sensor(back_sensor, nearby_cells,
                                  [back_sensor_wall_pos, back_sensor_l_wall_pos, back_sensor_r_wall_pos],
                                  back_sensor_pos_in_eevee)

        # confirm no walls where we are moving through
        colliding_possible_walls = []
        for cell in nearby_cells:
            for wall in cell.walls:
                dist = dist_to_line_segment(self.eevee, wall.line[0], wall.line[1])
                if dist < 0.25 * 1.1 and colliding:
                    # append possible walls we're colliding with
                    if not wall.confirmed_no_wall:
                        colliding_possible_walls.append(wall)
                elif dist < 0.25 * 0.5 and not colliding:
                    wall.confirm_no_wall()
                    wall.get_adjacent_wall().confirm_no_wall()
        # but we only confirm them if there's only 1 possible wall
        # otherwise we could be confirming multiple walls when we're only colliding with one
        if len(colliding_possible_walls) == 1:
            colliding_possible_walls[0].confirm_wall()
            colliding_possible_walls[0].get_adjacent_wall().confirm_wall()

    def update_single_sensor(self, sensor_val, nearby_cells, sensor_positions, sensor_pos_in_eevee):
        # if obstacle found
        weighted_walls = []
        if sensor_val < self.sensor_cutoff_point:  # self.max_dist_threshold:
            # check closest ray to any wall
            ray_dists = [self.max_dist_threshold, self.max_dist_threshold, self.max_dist_threshold]
            ray_walls = [None, None, None]
            for index, dot in enumerate(sensor_positions):
                for cell in nearby_cells:
                    for wall in cell.walls:
                        d = dist_to_line_segment(dot, wall.line[0], wall.line[1])
                        if d < ray_dists[index]:
                            ray_dists[index] = d
                            ray_walls[index] = wall
            min_index = np.argmin(ray_dists)

            # if that ray is actually close to a wall
            # closest_dot_to_wall_distance = ray_dists[min_index]
            closest_dot_to_wall = sensor_positions[min_index]
            closest_dot_cell_wall = ray_walls[min_index]
            self.wall_dots.append(closest_dot_to_wall)
            # trust_val = 5 #self.trust_based_on_distance(
            #    dist_to_line_segment(sensor_pos_in_eevee, closest_dot_cell_wall.line[0], closest_dot_cell_wall.line[1]))
            # print("found obs", closest_dot_cell_wall, trust_val)
            closest_dot_cell_wall.weigh(self.trust_val)
            closest_dot_cell_wall.get_adjacent_wall().weigh(self.trust_val)
            weighted_walls = [closest_dot_cell_wall, closest_dot_cell_wall.get_adjacent_wall()]
            # print("wall", closest_dot_cell_wall)

        # decrease all other wall intersections score
        for dot in sensor_positions:
            for cell in nearby_cells:
                for wall in cell.walls:
                    if wall not in weighted_walls and intersects(wall.line[0], wall.line[1],
                                                                 sensor_pos_in_eevee, dot):
                        dist = dist_to_line_segment(sensor_pos_in_eevee, wall.line[0], wall.line[1])
                        # if dist < 1:
                        # trust_val = self.trust_based_on_distance(dist)
                        wall.weigh(-self.trust_val)
                        wall.get_adjacent_wall().weigh(-self.trust_val)
                        # print("cleared wall", wall, trust_val)
                        # print("no wall", wall)
                        weighted_walls.append(wall)
            self.sensor_dots.append(dot)


max_val, min_val = 355, -355


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
