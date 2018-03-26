# from astar import AStar
import numpy as np
import math
from pygame.locals import *
import pygame
from CiberEEVEE.Utils import *

cell_resolution = 16
half_cell_resolution = int(cell_resolution / 2)


class Maze(object):
    """docstring for Maze"""

    def __init__(self):
        self.width = 32
        self.height = 18
        self.maze = [[Cell(x, y) for y in range(self.height)] for x in range(self.width)]
        for x in range(1, self.width - 1):
            for y in range(1, self.height - 1):
                self.maze[x][y].neighbor_north = self.maze[x][y - 1]
                self.maze[x][y].neighbor_south = self.maze[x][y + 1]
                self.maze[x][y].neighbor_west = self.maze[x - 1][y]
                self.maze[x][y].neighbor_east = self.maze[x + 1][y]
        self.screen = None
        self.screen_res = [self.width * cell_resolution * 2, self.height * cell_resolution * 2]
        self.max_dist_threshold = 2  # maximum sensor distance measured
        self.sensor_cutoff_point = 1  # we ignore sensor measures beyond this point

        self.cheese = None
        self.home = None
        self.eevee = [self.width / 2, self.height / 2]
        self.target = None
        self.sensor_dots, self.wall_dots = [], []

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
                    if not wall.no_wall:
                        wall_color = (0, 0, 0) if wall.wall else (225, 225, 225)
                        pygame.draw.rect(self.screen, wall_color, wall.rect)
                        # if wall.wall:
                        #    print("wall",wall.rect)
                """if not cell.wall_north.no_wall:
                    wall_color = (0, 0, 0) if cell.wall_north.wall else (225, 225, 225)
                    pygame.draw.rect(self.screen, wall_color,
                                     [cell.center_in_pixels[0] - half_cell_resolution,
                                      cell.center_in_pixels[1] - half_cell_resolution,
                                      cell_resolution, 1])
                if not cell.wall_south.no_wall:
                    wall_color = (0, 0, 0) if cell.wall_south.wall else (225, 225, 225)
                    pygame.draw.rect(self.screen, wall_color, [cell.wall_south.line[0][0],
                                                               cell.wall_south.line[0][1], cell_resolution, 1])
                if not cell.wall_west.no_wall:
                    wall_color = (0, 0, 0) if cell.wall_west.wall else (225, 225, 225)
                    pygame.draw.rect(self.screen, wall_color,
                                     [cell.center_in_pixels[0] - half_cell_resolution,
                                      cell.center_in_pixels[1] - half_cell_resolution,
                                      1, cell_resolution])
                if not cell.wall_east.no_wall:
                    wall_color = (0, 0, 0) if cell.wall_east.wall else (225, 225, 225)
                    pygame.draw.rect(self.screen, wall_color,
                                     [cell.center_in_pixels[0] + half_cell_resolution,
                                      cell.center_in_pixels[1] - half_cell_resolution,
                                      1, cell_resolution])"""

        # Draw eevee
        # print("eevee",[int(round(self.eevee[0] * cell_resolution)),
        #                    int(round(self.eevee[1] * cell_resolution))])
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

        # Draw cheese, target, home
        # TODO

        self.gui_window.blit(pygame.transform.scale(self.screen, self.screen_res), (0, 0))
        pygame.display.flip()

    def trust_based_on_distance(self, val):
        val = np.min([self.max_dist_threshold, np.max([val, 0])])
        return (1 - val) * 5  # 1 / val

    def update(self, my_x, my_y, left_sensor, front_sensor, right_sensor, back_sensor,
               compass, ground):
        # my_x and my_y are measured in robot diameters, [0,0] is initial robot position
        # eevee is measured in cells (1 cell = 2 robot diameters), [half_width, half_height] is initial robot position
        self.eevee = [my_x / 2 + self.width / 2, my_y / 2 + self.height / 2]
        my_cell = self.maze[int(round(self.eevee[0]))][int(round(self.eevee[1]))]
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
        for cell in nearby_cells:
            for wall in cell.walls:
                dist = dist_to_line_segment(self.eevee, wall.line[0], wall.line[1])
                if dist < 0.25 * 0.8:
                    wall.confirm_no_wall()
                    wall.get_adjacent_wall().confirm_no_wall()

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
            # if closest_dot_to_wall_distance < .1:  # TODO account of noise increase as dist increases - sensor_val / 10
            closest_dot_to_wall = sensor_positions[min_index]
            closest_dot_cell_wall = ray_walls[min_index]
            self.wall_dots.append(closest_dot_to_wall)
            trust_val = self.trust_based_on_distance(
                dist_to_line_segment(sensor_pos_in_eevee, closest_dot_cell_wall.line[0], closest_dot_cell_wall.line[1]))
            # print("found obs", closest_dot_cell_wall, trust_val)
            closest_dot_cell_wall.weigh(trust_val)
            closest_dot_cell_wall.get_adjacent_wall().weigh(trust_val)
            weighted_walls = [closest_dot_cell_wall]
            # print("wall", closest_dot_cell_wall)

        # decrease all other wall intersections score
        for dot in sensor_positions:
            for cell in nearby_cells:
                for wall in cell.walls:
                    if wall not in weighted_walls and intersects(wall.line[0], wall.line[1],
                                                                 sensor_pos_in_eevee, dot):
                        dist = dist_to_line_segment(sensor_pos_in_eevee, wall.line[0], wall.line[1])
                        # if dist < 1:
                        trust_val = self.trust_based_on_distance(dist - 0.25)
                        wall.weigh(-trust_val)
                        wall.get_adjacent_wall().weigh(-trust_val)
                        # print("cleared wall", wall, trust_val)
                        # print("no wall", wall)
                        weighted_walls.append(wall)
            self.sensor_dots.append(dot)


"""    def update2(self, my_x, my_y, left_sensor, front_sensor, right_sensor, back_sensor,
                compass, ground):
        # my_x and my_y are measured in robot diameters, [0,0] is initial robot position
        # eevee is measured in cells (1 cell = 2 robot diameters), [half_width, half_height] is initial robot position
        self.eevee = [my_x / 2 + self.width / 2, my_y / 2 + self.height / 2]
        my_cell = self.maze[int(round(self.eevee[0]))][int(round(self.eevee[1]))]
        compass = compass * math.pi / 180

        # save points for front of sensor and 30º to left and right of each sensor (they're cone sensors)
        front_sensor_wall_pos = [self.eevee[0] + (front_sensor / 2 + 0.25) * math.cos(compass),
                                 self.eevee[1] + (front_sensor / 2 + 0.25) * math.sin(compass)]
        front_sensor_l_wall_pos = [self.eevee[0] + (front_sensor / 2 + 0.25) * math.cos(compass + math.pi / 6),
                                   self.eevee[1] + (front_sensor / 2 + 0.25) * math.sin(compass + math.pi / 6)]
        front_sensor_r_wall_pos = [self.eevee[0] + (front_sensor / 2 + 0.25) * math.cos(compass - math.pi / 6),
                                   self.eevee[1] + (front_sensor / 2 + 0.25) * math.sin(compass - math.pi / 6)]
        left_sensor_wall_pos = [self.eevee[0] + (left_sensor / 2 + 0.25) * math.cos(compass + math.pi / 2),
                                self.eevee[1] + (left_sensor / 2 + 0.25) * math.sin(compass + math.pi / 2)]
        left_sensor_l_wall_pos = [
            self.eevee[0] + (left_sensor / 2 + 0.25) * math.cos(compass + math.pi / 2 + math.pi / 6),
            self.eevee[1] + (left_sensor / 2 + 0.25) * math.sin(compass + math.pi / 2 + math.pi / 6)]
        left_sensor_r_wall_pos = [
            self.eevee[0] + (left_sensor / 2 + 0.25) * math.cos(compass + math.pi / 2 - math.pi / 6),
            self.eevee[1] + (left_sensor / 2 + 0.25) * math.sin(compass + math.pi / 2 - math.pi / 6)]
        right_sensor_wall_pos = [self.eevee[0] + (right_sensor / 2 + 0.25) * math.cos(compass - math.pi / 2),
                                 self.eevee[1] + (right_sensor / 2 + 0.25) * math.sin(compass - math.pi / 2)]
        right_sensor_l_wall_pos = [
            self.eevee[0] + (right_sensor / 2 + 0.25) * math.cos(compass - math.pi / 2 + math.pi / 6),
            self.eevee[1] + (right_sensor / 2 + 0.25) * math.sin(compass - math.pi / 2 + math.pi / 6)]
        right_sensor_r_wall_pos = [
            self.eevee[0] + (right_sensor / 2 + 0.25) * math.cos(compass - math.pi / 2 - math.pi / 6),
            self.eevee[1] + (right_sensor / 2 + 0.25) * math.sin(compass - math.pi / 2 - math.pi / 6)]
        back_sensor_wall_pos = [self.eevee[0] + (back_sensor / 2 + 0.25) * math.cos(compass + math.pi),
                                self.eevee[1] + (back_sensor / 2 + 0.25) * math.sin(compass + math.pi)]
        self.sensor_dots = [front_sensor_wall_pos, front_sensor_l_wall_pos, front_sensor_r_wall_pos,
                            left_sensor_wall_pos, left_sensor_l_wall_pos, left_sensor_r_wall_pos,
                            right_sensor_wall_pos, right_sensor_l_wall_pos, right_sensor_r_wall_pos,
                            back_sensor_wall_pos]
        # self.sensor_dots = [back_sensor_wall_pos]

        nearby_cells = [my_cell,
                        self.maze[my_cell.coords[0] + 1][my_cell.coords[1]],
                        self.maze[my_cell.coords[0] - 1][my_cell.coords[1]],
                        self.maze[my_cell.coords[0]][my_cell.coords[1] + 1],
                        self.maze[my_cell.coords[0]][my_cell.coords[1] - 1]]
        for dot in self.sensor_dots:
            for cell in nearby_cells:
                # first check distance of sensor_dot to wall_line
                # close, then wall_line is "obstacle"
                # if not, then check intersections between the line "eevee and sensor_dot" and the wall_line
                # and if intersection exists, mark that wall_line as "no obstacle"

                # NORTH
                wall_line_north = [[cell.coords[0] - .5, cell.coords[1] - .5],
                                   [cell.coords[0] + .5, cell.coords[1] - .5]]
                d = dist_to_line_segment(dot, wall_line_north[0], wall_line_north[1])
                trust_based_on_distance = 1.5 - np.min([1.5, dist2(self.eevee, dot)])
                if d < 0.1:
                    cell.weigh_north(trust_based_on_distance)
                    self.maze[cell.coords[0]][cell.coords[1] - 1].weigh_south(trust_based_on_distance)
                elif intersects(wall_line_north[0], wall_line_north[1], self.eevee, dot):
                    cell.weigh_north(-trust_based_on_distance)
                    self.maze[cell.coords[0]][cell.coords[1] - 1].weigh_south(-trust_based_on_distance)

                # SOUTH
                wall_line_south = [[cell.coords[0] - .5, cell.coords[1] + .5],
                                   [cell.coords[0] + .5, cell.coords[1] + .5]]
                d = dist_to_line_segment(dot, wall_line_south[0], wall_line_south[1])
                trust_based_on_distance = 1.5 - np.min([1.5, dist2(self.eevee, dot) - 0.25])  # TODO
                if d < 0.1:
                    cell.weigh_south(trust_based_on_distance)
                    self.maze[cell.coords[0]][cell.coords[1] + 1].weigh_north(trust_based_on_distance)
                elif intersects(wall_line_north[0], wall_line_north[1], self.eevee, dot):
                    cell.weigh_south(-trust_based_on_distance)
                    self.maze[cell.coords[0]][cell.coords[1] + 1].weigh_north(-trust_based_on_distance)

                # WEST
                wall_line_west = [[cell.coords[0] - .5, cell.coords[1] - .5],
                                  [cell.coords[0] - .5, cell.coords[1] + .5]]
                d = dist_to_line_segment(dot, wall_line_west[0], wall_line_west[1])
                trust_based_on_distance = 1.5 - np.min([1.5, dist2(self.eevee, dot)])
                if d < 0.1:
                    cell.weigh_west(trust_based_on_distance)
                    self.maze[cell.coords[0] - 1][cell.coords[1]].weigh_east(trust_based_on_distance)
                elif intersects(wall_line_north[0], wall_line_north[1], self.eevee, dot):
                    cell.weigh_west(-trust_based_on_distance)
                    self.maze[cell.coords[0] - 1][cell.coords[1]].weigh_east(-trust_based_on_distance)

                # EAST
                wall_line_east = [[cell.coords[0] + .5, cell.coords[1] - .5],
                                  [cell.coords[0] + .5, cell.coords[1] + .5]]
                d = dist_to_line_segment(dot, wall_line_east[0], wall_line_east[1])
                trust_based_on_distance = 1.5 - np.min([1.5, dist2(self.eevee, dot)])
                if d < 0.1:
                    cell.weigh_east(trust_based_on_distance)
                    self.maze[cell.coords[0] + 1][cell.coords[1]].weigh_west(trust_based_on_distance)
                elif intersects(wall_line_north[0], wall_line_north[1], self.eevee, dot):
                    cell.weigh_east(-trust_based_on_distance)
                    self.maze[cell.coords[0] + 1][cell.coords[1]].weigh_west(-trust_based_on_distance)
"""

"""    # sets a "c" line between "x,y" and "x2,y2", with probability values according to the distance of this line
    def addLine(self, x, y, x2, y2, c, distMin, distMax):
        length_line_x = x2 - x
        length_line_y = y2 - y
        dx1 = 0
        dy1 = 0
        dx2 = 0
        dy2 = 0

        if (length_line_x < 0):
            dx1 = -1
            dx2 = -1
        elif (length_line_x > 0):
            dx1 = 1
            dx2 = 1

        if (length_line_y < 0):
            dy1 = -1
        elif (length_line_y > 0):
            dy1 = 1

        longest = np.abs(length_line_x)
        shortest = np.abs(length_line_y)
        if longest <= shortest:
            longest = np.abs(length_line_y)
            shortest = np.abs(length_line_x)
            if (length_line_y < 0):
                dy2 = -1
            elif (length_line_y > 0):
                dy2 = 1
            dx2 = 0

        numerator = longest >> 1;

        for i in range(longest + 1):
            print("setReading", x, y, c, self.getRangedVal(distMin, distMax, i + 1, longest + 1))

            numerator += shortest
            if numerator >= longest:
                numerator -= longest
                x += dx1
                y += dy1
            else:
                x += dx2
                y += dy2

    # return the probabily value for a given distance from the robot
    def getRangedVal(self, minDist, maxDist, curr, max):
        ratio = curr * 1.0 / max;
        val = (int)((maxDist - minDist) * ratio + minDist);
        return (int)(10 - val);"""

max_val, min_val = 255, -255


class Cell(object):
    def __init__(self, x, y):
        self.center_in_pixels = [x * cell_resolution, y * cell_resolution]
        self.coords = [x, y]
        self.wall_north = Wall([[x - .4375, y - .4375], [x + .4375, y - .4375]], self, "north")
        self.wall_south = Wall([[x - .4375, y + .4375], [x + .4375, y + .4375]], self, "south")
        self.wall_west = Wall([[x - .4375, y - .4375], [x - .4375, y + .4375]], self, "west")
        self.wall_east = Wall([[x + .4375, y - .4375], [x + .4375, y + .4375]], self, "east")
        self.walls = [self.wall_north, self.wall_south, self.wall_west, self.wall_east]
        # self.wall_north, self.wall_south, self.wall_west, self.wall_east = False, False, False, False
        # self.no_wall_north, self.no_wall_north, self.no_wall_north, self.no_wall_north = False, False, False, False
        # self.weight_north, self.weight_south, self.weight_west, self.weight_east = 0, 0, 0, 0
        # wall_line_north = [[cell.coords[0] - .5, cell.coords[1] - .5],
        #                   [cell.coords[0] + .5, cell.coords[1] - .5]]

    """def weigh_north(self, val):
        self.weight_north = np.min([np.max([self.weight_north + val, min_val]), max_val])
        self.wall_north = self.weight_north > 128

    def weigh_south(self, val):
        self.weight_south = np.min([np.max([self.weight_north + val, min_val]), max_val])
        self.wall_north = self.weight_south > 128

    def weigh_west(self, val):
        self.weight_west = np.min([np.max([self.weight_north + val, min_val]), max_val])
        self.wall_west = self.weight_west > 128

    def weigh_east(self, val):
        self.weight_east = np.min([np.max([self.weight_north + val, min_val]), max_val])
        self.wall_east = self.weight_east > 128"""


class Wall(object):
    def __init__(self, line_coords, cell, wall_type):
        self.weight = 0
        self.wall, self.no_wall, self.confirmed_no_wall = False, False, False
        self.line = line_coords
        self.rect = [line_coords[0][0] * cell_resolution, line_coords[0][1] * cell_resolution,
                     cell_resolution if line_coords[1][0] != line_coords[0][0] else 1,
                     cell_resolution if line_coords[1][1] != line_coords[0][1] else 1]
        self.cell = cell
        self.wall_type = wall_type

    def weigh(self, val):
        if not self.confirmed_no_wall:
            self.weight = np.min([np.max([self.weight + val, min_val]), max_val])
            self.wall = self.weight > 32
            self.no_wall = self.weight < -32

    def confirm_no_wall(self):
        self.confirmed_no_wall = True
        self.no_wall = True
        self.wall = False

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


"""class MazeSolver(AStar):

    ""sample use of the astar algorithm. In this exemple we work on a maze made of ascii characters,
    and a 'node' is just a (x,y) tuple that represents a reachable position""

    def __init__(self, maze):
        self.lines = maze.maze
        # self.lines = maze.strip().split('\n')

        self.width = len(self.lines[0])
        self.height = len(self.lines)
        self.minX = 0
        self.minY = 0


    def heuristic_cost_estimate(self, n1, n2):
        ""computes the 'direct' distance between two (x,y) tuples""
        (x1, y1) = n1
        (x2, y2) = n2

        return math.hypot(x2 - x1, y2 - y1)

    def distance_between(self, n1, n2):
        ""this method always returns 1, as two 'neighbors' are always adajcent""
        return 1

    def neighbors(self, node):
        "" for a given coordinate in the maze, returns up to 4 adjacent(north,east,south,west)
            nodes that can be reached (=any adjacent coordinate that is not a wall)
        ""
        x, y = node
        l = [(x, y - 1), (x, y + 1), (x - 1, y), (x + 1, y)]
        result = []

        nx , ny = l[0]
        if ( self.minX <= nx < self.height and self.minY <= ny < self.width and self.lines[x][y].hasWall(2) == NO_WALL):
            result+=[(nx,ny)]
        nx , ny = l[1]
        if (self.minX <= nx < self.height and self.minY <= ny < self.width and self.lines[x][y].hasWall(0) == NO_WALL):
            result+=[(nx,ny)]
        nx , ny = l[2]
        if (self.minX <= nx < self.height and self.minY <= ny < self.width and self.lines[x][y].hasWall(3) == NO_WALL):
            result+=[(nx,ny)]
        nx , ny = l[3]

        if (self.minX <= nx < self.height and self.minY <= ny < self.width and self.lines[x][y].hasWall(1) == NO_WALL):
            result+=[(nx,ny)]

        # print(node,result)
        return result



if __name__ == "__main__": #OUT OF DATE
    #7,14
    mp = Maze(15, 29) #com margens de erro

    d = {(0,0) : [100, -100, 100, 100] , (0,1) : [-100, -100, 100, 100] , (0,2) : [-100, -100, -100, 100] , (0,3) : [100, -100, -100, 100] , (1,0) : [100, -100, 100, -100], (1,1) :[-100, 100, -100, -100] , (1,2) : [-100, -100, -100, -100], (1,3) : [1000, -100, -100, -100] , (2,0) : [-100, 100, 100, -100]  , (2,1) : [-100, 100, -100, 100] , (2,2) : [100, 100, -100, -100] , (2,3) :[100, 100, 100, -100]}

    t = [(8,15)]
    (x1, y1) = t[-1]
    print("clear")
    mp.addWall(0, [100, -100, 100, 100] , 0,0)
    print ("visited", mp.maze[8][15].vis)
    inter = 0
    while(True):
        t = mp.findNext((x1,y1),math.pi)
        inter += len(t)
        if(t == []):
            break
        (x1, y1) = t[-1]

        l = d[(x1-8, y1-15)]
        print("list",x1, y1, l)


        mp.addWall(0, l , x1-8, y1-15 )
        #print ("visited",mp.maze[x1][y1].vis ,"\n walls" , mp.maze[x1][y1].walls)
    print("interations: ", inter)

    # mp.addWall(0, [1, 0, 1, 1] , 0,0)
    # mp.addWall(0, [0, 0, 1, 1] , 0,2)
    # mp.addWall(0, [0, 0, 0, 1] , 0,4)
    # mp.addWall(0, [1, 0, 0, 1] , 0,6)

    # mp.addWall(0, [-1, 0, 1, 0] , 2,0)
    # #mp.addWall(0, [0, 1, 0, 0] , 1,1)
    # mp.addWall(0, [0, 0, 0, 0] , 2,4)
    # mp.addWall(0, [1, 0, 0, 0] , 2,6)
    # mp.addWall(0, [0, 1, 1, 0] , 4,0)

    # mp.addWall(0, [0, 1, 0, 1] , 4,2)
    # mp.addWall(0, [1, 1, 0, 0] , 4,4)
    # mp.addWall(0, [1, 1, 1, 0] , 4,6)



    print (mp.drawMaze())


    start = (0+8,0+15)
    goal = (2+8,3+15)

    # let's solve it
    foundPath = list(MazeSolver(mp).astar(start, goal))

    print(foundPath)
    # print the solution
    print(mp.drawMaze( list(foundPath)))
    print ('limits x: ',mp.limits[0][0] , mp.limits[0][1] )
    print('limits y: ', mp.limits[1][0] , mp.limits[1][1])
"""
