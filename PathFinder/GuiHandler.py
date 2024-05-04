import pygame
import math
import time

from MotorHandler import MovementHandler
from ArduinoHandler import ArduinoHandler
from Simulator import MazeSimulator
from MapHandler import Maze
import MapHandler
import Simulator


class FakeGui():
    def __init__(self):
        pass

    def render(self):
        pass

    def clean(self):
        pass


BLUE = (0, 0, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
GREY = (128, 128, 128)

CELL_RESOLUTION = 4
HALF_CELL_RESOLUTION = int(CELL_RESOLUTION / 2)

SIM_CELL_RESOLUTION = int(CELL_RESOLUTION * Simulator.CM_PER_CELL / MapHandler.CM_PER_CELL)
HALF_SIM_CELL_RESOLUTION = int(SIM_CELL_RESOLUTION / 2)
QUARTER_SIM_CELL_RESOLUTION = int(HALF_SIM_CELL_RESOLUTION / 2)

ROBOT_RADIUS = int(CELL_RESOLUTION * 10 / MapHandler.CM_PER_CELL)

class GuiHandler():
    def __init__(self, arduino: ArduinoHandler, motors: MovementHandler, map: Maze, simulator: MazeSimulator):
        self.arduino = arduino
        self.motors = motors
        self.map = map
        self.simulator = simulator

        print("Enabling GUI...")
        pygame.init()
        self.screen = pygame.display.set_mode(
            [300 + MapHandler.MAP_SIZE*CELL_RESOLUTION, MapHandler.MAP_SIZE*CELL_RESOLUTION])
        pygame.display.set_caption("EEVEE")

    def clean(self):
        pygame.display.quit()
        pygame.quit()

    def render(self):
        ir_left = self.arduino.ir0
        ir_right = self.arduino.ir1
        ground_far_left = self.arduino.ground0
        ground_left = self.arduino.ground1
        ground_mid = self.arduino.ground2
        ground_right = self.arduino.ground3
        ground_far_right = self.arduino.ground4
        motor_left = self.motors.prev_left_speed
        motor_right = self.motors.prev_right_speed

        self.screen.fill(WHITE)

        pygame.draw.circle(self.screen, BLACK, [150, 150], 100)

        pygame.draw.circle(self.screen, BLACK, [100, 10], 6)
        pygame.draw.circle(self.screen, BLACK, [125, 10], 6)
        pygame.draw.circle(self.screen, BLACK, [150, 10], 6)
        pygame.draw.circle(self.screen, BLACK, [175, 10], 6)
        pygame.draw.circle(self.screen, BLACK, [200, 10], 6)
        pygame.draw.circle(
            self.screen, BLACK if ground_far_left else WHITE, [100, 10], 5)
        pygame.draw.circle(
            self.screen, BLACK if ground_left else WHITE, [125, 10], 5)
        pygame.draw.circle(
            self.screen, BLACK if ground_mid else WHITE, [150, 10], 5)
        pygame.draw.circle(
            self.screen, BLACK if ground_right else WHITE, [175, 10], 5)
        pygame.draw.circle(
            self.screen, BLACK if ground_far_right else WHITE, [200, 10], 5)

        pygame.draw.line(self.screen, RED, [10, 150], [
                         10, 150 - int(motor_left)], 5)
        pygame.draw.line(self.screen, RED, [290, 150], [
                         290, 150 - int(motor_right)], 5)

        for x in range(MapHandler.MAP_SIZE):
            for y in range(MapHandler.MAP_SIZE):
                cell = self.map.maze[x][y]
                if cell.is_line:
                    color = BLACK
                elif cell.explored:
                    color = WHITE
                else:
                    color = GREY
                pygame.draw.rect(self.screen, color, [
                    300 + CELL_RESOLUTION*x, CELL_RESOLUTION*y, CELL_RESOLUTION, CELL_RESOLUTION])
        # draw robot
        robot_x = 300 + CELL_RESOLUTION * \
            self.map.my_cell_coords[0] + HALF_CELL_RESOLUTION
        robot_y = CELL_RESOLUTION * \
            self.map.my_cell_coords[1]+HALF_CELL_RESOLUTION
        pygame.draw.circle(self.screen, BLACK, [robot_x, robot_y],
                           ROBOT_RADIUS, 2)

        if self.simulator is not None:
            mod_x = 300 + MapHandler.MAP_SIZE*CELL_RESOLUTION / 2 - self.simulator.starting_pos[0]*SIM_CELL_RESOLUTION + HALF_CELL_RESOLUTION
            mod_y = MapHandler.MAP_SIZE*CELL_RESOLUTION / 2 - self.simulator.starting_pos[1]*SIM_CELL_RESOLUTION + HALF_CELL_RESOLUTION
            for x in range(Simulator.MAP_SIZE):
                for y in range(Simulator.MAP_SIZE):
                    cell = self.simulator.maze[x][y]
                    cell_pos_x = x*SIM_CELL_RESOLUTION + mod_x
                    cell_pos_y = y*SIM_CELL_RESOLUTION + mod_y
                    if cell.up:
                        pygame.draw.line(self.screen, GREEN, [cell_pos_x, cell_pos_y], [
                            cell_pos_x, cell_pos_y-HALF_SIM_CELL_RESOLUTION], 2)
                    if cell.down:
                        pygame.draw.line(self.screen, GREEN, [cell_pos_x, cell_pos_y], [
                            cell_pos_x, cell_pos_y+HALF_SIM_CELL_RESOLUTION], 2)
                    if cell.left:
                        pygame.draw.line(self.screen, GREEN, [cell_pos_x, cell_pos_y], [
                            cell_pos_x-HALF_SIM_CELL_RESOLUTION, cell_pos_y], 2)
                    if cell.right:
                        pygame.draw.line(self.screen, GREEN, [cell_pos_x, cell_pos_y], [
                            cell_pos_x+HALF_SIM_CELL_RESOLUTION, cell_pos_y], 2)
                    if cell.goal:
                        if cell.left or cell.right:
                            pygame.draw.rect(self.screen, GREEN, [
                                cell_pos_x-QUARTER_SIM_CELL_RESOLUTION, cell_pos_y-HALF_SIM_CELL_RESOLUTION, HALF_SIM_CELL_RESOLUTION, SIM_CELL_RESOLUTION])
                        elif cell.up or cell.down:
                            pygame.draw.rect(self.screen, GREEN, [
                                cell_pos_x-HALF_SIM_CELL_RESOLUTION, cell_pos_y-QUARTER_SIM_CELL_RESOLUTION, SIM_CELL_RESOLUTION, HALF_SIM_CELL_RESOLUTION])
                    pygame.draw.circle(
                        self.screen, BLACK, [cell_pos_x, cell_pos_y], 2)

        pygame.display.flip()
        time.sleep(0.1)
