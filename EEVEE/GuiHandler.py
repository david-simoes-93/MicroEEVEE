import pygame
import math
import time

from USHandler import UltrasoundHandler
from MotorHandler import MovementHandler
from ArduinoHandler import ArduinoHandler

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

class GuiHandler():
    def __init__(self, arduino: ArduinoHandler, us_handler: UltrasoundHandler, motors: MovementHandler):
        self.arduino = arduino
        self.us_handler = us_handler
        self.motors = motors

        print("Enabling GUI...")
        pygame.init()
        self.screen = pygame.display.set_mode([300, 300])
        pygame.display.set_caption("EEVEE")


    def clean(self):
        pygame.display.quit()
        pygame.quit()

    def render(self):
        ir_left = self.arduino.ir0
        ir_right = self.arduino.ir1
        us_left = self.us_handler.left()
        us_front = self.us_handler.front()
        us_right = self.us_handler.right()
        us_back = self.us_handler.back()
        ground_far_left = self.arduino.ground0
        ground_left = self.arduino.ground1
        ground_mid = self.arduino.ground2
        ground_right = self.arduino.ground3
        ground_far_right = self.arduino.ground4
        motor_left = self.motors.prev_left_speed
        motor_right = self.motors.prev_right_speed


        self.screen.fill(WHITE)

        pygame.draw.circle(self.screen, BLACK, [150, 150], 10)

        pygame.draw.line(self.screen, BLUE, [148, 150], [148 - int(ir_left * 100), 150], 2)
        pygame.draw.line(self.screen, BLUE, [152, 150], [152 + int(ir_right * 100), 150], 2)

        us_left_diagonal = int(math.sqrt(2*(us_left*us_left)))
        us_right_diagonal = int(math.sqrt(2*(us_right*us_right)))
        pygame.draw.line(self.screen, GREEN, [144, 144], [144 - us_left_diagonal, 144 - us_left_diagonal], 2)
        pygame.draw.line(self.screen, GREEN, [156, 144], [156 + us_right_diagonal, 144 - us_right_diagonal], 2)
        pygame.draw.line(self.screen, GREEN, [150, 141], [150, 141 - int(us_front)], 2)
        pygame.draw.line(self.screen, GREEN, [150, 159], [150, 159 + int(us_back)], 2)

        pygame.draw.circle(self.screen, BLACK, [100, 10], 6)
        pygame.draw.circle(self.screen, BLACK, [125, 10], 6)
        pygame.draw.circle(self.screen, BLACK, [150, 10], 6)
        pygame.draw.circle(self.screen, BLACK, [175, 10], 6)
        pygame.draw.circle(self.screen, BLACK, [200, 10], 6)
        pygame.draw.circle(self.screen, BLACK if ground_far_left else WHITE, [100, 10], 5)
        pygame.draw.circle(self.screen, BLACK if ground_left else WHITE, [125, 10], 5)
        pygame.draw.circle(self.screen, BLACK if ground_mid else WHITE, [150, 10], 5)
        pygame.draw.circle(self.screen, BLACK if ground_right else WHITE, [175, 10], 5)
        pygame.draw.circle(self.screen, BLACK if ground_far_right else WHITE, [200, 10], 5)

        pygame.draw.line(self.screen, RED, [10, 150], [10, 150 - int(motor_left)], 5)
        pygame.draw.line(self.screen, RED, [290, 150], [290, 150 - int(motor_right)], 5)

        pygame.display.flip()
        time.sleep(0.1)

