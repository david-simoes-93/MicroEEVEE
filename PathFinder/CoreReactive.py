import time
import multiprocessing
import math
import copy
from serial import SerialException
import time
import sys
import signal
import pickle
from enum import Enum

try:
    import RPi.GPIO as GPIO
except Exception as e:
    print(e)
    from FakeGpio import GPIO

import Utils
from CameraHandler import CameraHandler
from MotorHandler import MotorActuator, MovementHandler, MotorState
from LedHandler import LedActuator
from ArduinoHandler import ArduinoHandler, EmptyArduino
from MapHandler import Maze
from PathPlanner import AStar
from GuiHandler import GuiHandler, FakeGui
from OdometryHandler import OdometryHandler
from Simulator import MazeSimulator
from Utils import Location
from Eevee import Eevee
import MapHandler

SLOWEST_SPEED = 15
SLOW_SPEED = 30
FAST_SPEED = 45

TIME = 1800  # seconds
start_time = time.time()
timeout = False

gui = True
sim = True

# TODO: really avoid going over blank lines, fuck that
# TODO: if going to goal, only way is explored way, ignore unknown paths

class EeveeState(Enum):
    STOPPED = 0
    FORWARD = 1
    TURNING_LEFT_1 = 2
    TURNING_LEFT_2 = 3
    TURNING_RIGHT_1 = 4
    TURNING_RIGHT_2 = 5
    BACK_LEFT = 6
    BACK_RIGHT = 7
    STOPPING = 8

class AvgSensors():
    def __init__(self):
        self.counter = 0
        self.sensor_arr = [[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]]

    def set_reading(self, sensors):
        self.sensor_arr[self.counter] = sensors
        self.counter = (self.counter + 1) % 3

    def count_left(self):
        return self.sensor_arr[0][1] + self.sensor_arr[1][1] + self.sensor_arr[2][1]
    def count_center(self):
        return self.sensor_arr[0][2] + self.sensor_arr[1][2] + self.sensor_arr[2][2]
    def count_right(self):
        return self.sensor_arr[0][3] + self.sensor_arr[1][3] + self.sensor_arr[2][3]
    
    def far_left(self):
        return self.sensor_arr[0][0] + self.sensor_arr[1][0] + self.sensor_arr[2][0] > 1.5
    def left(self):
        return self.count_left() > 1.5
    def center(self):
        return self.count_center() > 1.5
    def right(self):
        return self.count_right() > 1.5
    def far_right(self):
        return self.sensor_arr[0][4] + self.sensor_arr[1][4] + self.sensor_arr[2][4] > 1.5
    def all_sensors(self):
        return self.far_left() and self.left() and self.center() and self.right() and self.far_right()

def explore_loop(eevee: Eevee, arduino: ArduinoHandler, led0, led1, motors: MovementHandler, cam, my_map, gui):
    sensors = AvgSensors()
    my_state = EeveeState.FORWARD

    goal_detected_at = None

    delay_counter = 0

    while True:
        # safety check
        if not arduino.get():
            blink_panic(led0, led1, motors)

        # timeout check
        if (time.time() > start_time + TIME):
            motors.stop()
            global timeout
            timeout = True
            return

        sensors.set_reading(arduino.get_ground_sensors())
        #eevee.odom(arduino.m2_encoder, arduino.m1_encoder, arduino.get_ground_sensors())
        gui.render()

        # ver sensor far right
        print(my_state)
        if my_state == EeveeState.TURNING_RIGHT_1:
            motors.slow_adapt_speed(SLOW_SPEED, -SLOWEST_SPEED)
            if not sensors.left() and not sensors.center() and not sensors.right():
                my_state = EeveeState.TURNING_RIGHT_2
        elif my_state == EeveeState.TURNING_RIGHT_2:
            motors.slow_adapt_speed(SLOW_SPEED, -SLOWEST_SPEED)
            if sensors.right() or sensors.center() or sensors.left():
                delay_counter += 1
                if delay_counter > 5:
                    my_state = EeveeState.FORWARD
        elif my_state == EeveeState.TURNING_LEFT_1:
            motors.slow_adapt_speed(-SLOWEST_SPEED, SLOW_SPEED)
            if not sensors.left() and not sensors.center() and not sensors.right():
                my_state = EeveeState.TURNING_LEFT_2
        elif my_state == EeveeState.TURNING_LEFT_2:
            motors.slow_adapt_speed(-SLOWEST_SPEED, SLOW_SPEED)
            if sensors.right() or sensors.center() or sensors.left():
                delay_counter += 1
                if delay_counter > 5:
                    my_state = EeveeState.FORWARD
        elif my_state == EeveeState.BACK_RIGHT:
            motors.slow_adapt_speed(SLOW_SPEED, -SLOW_SPEED)
            if sensors.left() or sensors.center() or sensors.right():
                my_state = EeveeState.FORWARD
        elif my_state == EeveeState.BACK_LEFT:
            motors.slow_adapt_speed(-SLOW_SPEED, SLOW_SPEED)
            if sensors.left() or sensors.center() or sensors.right():
                my_state = EeveeState.FORWARD
        elif my_state == EeveeState.FORWARD:
            if sensors.far_right():
                my_state = EeveeState.TURNING_RIGHT_1
                delay_counter = 0
            elif sensors.far_left() and not sensors.left() and not sensors.center() and not sensors.right():
                delay_counter += 1
                if delay_counter > 5:
                    my_state = EeveeState.TURNING_LEFT_1
            elif sensors.center():
                delay_counter = 0
                motors.slow_adapt_speed(SLOW_SPEED, SLOW_SPEED)
            elif sensors.right():
                delay_counter = 0
                motors.slow_adapt_speed(SLOW_SPEED, SLOWEST_SPEED)
            elif sensors.left():
                delay_counter = 0
                motors.slow_adapt_speed(SLOWEST_SPEED, SLOW_SPEED)
            else:
                delay_counter += 1
                if delay_counter > 5:
                    if sensors.count_center() > 0:
                        # we were seeing center and it's gone
                        my_state = EeveeState.BACK_RIGHT
                    else:
                        if sensors.count_right() > sensors.count_left():
                            my_state = EeveeState.BACK_RIGHT
                        else:
                            my_state = EeveeState.BACK_LEFT

        # se houver, hard right turn ate ver sensor no meio
        # ver sensor far left
        # se houver, hard left turn ate ver sensor no meio
        # se nao houver nenhum sensor
        # hard turn pa trás
        # otherwise, tentar manter sensor do meio ativo

        if arduino.get_ground_average() >= 0.8:
            delay_counter += 1
            if goal_detected_at is None:
                print(f"Is this goal? {eevee.gps}")
                goal_detected_at = eevee.gps
            motors.follow_direction(eevee.theta, eevee.theta, SLOW_SPEED)
            if delay_counter > 10:
                print("GOAL DETECTED")
                motors.stop()
                goal_indices = Maze.get_cell_indexes_from_gps_coords(eevee.sensor_positions[2])
                my_map.maze[goal_indices.x][goal_indices.y].w_goal = MapHandler.MAX_WEIGHT
                my_map.goal = my_map.maze[goal_indices.x][goal_indices.y]
                gui.render()
                return
            continue
        goal_detected_at = None
        continue
       
def wait_until_button(eevee, arduino, led0, led1, my_map, gui):
    # LEDS out
    led1.set(True)
    print("Ready to go. Press a button...")
    while True:
        if not arduino.get():
            blink_panic(led0, led1, None)
        my_map.update(eevee, arduino.get_ground_sensors())
        gui.render()
        if arduino.button0 or arduino.button1:
            break

    led1.set(False)


def blink_lights_forever(led0, led1):
    last_time = time.time()
    led0_state, led1_state = False, True

    print("Blinking, ready to go. Press a button...")
    while True:
        curr_time = time.time()

        if curr_time - last_time > 1:
            last_time = curr_time

            led0_state = not led0_state
            # led1_state = not led1_state
            led0.set(led0_state)
            # led1.set(led1_state)


def blink_panic(led0, led1, motors):
    last_time = time.time()
    led0_state, led1_state = False, True

    print("PANIC")
    while True:
        if motors is not None:
            motors.stop()
        curr_time = time.time()

        if curr_time - last_time > 0.2:
            last_time = curr_time

            led0_state = not led0_state
            led1_state = not led1_state
            led0.set(led0_state)
            led1.set(led1_state)


def main():
    global sim
    if sim:
        sim_maze = MazeSimulator()
        arduino = EmptyArduino(sim_maze)
    else:
        sim_maze = None
        try:
            arduino = ArduinoHandler()
        except SerialException:
            print("Serial connection not found")
            exit(1)
            
    cam = CameraHandler()

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    # LED
    led0 = LedActuator(26)
    led1 = LedActuator(29)

    my_map = Maze()
    my_odom = OdometryHandler(my_map, sim_maze)

    # motors
    global m1, m2
    m1 = MotorActuator(36, 37, 33)  # IN1 IN2 ENA - Right Motor
    m2 = MotorActuator(40, 38, 32)  # IN3 IN4 ENB - Left Motor
    motors = MovementHandler(m2, m1, sim_maze)
    
    # Arduino
    for _ in range(5):
        if arduino.get():
            continue
        blink_panic(led0, led1, motors)

    global gui, gui_handler
    if gui:
        gui_handler = GuiHandler(arduino, motors, my_map, sim_maze)
    else:
        gui_handler = FakeGui()

    eevee = Eevee(my_odom)

    # Wait at start until a button is pushed
    wait_until_button(eevee, arduino, led0, led1, my_map, gui_handler)

    # Explore until timeout or until goal is found
    global start_time
    start_time = time.time()
    explore_loop(eevee, arduino, led0, led1, motors, cam, my_map, gui_handler)

    with open('map.pickle', 'wb') as handle:
        pickle.dump(my_map.store_to_pickle(), handle, protocol=pickle.HIGHEST_PROTOCOL)

    # Wait forever
    blink_lights_forever(led0, led1)


m1, m2, us, gui_handler = None, None, None, None


def signal_handler(sig, frame):
    if multiprocessing.current_process().name != 'MainProcess':
        print("Thread...")
        return

    print('Terminating...')
    if m1 is not None:
        m1.set(0)
    if m2 is not None:
        m2.set(0)
    GPIO.cleanup()
    gui_handler.clean()
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()
