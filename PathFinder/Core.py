import time
import multiprocessing
import math
import copy
from serial import SerialException
import time
import sys
import signal
import pickle

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

TIME = 180  # seconds
start_time = time.time()
timeout = False

gui = True
sim = True

# TODO: really avoid going over blank lines, fuck that
# TODO: if going to goal, only way is explored way, ignore unknown paths

def explore_loop(eevee: Eevee, arduino: ArduinoHandler, led0, led1, motors: MovementHandler, cam, my_map, gui):
    path_planner = AStar()
    target_cell = None
    my_cell = None

    path_curr_cell = my_map.my_cell
    path_prev_cell = path_curr_cell.neighbor_left

    goal_detected_at = None

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

        eevee.odom(arduino.m2_encoder, arduino.m1_encoder, arduino.get_ground_sensors())
        my_map.update(eevee, arduino.get_ground_sensors())
        gui.render()

        if my_map.my_cell != path_curr_cell:
            path_prev_cell = path_curr_cell
            path_curr_cell = my_map.my_cell

        if arduino.get_ground_average() >= 0.8:
            if goal_detected_at is None:
                print(f"Is this goal? {eevee.gps}")
                goal_detected_at = eevee.gps
            motors.follow_direction(eevee.theta, eevee.theta, SLOW_SPEED)
            if Utils.dist(goal_detected_at, eevee.gps) > 3:
                print("GOAL DETECTED")
                motors.stop()
                goal_indices = Maze.get_cell_indexes_from_gps_coords(eevee.sensor_positions[2])
                my_map.maze[goal_indices.x][goal_indices.y].w_goal = MapHandler.MAX_WEIGHT
                my_map.goal = my_map.maze[goal_indices.x][goal_indices.y]
                gui.render()
                return
            continue
        goal_detected_at = None
        
        new_target_cell = my_map.pick_exploration_target(path_planner, path_prev_cell)
        #print(f"{my_map.my_cell} -> {new_target_cell}")
        if new_target_cell != target_cell or my_cell != my_map.my_cell:
            target_cell = new_target_cell
            my_cell = copy.copy(my_map.my_cell) # keep copy so that if lines change, we retry the path
            my_map.planned_path = path_planner.astar(my_map.my_cell, target_cell, only_sure_neighbors=my_map.goal is not None)
            if len(my_map.planned_path) < 2:
                # motors.stop()
                print(f"PATH NOT FOUND from cell {my_map.my_cell} to cell {target_cell}")
                my_map.planned_path = [my_cell, my_cell]
        if Utils.dist(eevee.gps, my_map.planned_path[0].coords) > 3.5 and Utils.dist(eevee.gps, my_map.planned_path[1].coords) > 9.5:
            next_cell = my_map.planned_path[0]
        else:
            next_cell = my_map.planned_path[1]
        target_theta = Utils.get_radian_between_points(eevee.gps, next_cell.coords)
        
        explore(eevee.theta, target_theta, arduino, motors)


def explore(my_theta: float, target_theta: float, arduino: ArduinoHandler, motors: MovementHandler):
    how_much_to_turn = Utils.normalize_radian_angle(target_theta - my_theta)
    # turn 90º or whatever
    if motors.state == MotorState.TURNING_LEFT:
        #print(f"turning LEFT from {Utils.to_degree(my_theta):.2f}º by {Utils.to_degree(how_much_to_turn):.2f}")
        # just wait until finish turning
        if -Utils.to_radian(10) < how_much_to_turn:  # 10º
            motors.stop()
        elif -Utils.to_radian(60) < how_much_to_turn:
            motors.rotate_left(SLOW_SPEED)
        else:
            motors.rotate_left(FAST_SPEED)
        return

    if motors.state == MotorState.TURNING_RIGHT:
        #print(f"turning RIGHT from {Utils.to_degree(my_theta):.2f}º by {Utils.to_degree(how_much_to_turn):.2f}")
        # just wait until finish turning
        if how_much_to_turn < Utils.to_radian(10):
            motors.stop()
        elif how_much_to_turn < Utils.to_radian(60):
            motors.rotate_right(SLOW_SPEED)
        else:
            motors.rotate_right(FAST_SPEED)
        return

    if motors.state == MotorState.FORWARD:
        # start rotating if needed
        if how_much_to_turn > Utils.to_radian(45):
            motors.rotate_right()
            return
        elif how_much_to_turn < -Utils.to_radian(45):
            motors.rotate_left()
            return

        # slow down if no more line is found
        if sum(arduino.get_ground_sensors()[1:4]) == 0:
            motors.follow_direction(target_theta, my_theta, SLOW_SPEED)
        #elif far_sensor_positions inside lines and lines unexplored:
        #    motors.follow_direction(target_theta, my_theta, SLOW_SPEED)
        else:
            motors.follow_direction(target_theta, my_theta, FAST_SPEED)
        return

    if motors.state == MotorState.STOPPED:
        motors.follow_direction(target_theta, my_theta, SLOW_SPEED)
        #print("stopped, start moving")
        return

    if motors.state == MotorState.BACK:
        #print("back")
        return
    if motors.state == MotorState.STOPPING:
        #print("stopping")
        motors.stop()
        return


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


def load_map_based_on_button(led0, led1, arduino):
    # load previous map or start fresh depending on button we pressed
    while not arduino.button0 and not arduino.button1:
        if not arduino.get():
            blink_panic(led0, led1, None)
    my_map = Maze()
    if arduino.button0:
        try:
            with open('map.pickle', 'rb') as handle:
                my_map.load_from_pickle(pickle.load(handle))
        except:
            print("Failed to load map")

    # otherwise if button1    
    return my_map


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

    my_map = load_map_based_on_button(led0, led1, arduino)
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
