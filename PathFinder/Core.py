import time
import multiprocessing
import math
from serial import SerialException
import time
import sys
import signal

try:
    import RPi.GPIO as GPIO
except Exception as e:
    print(e)
    from PathFinder.FakeGpio import GPIO

import Utils
from CameraHandler import CameraHandler
from MotorHandler import MotorActuator, MovementHandler, MotorState
from LedHandler import LedActuator
from ArduinoHandler import ArduinoHandler, EmptyArduino
from PathFinder.MapHandler import Maze, Cell
from PathFinder.PathPlanner import AStar
from GuiHandler import GuiHandler, FakeGui
from Simulator import MazeSimulator

SLOW_SPEED = 30
FAST_SPEED = 45

TIME = 180  # seconds
start_time = time.time()
timeout = False

gui = True
sim = True


def explore_loop(arduino: ArduinoHandler, led0, led1, motors: MovementHandler, cam, my_map, gui):
    my_x, my_y, my_theta = 0, 0, 0
    path_planner = AStar()
    beacon_cell = None

    target_theta, target_cell = 0, Cell(10, 10)

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

        my_x, my_y, my_theta = MovementHandler.odometry(
            arduino.m2_encoder, arduino.m1_encoder, my_x, my_y, my_theta)

        # if not turning --> update map, else not worth it
        if motors.state != MotorState.TURNING_RIGHT and motors.state != MotorState.TURNING_LEFT:
            my_map.update(my_x, my_y, my_theta, arduino.get_ground_sensors())
        gui.render()

        explore(my_theta, arduino, motors)


def explore(my_theta, arduino: ArduinoHandler, motors: MovementHandler):
    # turn 90º or whatever
    target_theta = 0
    if motors.state == MotorState.TURNING_LEFT:
        how_much_to_turn = Utils.normalize_radian_angle(
            target_theta - my_theta)
        print("turning LEFT", Utils.to_degree(my_theta), Utils.to_degree(
            target_theta), Utils.to_degree(how_much_to_turn))
        # just wait until finish turning
        if -math.pi / 16 < how_much_to_turn:  # 10º
            motors.stop()
        elif -math.pi / 3 < how_much_to_turn:
            motors.rotate_left(SLOW_SPEED)
        else:
            motors.rotate_left(FAST_SPEED)
        return

    if motors.state == MotorState.TURNING_RIGHT:
        how_much_to_turn = Utils.normalize_radian_angle(
            target_theta - my_theta)
        print("turning RIGHT", Utils.to_degree(my_theta), Utils.to_degree(
            target_theta), Utils.to_degree(how_much_to_turn))
        # just wait until finish turning
        if how_much_to_turn < math.pi / 16:
            motors.stop()
        elif how_much_to_turn < math.pi / 3:
            motors.rotate_right(SLOW_SPEED)
        else:
            motors.rotate_right(FAST_SPEED)
        return

    if motors.state == MotorState.FORWARD:
        # start rotating if needed
        how_much_to_turn = Utils.normalize_radian_angle(
            target_theta - my_theta)
        if how_much_to_turn > math.pi / 4:
            motors.rotate_right()
            return
        elif how_much_to_turn < -math.pi / 4:
            motors.rotate_left()
            return

        # just wait until no more line is found
        too_far: bool = False  # arduino.get_ground_average() < 0.6

        if too_far:
            motors.stop()  # in next versions podemos por a ir logo para o proximo state em vez de parar
        # elif us_handler.front() < 25:
        #    motors.follow_direction(target_theta, my_theta, SLOW_SPEED)
        else:
            motors.follow_direction(target_theta, my_theta, FAST_SPEED)
        return

    if motors.state == MotorState.STOPPED:
        motors.follow_direction(target_theta, my_theta, SLOW_SPEED)
        print("stopped, start moving")
        return

    if motors.state == MotorState.BACK:
        print("back")
        return
    if motors.state == MotorState.STOPPING:
        print("stopping")
        motors.stop()
        return


def wait_until_button(arduino, led0, led1, my_map, gui):
    # LEDS out
    led1.set(True)
    print("Ready to go. Press a button...")
    while True:
        if not arduino.get():
            blink_panic(led0, led1, None)
        my_map.update(0, 0, 0, arduino.get_ground_sensors())
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

    my_map = Maze()

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    # motors
    global m1, m2
    m1 = MotorActuator(36, 37, 33)  # IN1 IN2 ENA - Right Motor
    m2 = MotorActuator(40, 38, 32)  # IN3 IN4 ENB - Left Motor
    motors = MovementHandler(m2, m1, sim_maze)

    # LED
    led0 = LedActuator(26)
    led1 = LedActuator(29)

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

    # Wait at start until a button is pushed
    wait_until_button(arduino, led0, led1, my_map, gui_handler)

    # Explore until timeout or until goal is found
    global start_time
    start_time = time.time()
    explore_loop(arduino, led0, led1, motors, cam, my_map, gui_handler)

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
