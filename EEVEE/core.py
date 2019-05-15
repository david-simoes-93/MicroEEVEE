import time
from multiprocessing import Process, Value
import multiprocessing
from EEVEE import MotorHandler, Utils
from EEVEE.CameraHandler import CameraHandler
from EEVEE.USHandler import us_async
from EEVEE.MotorHandler import MotorActuator, MovementHandler
from EEVEE.LedHandler import LedActuator
import RPi.GPIO as GPIO
from EEVEE.ArduinoHandler import ArduinoHandler, EmptyArduino
import pygame
import math
from serial import SerialException
import time
import sys
import signal
from EEVEE.mapping import Maze, Cell
from EEVEE.pathplanner import AStar
from EEVEE.Utils import *

STATE_STOPPED = 0
STATE_LEFT = 1
STATE_RIGHT = 2
STATE_FORWARD = 3

prev_my_theta = 0

slow_speed = 30
fast_speed = 45


def explore_loop(arduino, us_left, us_front, us_right, us_back, led0, led1, motors, cam, my_map):
    my_x, my_y, my_theta = 0, 0, 0
    path_planner = AStar()
    beacon_cell = None
    prev_distances = [2, 2, 2, 2, 2]
    prev_distances_index = 0

    """ 
    problems:
        1. na mediana podemos estar a retornar o 2.00 pois pode haver demasiado ruido
        2. ...
        """
    target_theta, target_cell = 0, Cell(10, 10)
    modified_target_theta_due_to_wall_being_close = False
    while arduino.get_ground_average()<0.6:
        if not arduino.get():
            blink_panic(led0, led1, motors)
        my_x, my_y, my_theta = MotorHandler.odometry(arduino.m2_encoder, arduino.m1_encoder, my_x, my_y, my_theta)

        # if not turning --> update map, else not worth it
        if motors.state != TURNING_RIGHT and motors.state != TURNING_LEFT:
            my_map.update(my_x, my_y,
                          us_left.value * 100, us_front.value * 100, us_right.value * 100, us_back.value * 100,
                          arduino.ir0, arduino.ir1,
                          my_theta, arduino.get_ground_average())
        # print("US: %4.2f %4.2f %4.2f %4.2f" % (us_left.value, us_front.value, us_right.value, us_back.value))
        print(motors.state, "Odometry %4.2f %4.2f %5.2f, Sensors: L%4.2f F%4.2f R%4.2f" %
              (my_x, my_y, to_degree(my_theta), arduino.ir0, us_front.value * 100, arduino.ir1))

        if motors.state == STOPPING:
            motors.stop()
            continue

        if motors.state == BACK and us_front.value >= 0.10:
            motors.stop()
            continue

        # -->emergency stop condition
        if motors.state != TURNING_LEFT and motors.state != TURNING_RIGHT:
            if us_front.value < 0.10:  # 10cm
                print("Emergency front")
                if motors.state != STOPPED and motors.state != BACK:
                    print("Stopping. We could reset odometry!")
                    motors.stop()
                else:
                    print("Backing up.")
                    motors.forward(-fast_speed)
                continue

            # we arent properly rotated! lets compensate with 20º theta change
            if us_left.value < 0.10:  # 10cm
                print("Emergency front-left")
                if motors.state != STOPPED and motors.state != TURNING_RIGHT:
                    print("Stopping. We could reset odometry!")
                    motors.stop()
                else:
                    if motors.state == STOPPED:
                        # replace our theta, we probably are 20º (or more) skewed to the left
                        my_theta -= math.pi / 8
                    print("Rotating a bit.")
                    motors.rotate_right(fast_speed)
                continue
            if us_right.value < 0.10:  # 10cm
                print("Emergency front-right")
                if motors.state != STOPPED and motors.state != TURNING_LEFT:
                    print("Stopping. We could reset odometry!")
                    motors.stop()
                else:
                    if motors.state == STOPPED:
                        # replace our theta, we probably are 20º (or more) skewed to the right
                        my_theta += math.pi / 8
                        print("Rotating a bit.")
                    motors.rotate_left(fast_speed)
                continue

            # too close to walls on side
            if arduino.ir0 < 15 and not modified_target_theta_due_to_wall_being_close:  # 15cm (8 are within EEVEE)
                modified_target_theta_due_to_wall_being_close = True
                target_theta += math.pi / 8
            elif arduino.ir0 > 15 and modified_target_theta_due_to_wall_being_close:
                modified_target_theta_due_to_wall_being_close = False
                target_theta -= math.pi / 8

            if arduino.ir1 < 15 and not modified_target_theta_due_to_wall_being_close:  # 15cm (8 are within EEVEE)
                modified_target_theta_due_to_wall_being_close = True
                target_theta -= math.pi / 8
            elif arduino.ir1 > 15 and modified_target_theta_due_to_wall_being_close:
                modified_target_theta_due_to_wall_being_close = False
                target_theta += math.pi / 8

        # ---------------------------

        if motors.state == STOPPED:
            # take a picture, find a target_cell
            beacon_position = cam.get(my_x, my_y, my_theta, led0, led1)
            if beacon_position is not None:
                # print("beacon espectation", beacon_position)
                cell_coords = my_map.get_cell_coords_from_gps_coords(beacon_position)
                # print("beacon expectation (cell)", cell_coords)
                if cell_coords[0] > 20:
                    cell_coords[0] = 20
                if cell_coords[1] > 20:
                    cell_coords[1] = 20
                print("beacon expectation (cell)", cell_coords)
                beacon_cell = my_map.maze[int(round(cell_coords[0]))][int(round(cell_coords[1]))]
            arduino.clear()

            # follow a target
            astar_target = beacon_cell if beacon_cell is not None else \
                my_map.pick_exploration_target(path_planner, my_theta)
            planned_path = path_planner.astar(my_map.my_cell, astar_target)
            if planned_path is not None:
                planned_path = list(planned_path)

            print("Current final destination", str(astar_target))

            if planned_path is not None:
                # print([str(x) for x in planned_path])

                # beacon not found. a new photo has to be taken in the next cycle
                if len(planned_path) == 1:
                    print("Got to destination")
                    target_cell = planned_path[0]
                    target_theta = normalize_radian_angle(
                        get_radian_between_points(my_map.eevee, planned_path[0].coords))

                    #motors.stop()
                    # go forward slowly
                    motors.follow_direction(my_theta, my_theta, slow_speed)
                else:
                    print("path:", [str(x) for x in planned_path])
                    target_cell = planned_path[1]
                    target_theta = normalize_radian_angle(
                        get_radian_between_points(my_map.eevee, planned_path[1].coords))
                    modified_target_theta_due_to_wall_being_close = False
                    prev_distances = [2, 2, 2, 2, 2]
                    prev_distances_index = 0

                    theta_diff = normalize_radian_angle(target_theta - my_theta)
                    if -math.pi / 4 < theta_diff < math.pi / 4:
                        motors.follow_direction(target_theta, my_theta, fast_speed)
                    elif -3 * math.pi / 4 < theta_diff < -math.pi / 4:
                        motors.rotate_left(fast_speed)
                    elif math.pi / 4 < theta_diff < 3 * math.pi / 4:
                        motors.rotate_right(fast_speed)
                    else:
                        # back
                        if theta_diff < 0:
                            motors.rotate_left(fast_speed)
                        else:
                            motors.rotate_right(fast_speed)
            else:
                print("path not found")
                #TODO rotate
            continue

        print(motors.state, "going somewhere", str(target_cell), "from", my_map.my_cell, to_degree(target_theta))
        # turn 90º or whatever
        if motors.state == TURNING_LEFT:
            how_much_to_turn = normalize_radian_angle(target_theta - my_theta)
            print("turning left", to_degree(my_theta), to_degree(target_theta), to_degree(how_much_to_turn))
            # just wait until finish turning
            if -math.pi / 16 < how_much_to_turn:  # 10º
                motors.stop()
            elif -math.pi / 3 < how_much_to_turn:
                motors.rotate_left(slow_speed)
            else:
                motors.rotate_left(fast_speed)
            continue
        if motors.state == TURNING_RIGHT:
            how_much_to_turn = normalize_radian_angle(target_theta - my_theta)
            print("turning RIGHT", to_degree(my_theta), to_degree(target_theta), to_degree(how_much_to_turn))
            # just wait until finish turning
            if how_much_to_turn < math.pi / 16:
                motors.stop()
            elif how_much_to_turn < math.pi / 3:
                motors.rotate_right(slow_speed)
            else:
                motors.rotate_right(fast_speed)
            continue

        if motors.state == FORWARD:
            how_much_to_turn = normalize_radian_angle(target_theta - my_theta)
            if how_much_to_turn > math.pi/4:
                motors.rotate_right()
                continue
            elif how_much_to_turn < -math.pi/4:
                motors.rotate_left()
                continue
            print("forward")
            # just wait until i'm in the middle of next
            dist_to_cell_center = dist(my_map.eevee, target_cell.coords)

            if dist_to_cell_center < 0.25:
                # note: a new cell is going to be calculated in the next cycle
                motors.stop()  # in next versions podemos por a ir logo para o proximo state em vez de parar
            elif dist_to_cell_center > prev_distances[(prev_distances_index + 1) % 5]:
                # we're moving away from the cell
                motors.stop()
            elif us_front.value < 0.25:
                motors.follow_direction(target_theta, my_theta, slow_speed)
            else:
                motors.follow_direction(target_theta, my_theta, fast_speed)
            prev_distances[prev_distances_index] = dist_to_cell_center
            prev_distances_index = (prev_distances_index + 1) % 5
            continue

        ## -----------------------------------old stuff-------------------------------------

        """ planned_path = list(path_planner.astar(my_map.my_cell, target_cell))
        if planned_path is not None:
            print([str(x) for x in planned_path])

        print("US: %4.2f %4.2f %4.2f %4.2f" % (us_left.value, us_front.value, us_right.value, us_back.value))
        # print("IR:",arduino.ir0, arduino.ir1)
        # print("Buttons:",arduino.button0, arduino.button1)
        # print("Ground:",arduino.ground0, arduino.ground1, arduino.ground2, arduino.ground3, arduino.ground4)
        # print("Pose: (%5.2f, %5.2f) %5.2fº" % (my_x, my_y, Utils.to_degree(my_theta)))

        if motors.state == STOPPED:
            beacon = cam.get()
            if beacon is not None:
                print("Beacon!", beacon)

        if motors.state == STOPPING:
            print("Stopping")
            motors.stop()
            continue

        # shouldnt happen :|
        if planned_path is None:
            print("None path")
            motors.follow_direction(my_theta, my_theta, 35)
            continue

        if len(planned_path) == 1:
            print("Got to destination")
            motors.stop()
            continue

        target_dist = dist(my_map.eevee, planned_path[1].coords)
        target_dir = normalize_radian_angle(to_radian(
            get_angle_between_points(my_map.eevee, planned_path[1].coords)))
        print("Target dir:", to_degree(target_dir), "target dist", target_dist)

        if target_dir > math.pi / 6:
            print("turning right")
            if motors.state != TURNING_RIGHT and motors.state != STOPPED:
                motors.stop()
                continue
            motors.rotate_right()
            continue

        if target_dir < -math.pi / 6:
            print("turning left")
            if motors.state != TURNING_LEFT and motors.state != STOPPED:
                motors.stop()
                continue
            motors.rotate_left()

        if target_dist < 0.25:
            motors.stop()
        else:
            motors.follow_direction(target_dir, my_theta, 25)
        # print("moved forward") """

        """
        if my_x > 100:
            print("MOVING BACKWARDS")
            moving_forward = False
        elif my_x < -100:
            print("MOVING FORWARDS")
            moving_forward = True

        if moving_forward:
            motors.follow_direction(0, my_theta, 35)
        else:
            motors.follow_direction(0, my_theta, -35)"""


def return_loop(arduino, us_left, us_front, us_right, us_back, led0, led1, motors, my_map, return_area):
    my_x, my_y, my_theta = 0, 0, 0
    path_planner = AStar()
    home_cell = Cell(10, 10)
    prev_distances = [2, 2, 2, 2, 2]
    prev_distances_index = 0

    target_theta, target_cell = 0, Cell(10, 10)
    modified_target_theta_due_to_wall_being_close = False
    while not beacon_area_detected(arduino):
        if not arduino.get():
            blink_panic(led0, led1, motors)
        my_x, my_y, my_theta = MotorHandler.odometry(arduino.m2_encoder, arduino.m1_encoder, my_x, my_y, my_theta)

        # if not turning --> update map, else not worth it
        if motors.state != TURNING_RIGHT and motors.state != TURNING_LEFT:
            my_map.update(my_x, my_y,
                          us_left.value * 100, us_front.value * 100, us_right.value * 100, us_back.value * 100,
                          arduino.ir0, arduino.ir1,
                          my_theta, arduino.get_ground_average())
        # print("US: %4.2f %4.2f %4.2f %4.2f" % (us_left.value, us_front.value, us_right.value, us_back.value))
        print(motors.state, "Odometry %4.2f %4.2f %5.2f, Sensors: L%4.2f F%4.2f R%4.2f" %
              (my_x, my_y, to_degree(my_theta), arduino.ir0, us_front.value * 100, arduino.ir1))

        if motors.state == STOPPING:
            motors.stop()
            continue

        if motors.state == BACK and us_front.value >= 0.10:
            motors.stop()
            continue

        # -->emergency stop condition
        if us_front.value < 0.10:  # 10cm
            print("Emergency front")
            if motors.state != STOPPED and motors.state != BACK:
                print("Stopping. We could reset odometry!")
                motors.stop()
            else:
                print("Backing up.")
                motors.forward(-fast_speed)
            continue

        # we arent properly rotated! lets compensate with 20º theta change
        if us_left.value < 0.10:  # 10cm
            print("Emergency front-left")
            if motors.state != STOPPED and motors.state != TURNING_RIGHT:
                print("Stopping. We could reset odometry!")
                motors.stop()
            else:
                if motors.state == STOPPED:
                    # replace our theta, we probably are 20º (or more) skewed to the left
                    my_theta -= math.pi / 8
                print("Rotating a bit.")
                motors.rotate_right(fast_speed)
            continue
        if us_right.value < 0.10:  # 10cm
            print("Emergency front-right")
            if motors.state != STOPPED and motors.state != TURNING_LEFT:
                print("Stopping. We could reset odometry!")
                motors.stop()
            else:
                if motors.state == STOPPED:
                    # replace our theta, we probably are 20º (or more) skewed to the right
                    my_theta += math.pi / 8
                    print("Rotating a bit.")
                motors.rotate_left(fast_speed)
            continue

        # too close to walls on side
        if arduino.ir0 < 15 and not modified_target_theta_due_to_wall_being_close:  # 15cm (8 are within EEVEE)
            modified_target_theta_due_to_wall_being_close = True
            target_theta += math.pi / 8
        elif arduino.ir0 > 15 and modified_target_theta_due_to_wall_being_close:
            modified_target_theta_due_to_wall_being_close = False
            target_theta -= math.pi / 8

        if arduino.ir1 < 15 and not modified_target_theta_due_to_wall_being_close:  # 15cm (8 are within EEVEE)
            modified_target_theta_due_to_wall_being_close = True
            target_theta -= math.pi / 8
        elif arduino.ir1 > 15 and modified_target_theta_due_to_wall_being_close:
            modified_target_theta_due_to_wall_being_close = False
            target_theta += math.pi / 8

        # ---------------------------

        if motors.state == STOPPED:
            # follow a target
            planned_path = path_planner.astar(my_map.my_cell, home_cell)
            if planned_path is not None:
                planned_path = list(planned_path)
            else:
                print("PLANNED PATH = NONE")

            if planned_path is not None:
                # print([str(x) for x in planned_path])

                if len(planned_path) == 1:
                    print("Got to destination")
                    target_cell = None
                    motors.stop()
                else:
                    print("path:", [str(x) for x in planned_path])
                    target_cell = planned_path[1]
                    target_theta = normalize_radian_angle(
                        get_radian_between_points(my_map.eevee, planned_path[1].coords))
                    modified_target_theta_due_to_wall_being_close = False
                    prev_distances = [2, 2, 2, 2, 2]
                    prev_distances_index = 0

                    theta_diff = normalize_radian_angle(target_theta - my_theta)
                    if -math.pi / 4 < theta_diff < math.pi / 4:
                        motors.follow_direction(target_theta, my_theta, fast_speed)
                    elif -3 * math.pi / 4 < theta_diff < -math.pi / 4:
                        motors.rotate_left(fast_speed)
                    elif math.pi / 4 < theta_diff < 3 * math.pi / 4:
                        motors.rotate_right(fast_speed)
                    else:
                        # back
                        if theta_diff < 0:
                            motors.rotate_left(fast_speed)
                        else:
                            motors.rotate_right(fast_speed)
            else:
                print("path not found")
            continue

        print(motors.state, "going somewhere", str(target_cell), "from", my_map.my_cell, to_degree(target_theta))
        # turn 90º or whatever
        if motors.state == TURNING_LEFT:
            how_much_to_turn = normalize_radian_angle(target_theta - my_theta)
            print("turning left", to_degree(my_theta), to_degree(target_theta), to_degree(how_much_to_turn))
            # just wait until finish turning
            if -math.pi / 16 < how_much_to_turn:  # 10º
                motors.stop()
            elif -math.pi / 3 < how_much_to_turn:
                motors.rotate_left(slow_speed)
            else:
                motors.rotate_left(fast_speed)
            continue
        if motors.state == TURNING_RIGHT:
            how_much_to_turn = normalize_radian_angle(target_theta - my_theta)
            print("turning RIGHT", to_degree(my_theta), to_degree(target_theta), to_degree(how_much_to_turn))
            # just wait until finish turning
            if how_much_to_turn < math.pi / 16:
                motors.stop()
            elif how_much_to_turn < math.pi / 3:
                motors.rotate_right(slow_speed)
            else:
                motors.rotate_right(fast_speed)
            continue

        if motors.state == FORWARD:
            print("forward")
            # just wait until i'm in the middle of next
            dist_to_cell_center = dist(my_map.eevee, target_cell.coords)
            if dist_to_cell_center < 0.25:
                # note: a new cell is going to be calculated in the next cycle
                motors.stop()  # in next versions podemos por a ir logo para o proximo state em vez de parar
            elif dist_to_cell_center > prev_distances[(prev_distances_index + 1) % 5]:
                # we're moving away from the cell
                motors.stop()
            elif us_front.value < 0.25: #25cm
                motors.follow_direction(target_theta, my_theta, 20)
            else:
                motors.follow_direction(target_theta, my_theta, fast_speed)
            continue


def beacon_area_detected(arduino):
    # arduino.ground1 is always true :|

    if not arduino.ground0 and not arduino.ground1 and not arduino.ground2 \
            and not arduino.ground3 and not arduino.ground4:
        print("Beacon found!", arduino.ground0, arduino.ground1, arduino.ground2, arduino.ground3, arduino.ground4)
        return True
    return False


def render(screen, ir_left, ir_right, us_left, us_front, us_right, us_back,
           ground_far_left, ground_left, ground_mid, ground_right, ground_far_right,
           motor_left, motor_right):
    white, black = (255, 255, 255), (0, 0, 0)
    screen.fill(white)

    pygame.draw.circle(screen, black, [150, 150], 10)

    pygame.draw.line(screen, (0, 0, 255), [148, 150], [148 - int(ir_left * 100), 150], 2)
    pygame.draw.line(screen, (0, 0, 255), [152, 150], [152 + int(ir_right * 100), 150], 2)

    us_left_xy, us_right_xy = int(math.sqrt(us_left) / 2 * 100), int(math.sqrt(us_right) / 2 * 100)
    pygame.draw.line(screen, (0, 255, 0), [144, 144], [144 - us_left_xy, 144 - us_left_xy], 2)
    pygame.draw.line(screen, (0, 255, 0), [156, 144], [156 + us_right_xy, 144 - us_right_xy], 2)
    pygame.draw.line(screen, (0, 255, 0), [150, 141], [150, 141 - int(us_front * 100)], 2)
    pygame.draw.line(screen, (0, 255, 0), [150, 159], [150, 159 + int(us_back * 100)], 2)

    pygame.draw.circle(screen, black, [100, 10], 6)
    pygame.draw.circle(screen, black, [125, 10], 6)
    pygame.draw.circle(screen, black, [150, 10], 6)
    pygame.draw.circle(screen, black, [175, 10], 6)
    pygame.draw.circle(screen, black, [200, 10], 6)
    pygame.draw.circle(screen, black if ground_far_left else white, [100, 10], 5)
    pygame.draw.circle(screen, black if ground_left else white, [125, 10], 5)
    pygame.draw.circle(screen, black if ground_mid else white, [150, 10], 5)
    pygame.draw.circle(screen, black if ground_right else white, [175, 10], 5)
    pygame.draw.circle(screen, black if ground_far_right else white, [200, 10], 5)

    pygame.draw.line(screen, (255, 0, 0), [10, 150], [10, 150 - int(motor_left)], 5)
    pygame.draw.line(screen, (255, 0, 0), [290, 150], [290, 150 - int(motor_right)], 5)

    pygame.display.flip()


def blink_lights_until_button(arduino, us_left, us_front, us_right, us_back, led0, led1, my_map):
    last_time = time.time()
    led0_state, led1_state = False, True

    print("Ready to go. Press a button...")
    while True:
        if not arduino.get():
            blink_panic(led0, led1, None)
        my_map.update(0, 0,
                      us_left.value * 100, us_front.value * 100, us_right.value * 100, us_back.value * 100,
                      arduino.ir0, arduino.ir1,
                      0, arduino.get_ground_average())

        print("Sensors: L%4.2f F%4.2f R%4.2f" % (arduino.ir0, us_front.value * 100, arduino.ir1))

        if arduino.button0 or arduino.button1:
            break
        curr_time = time.time()

        if curr_time - last_time > 1:
            last_time = curr_time

            led0_state = not led0_state
            led1_state = not led1_state
            led0.set(led0_state)
            led1.set(led1_state)

    led0.set(False)
    led1.set(False)


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
    cam = CameraHandler()

    my_map = Maze()

    """gui = False
    remote_control = False"""

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    # asynchronously update US sensors
    us_left, us_front, us_right, us_back = Value('f', 0), Value('f', 0), Value('f', 0), Value('f', 0)
    global keep_running_us
    keep_running_us = Value('b', True)
    Process(target=us_async, args=(keep_running_us, 13, 16, us_back, 11, 18, us_right,
                                   7, 22, us_front, 12, 24, us_left)).start()

    # motors
    global m1, m2
    m1 = MotorActuator(36, 37, 33)  # IN1 IN2 ENA - Right Motor
    m2 = MotorActuator(40, 38, 32)  # IN3 IN4 ENB - Left Motor
    motors = MovementHandler(m2, m1)

    # LED
    led0 = LedActuator(26)
    led1 = LedActuator(29)

    # Arduino
    try:
        arduino = ArduinoHandler()
    except SerialException:
        print("Serial connection not found")
        arduino = EmptyArduino()

    for _ in range(5):
        if not arduino.get():
            blink_panic(led0, led1, motors)

    """if gui:
        print("Enabling GUI...")
        pygame.init()
        screen = pygame.display.set_mode([300, 300])
        pygame.display.set_caption("EEVEE")"""

    # test_turning_odometry(arduino, motors)

    blink_lights_until_button(arduino, us_left, us_front, us_right, us_back, led0, led1, my_map)

    explore_loop(arduino, us_left, us_front, us_right, us_back, led0, led1, motors, cam, my_map)

    blink_lights_until_button(arduino, led0, led1)

    return_loop(arduino, us_left, us_front, us_right, us_back, led0, led1, motors, my_map, None)

    while True:
        blink_lights_until_button(arduino, led0, led1)

    # irrelevant legacy code
    """
    while True:
        arduino.get()
        if not gui:
            print(us_left.value, us_front.value, us_right.value, us_back.value)
            print(arduino.ir0, arduino.ir1)
            print(arduino.button0, arduino.button1)
            print(arduino.ground0, arduino.ground1, arduino.ground2, arduino.ground3, arduino.ground4)
            # print(arduino.m1_encoder, arduino.m2_encoder)
            movedl += arduino.m1_encoder
            movedr += arduino.m2_encoder
            print("Enc", movedl, movedr)

        led0.set(us_front.value < 0.20)
        led1.set(us_left.value < 0.20 or us_right.value < 0.20)

        if remote_control:
            if left_motor_speed > 0:
                left_motor_speed -= 5
            elif left_motor_speed < 0:
                left_motor_speed += 5
            if right_motor_speed > 0:
                right_motor_speed -= 5
            elif right_motor_speed < 0:
                right_motor_speed += 5

            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_LEFT:
                        left_motor_speed -= 10
                        right_motor_speed += 10
                    elif event.key == pygame.K_RIGHT:
                        left_motor_speed += 10
                        right_motor_speed -= 10
                    if event.key == pygame.K_UP:
                        left_motor_speed += 10
                        right_motor_speed += 10
                    elif event.key == pygame.K_DOWN:
                        left_motor_speed -= 10
                        right_motor_speed -= 10

                    if event.key == pygame.K_SPACE:
                        left_motor_speed = 0
                        right_motor_speed = 0
                    if event.key == pygame.K_ESCAPE:
                        m1.set(0)
                        m2.set(0)
                        pygame.quit()
                        keep_running_us.value = False
                        exit()

        left_motor_speed = max(min(100, left_motor_speed), -100)
        right_motor_speed = max(min(100, right_motor_speed), -100)
        m1.set(left_motor_speed)
        m2.set(right_motor_speed)

        if gui:
            render(screen, arduino.ir0, arduino.ir1, us_left.value, us_front.value, us_right.value, us_back.value,
                   arduino.ground0, arduino.ground1, arduino.ground2, arduino.ground3, arduino.ground4,
                   left_motor_speed, right_motor_speed)"""


m1, m2, keep_running_us = None, None, None


def test_moving_odometry(arduino, motors):
    my_x, my_y, my_theta = 0, 0, 0
    encL, encR = 0, 0
    arduino.get()

    while my_x < 100:
        arduino.get()
        encL += arduino.m2_encoder
        encR += arduino.m1_encoder
        my_x, my_y, my_theta = MotorHandler.odometry(arduino.m2_encoder, arduino.m1_encoder, my_x, my_y, my_theta)
        motors.forward(slow_speed)
        print(encL, encR, to_degree(my_theta), my_x, my_y)
    while True:
        motors.stop()
        arduino.get()
        encL += arduino.m2_encoder
        encR += arduino.m1_encoder
        my_x, my_y, my_theta = MotorHandler.odometry(arduino.m2_encoder, arduino.m1_encoder, my_x, my_y, my_theta)
        print(encL, encR, to_degree(my_theta), my_x, my_y)


def test_turning_odometry(arduino, motors):
    my_x, my_y, my_theta = 0, 0, 0
    encL, encR = 0, 0
    arduino.get()

    while my_theta < to_radian(80):
        arduino.get()
        encL += arduino.m2_encoder
        encR += arduino.m1_encoder
        my_x, my_y, my_theta = MotorHandler.odometry(arduino.m2_encoder, arduino.m1_encoder, my_x, my_y, my_theta)
        motors.rotate_right(slow_speed)
        print(encL, encR, to_degree(my_theta), my_x, my_y)
    while True:
        motors.stop()
        arduino.get()
        encL += arduino.m2_encoder
        encR += arduino.m1_encoder
        my_x, my_y, my_theta = MotorHandler.odometry(arduino.m2_encoder, arduino.m1_encoder, my_x, my_y, my_theta)
        print(encL, encR, to_degree(my_theta), my_x, my_y)


def signal_handler(sig, frame):
    if multiprocessing.current_process().name != 'MainProcess':
        print("Thread...")
        return

    print('Terminating...')
    if keep_running_us is not None:
        keep_running_us.value = False
        time.sleep(0.5)
    if m1 is not None:
        m1.set(0)
    if m2 is not None:
        m2.set(0)
    GPIO.cleanup()
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()
