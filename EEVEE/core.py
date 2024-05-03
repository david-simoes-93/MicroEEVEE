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
    from fake_gpio import GPIO


import MotorHandler, Utils
from CameraHandler import CameraHandler
from USHandler import UltrasoundHandler
from MotorHandler import MotorActuator, MovementHandler, MotorState
from LedHandler import LedActuator
from ArduinoHandler import ArduinoHandler, EmptyArduino
from mapping import Maze, Cell
from pathplanner import AStar
from GuiHandler import GuiHandler, FakeGui

prev_my_theta = 0

slow_speed = 30
fast_speed = 45

TIME = 180  # seconds
start_time = time.time()
timeout = False

gui = True


def explore_loop(arduino: ArduinoHandler, us_handler: UltrasoundHandler, led0, led1, motors: MovementHandler, cam, my_map, gui):
    my_x, my_y, my_theta = 0, 0, 0
    path_planner = AStar()
    beacon_cell = None
    prev_distances = [2, 2, 2, 2, 2]
    prev_distances_index = 0

    target_theta, target_cell = 0, Cell(10, 10)
    modified_target_theta_due_to_wall_being_close = False

    # while white floor
    while arduino.get_ground_average() < 0.6:
        # safety check
        if not arduino.get():
            blink_panic(led0, led1, motors)

        # timeout check
        if (time.time() > start_time + TIME):
            print("timeout")
            motors.stop()
            global timeout
            timeout = True
            break

        my_x, my_y, my_theta = MotorHandler.odometry(
            arduino.m2_encoder, arduino.m1_encoder, my_x, my_y, my_theta)

        # if not turning --> update map, else not worth it
        if motors.state != MotorState.TURNING_RIGHT and motors.state != MotorState.TURNING_LEFT:
            my_map.update(my_x, my_y,
                          us_handler.left(), us_handler.front(
                          ), us_handler.right(), us_handler.back(),
                          arduino.ir0, arduino.ir1,
                          my_theta, arduino.get_ground_average())
        # print("US: %4.2f %4.2f %4.2f %4.2f" % (us_left.value, us_front.value, us_right.value, us_back.value))
        print(motors.state, "Odometry %4.2f %4.2f %5.2f, Sensors: L%4.2f F%4.2f R%4.2f" %
              (my_x, my_y, Utils.to_degree(my_theta), arduino.ir0, us_handler.front(), arduino.ir1))
        gui.render()

        if motors.state == MotorState.STOPPING:
            motors.stop()
            continue

        if motors.state == MotorState.BACK and us_handler.front() >= 10:
            motors.stop()
            continue

        # -->emergency stop condition
        if motors.state != MotorState.TURNING_LEFT and motors.state != MotorState.TURNING_RIGHT:
            if us_handler.front() < 10:  # 10cm
                print("Emergency front")
                if motors.state != MotorState.STOPPED and motors.state != MotorState.BACK:
                    print("Stopping. We could reset odometry!")
                    motors.stop()
                else:
                    print("Backing up.")
                    motors.forward(-fast_speed)
                continue

            # we arent properly rotated! lets compensate with 20º theta change
            if us_handler.left() < 10:  # 10cm
                print("Emergency front-left")
                if motors.state != MotorState.STOPPED and motors.state != MotorState.TURNING_RIGHT:
                    print("Stopping. We could reset odometry!")
                    motors.stop()
                else:
                    if motors.state == MotorState.STOPPED:
                        # replace our theta, we probably are 20º (or more) skewed to the left
                        my_theta -= math.pi / 8
                    print("Rotating a bit.")
                    motors.rotate_right(fast_speed)
                continue
            if us_handler.right() < 10:  # 10cm
                print("Emergency front-right")
                if motors.state != MotorState.STOPPED and motors.state != MotorState.TURNING_LEFT:
                    print("Stopping. We could reset odometry!")
                    motors.stop()
                else:
                    if motors.state == MotorState.STOPPED:
                        # replace our theta, we probably are 20º (or more) skewed to the right
                        my_theta += math.pi / 8
                        print("Rotating a bit.")
                    motors.rotate_left(fast_speed)
                continue

            # too close to walls on side
            # 15cm (8 are within EEVEE)
            if arduino.ir0 < 15 and not modified_target_theta_due_to_wall_being_close:
                modified_target_theta_due_to_wall_being_close = True
                target_theta += math.pi / 8
            elif arduino.ir0 > 15 and modified_target_theta_due_to_wall_being_close:
                modified_target_theta_due_to_wall_being_close = False
                target_theta -= math.pi / 8

            # 15cm (8 are within EEVEE)
            if arduino.ir1 < 15 and not modified_target_theta_due_to_wall_being_close:
                modified_target_theta_due_to_wall_being_close = True
                target_theta -= math.pi / 8
            elif arduino.ir1 > 15 and modified_target_theta_due_to_wall_being_close:
                modified_target_theta_due_to_wall_being_close = False
                target_theta += math.pi / 8

        # ---------------------------

        if motors.state == MotorState.STOPPED:
            # take a picture, find a target_cell
            beacon_position = cam.get(my_x, my_y, my_theta, led0, led1)
            if beacon_position is not None:
                # print("beacon espectation", beacon_position)
                cell_coords = my_map.get_cell_coords_from_gps_coords(
                    beacon_position)
                # print("beacon expectation (cell)", cell_coords)
                if cell_coords[0] > 20:
                    cell_coords[0] = 20
                if cell_coords[1] > 20:
                    cell_coords[1] = 20
                print("beacon expectation (cell)", cell_coords)
                beacon_cell = my_map.maze[int(
                    round(cell_coords[0]))][int(round(cell_coords[1]))]
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
                    target_theta = Utils.normalize_radian_angle(
                        Utils.get_radian_between_points(my_map.eevee, planned_path[0].coords))

                    # motors.stop()
                    # go forward slowly
                    motors.follow_direction(my_theta, my_theta, slow_speed)
                else:
                    print("path:", [str(x) for x in planned_path])
                    target_cell = planned_path[1]
                    target_theta = Utils.normalize_radian_angle(
                        Utils.get_radian_between_points(my_map.eevee, planned_path[1].coords))
                    modified_target_theta_due_to_wall_being_close = False
                    prev_distances = [2, 2, 2, 2, 2]
                    prev_distances_index = 0

                    theta_diff = Utils.normalize_radian_angle(
                        target_theta - my_theta)
                    if -math.pi / 4 < theta_diff < math.pi / 4:
                        motors.follow_direction(
                            target_theta, my_theta, fast_speed)
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
                # TODO rotate
            continue

        print(motors.state, "going somewhere", str(target_cell),
              "from", my_map.my_cell, Utils.to_degree(target_theta))
        # turn 90º or whatever
        if motors.state == MotorState.TURNING_LEFT:
            how_much_to_turn = Utils.normalize_radian_angle(
                target_theta - my_theta)
            print("turning left", Utils.to_degree(my_theta), Utils.to_degree(
                target_theta), Utils.to_degree(how_much_to_turn))
            # just wait until finish turning
            if -math.pi / 16 < how_much_to_turn:  # 10º
                motors.stop()
            elif -math.pi / 3 < how_much_to_turn:
                motors.rotate_left(slow_speed)
            else:
                motors.rotate_left(fast_speed)
            continue
        if motors.state == MotorState.TURNING_RIGHT:
            how_much_to_turn = Utils.normalize_radian_angle(
                target_theta - my_theta)
            print("turning RIGHT", Utils.to_degree(my_theta), Utils.to_degree(
                target_theta), Utils.to_degree(how_much_to_turn))
            # just wait until finish turning
            if how_much_to_turn < math.pi / 16:
                motors.stop()
            elif how_much_to_turn < math.pi / 3:
                motors.rotate_right(slow_speed)
            else:
                motors.rotate_right(fast_speed)
            continue

        if motors.state == MotorState.FORWARD:
            how_much_to_turn = Utils.normalize_radian_angle(
                target_theta - my_theta)
            if how_much_to_turn > math.pi / 4:
                motors.rotate_right()
                continue
            elif how_much_to_turn < -math.pi / 4:
                motors.rotate_left()
                continue
            print("forward")
            # just wait until i'm in the middle of next
            dist_to_cell_center = Utils.dist(my_map.eevee, target_cell.coords)

            if dist_to_cell_center < 0.25:
                # note: a new cell is going to be calculated in the next cycle
                motors.stop()  # in next versions podemos por a ir logo para o proximo state em vez de parar
            elif dist_to_cell_center > prev_distances[(prev_distances_index + 1) % 5]:
                # we're moving away from the cell
                motors.stop()
            elif us_handler.front() < 25:
                motors.follow_direction(target_theta, my_theta, slow_speed)
            else:
                motors.follow_direction(target_theta, my_theta, fast_speed)
            prev_distances[prev_distances_index] = dist_to_cell_center
            prev_distances_index = (prev_distances_index + 1) % 5
            continue


def return_loop(arduino, us_handler, led0, led1, motors, my_map, return_area, gui):
    my_x, my_y, my_theta = 0, 0, 0
    path_planner = AStar()
    home_cell = Cell(10, 10)
    prev_distances = [2, 2, 2, 2, 2]
    prev_distances_index = 0

    target_theta, target_cell = 0, Cell(10, 10)
    modified_target_theta_due_to_wall_being_close = False
    while True:
        if (time.time() > start_time + TIME):
            motors.stop()
            timeout = True
            break
        if not arduino.get():
            blink_panic(led0, led1, motors)

        my_x, my_y, my_theta = MotorHandler.odometry(
            arduino.m2_encoder, arduino.m1_encoder, my_x, my_y, my_theta)

        # if not turning --> update map, else not worth it
        if motors.state != MotorState.TURNING_RIGHT and motors.state != MotorState.TURNING_LEFT:
            my_map.update(my_x, my_y,
                          us_handler.left(), us_handler.front(), us_handler.right(), us_handler.back(),
                          arduino.ir0, arduino.ir1,
                          my_theta, arduino.get_ground_average())
        # print("US: %4.2f %4.2f %4.2f %4.2f" % (us_left.value, us_front.value, us_right.value, us_back.value))
        print(motors.state, "Odometry %4.2f %4.2f %5.2f, Sensors: L%4.2f F%4.2f R%4.2f" %
              (my_x, my_y, Utils.to_degree(my_theta), arduino.ir0, us_handler.front(), arduino.ir1))
        gui.render()

        if motors.state == MotorState.STOPPING:
            motors.stop()
            continue

        if motors.state == MotorState.BACK and us_handler.front() >= 10:
            motors.stop()
            continue

        # -->emergency stop condition
        if us_handler.front() < 10:  # 10cm
            print("Emergency front")
            if motors.state != MotorState.STOPPED and motors.state != MotorState.BACK:
                print("Stopping. We could reset odometry!")
                motors.stop()
            else:
                print("Backing up.")
                motors.forward(-fast_speed)
            continue

        # we arent properly rotated! lets compensate with 20º theta change
        if us_handler.left() < 10:  # 10cm
            print("Emergency front-left")
            if motors.state != MotorState.STOPPED and motors.state != MotorState.TURNING_RIGHT:
                print("Stopping. We could reset odometry!")
                motors.stop()
            else:
                if motors.state == MotorState.STOPPED:
                    # replace our theta, we probably are 20º (or more) skewed to the left
                    my_theta -= math.pi / 8
                print("Rotating a bit.")
                motors.rotate_right(fast_speed)
            continue
        if us_handler.right() < 10:  # 10cm
            print("Emergency front-right")
            if motors.state != MotorState.STOPPED and motors.state != MotorState.TURNING_LEFT:
                print("Stopping. We could reset odometry!")
                motors.stop()
            else:
                if motors.state == MotorState.STOPPED:
                    # replace our theta, we probably are 20º (or more) skewed to the right
                    my_theta += math.pi / 8
                    print("Rotating a bit.")
                motors.rotate_left(fast_speed)
            continue

        # too close to walls on side
        # 15cm (8 are within EEVEE)
        if arduino.ir0 < 15 and not modified_target_theta_due_to_wall_being_close:
            modified_target_theta_due_to_wall_being_close = True
            target_theta += math.pi / 8
        elif arduino.ir0 > 15 and modified_target_theta_due_to_wall_being_close:
            modified_target_theta_due_to_wall_being_close = False
            target_theta -= math.pi / 8

        # 15cm (8 are within EEVEE)
        if arduino.ir1 < 15 and not modified_target_theta_due_to_wall_being_close:
            modified_target_theta_due_to_wall_being_close = True
            target_theta -= math.pi / 8
        elif arduino.ir1 > 15 and modified_target_theta_due_to_wall_being_close:
            modified_target_theta_due_to_wall_being_close = False
            target_theta += math.pi / 8

        # ---------------------------

        if motors.state == MotorState.STOPPED:
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
                    target_theta = Utils.normalize_radian_angle(
                        Utils.get_radian_between_points(my_map.eevee, planned_path[1].coords))
                    modified_target_theta_due_to_wall_being_close = False
                    prev_distances = [2, 2, 2, 2, 2]
                    prev_distances_index = 0

                    theta_diff = Utils.normalize_radian_angle(
                        target_theta - my_theta)
                    if -math.pi / 4 < theta_diff < math.pi / 4:
                        motors.follow_direction(
                            target_theta, my_theta, fast_speed)
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

        print(motors.state, "going somewhere", str(target_cell),
              "from", my_map.my_cell, Utils.to_degree(target_theta))
        # turn 90º or whatever
        if motors.state == MotorState.TURNING_LEFT:
            how_much_to_turn = Utils.normalize_radian_angle(
                target_theta - my_theta)
            print("turning left", Utils.to_degree(my_theta), Utils.to_degree(
                target_theta), Utils.to_degree(how_much_to_turn))
            # just wait until finish turning
            if -math.pi / 16 < how_much_to_turn:  # 10º
                motors.stop()
            elif -math.pi / 3 < how_much_to_turn:
                motors.rotate_left(slow_speed)
            else:
                motors.rotate_left(fast_speed)
            continue
        if motors.state == MotorState.TURNING_RIGHT:
            how_much_to_turn = Utils.normalize_radian_angle(
                target_theta - my_theta)
            print("turning RIGHT", Utils.to_degree(my_theta), Utils.to_degree(
                target_theta), Utils.to_degree(how_much_to_turn))
            # just wait until finish turning
            if how_much_to_turn < math.pi / 16:
                motors.stop()
            elif how_much_to_turn < math.pi / 3:
                motors.rotate_right(slow_speed)
            else:
                motors.rotate_right(fast_speed)
            continue

        if motors.state == MotorState.FORWARD:
            print("forward")
            # just wait until i'm in the middle of next
            dist_to_cell_center = Utils.dist(my_map.eevee, target_cell.coords)
            if dist_to_cell_center < 0.25:
                # note: a new cell is going to be calculated in the next cycle
                motors.stop()  # in next versions podemos por a ir logo para o proximo state em vez de parar
            elif dist_to_cell_center > prev_distances[(prev_distances_index + 1) % 5]:
                # we're moving away from the cell
                motors.stop()
            elif us_handler.front() < 25:  # 25cm
                motors.follow_direction(target_theta, my_theta, 20)
            else:
                motors.follow_direction(target_theta, my_theta, fast_speed)
            continue


def wait_until_button(arduino, us_handler, led0, led1, my_map):
    # LEDS out
    led1.set(True)
    print("Ready to go. Press a button...")
    while True:
        if not arduino.get():
            blink_panic(led0, led1, None)
        my_map.update(0, 0,
                      us_handler.left(), us_handler.front(), us_handler.right(), us_handler.back(),
                      arduino.ir0, arduino.ir1,
                      0, arduino.get_ground_average())

        print("Sensors: L%4.2f F%4.2f R%4.2f" %
              (arduino.ir0, us_handler.front(), arduino.ir1))

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
    cam = CameraHandler()

    my_map = Maze()

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    # ultra sound sensors
    global us_handler
    us_handler = UltrasoundHandler(13, 16,
                                   11, 18,
                                   7, 22,
                                   12, 24)
    us_handler.start_async()

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
        if arduino.get():
            continue
        blink_panic(led0, led1, motors)

    global gui, gui_handler
    if gui:
        gui_handler = GuiHandler(arduino, us_handler, motors)
    else:
        gui_handler = FakeGui()

    # Wait at start until a button is pushed
    wait_until_button(arduino, us_handler, led0, led1, my_map)

    # Explore until timeout or until goal is found
    explore_loop(arduino, us_handler, led0, led1, motors, cam, my_map, gui_handler)

    # If we didn't time-out, return to start
    if not timeout:
        led0.set(True)
        return_loop(arduino, us_handler, led0, led1, motors, my_map, None, gui_handler)

    # Wait forever
    blink_lights_forever(led0, led1)


m1, m2, us, gui_handler = None, None, None, None


def signal_handler(sig, frame):
    if multiprocessing.current_process().name != 'MainProcess':
        print("Thread...")
        return

    print('Terminating...')
    if us_handler is not None:
        us_handler.keep_running.value = False
        time.sleep(0.5)
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
