from CiberEEVEE.croblink3 import CRobLinkAngs
from CiberEEVEE.mapping import Maze
from CiberEEVEE.pathplanner import AStar
from CiberEEVEE.utils import *
import sys
import numpy as np

# Connect to server
if len(sys.argv) < 2:
    sys.argv.append("localhost")
cif = CRobLinkAngs("EEVEE", 0, [0.0, 90.0, -90.0, 180], sys.argv[1])
if cif.status != 0:
    print("Connection refused or error")
    quit()

pos_cheese = None
my_x, my_y, prev_left, prev_right = 0, 0, 0, 0
pos_start = [my_x, my_y]
my_map = Maze()
path_planner = AStar()

front_buffer = [0, 0, 0]
left_buffer = [0, 0, 0]
right_buffer = [0, 0, 0]
back_buffer = [0, 0, 0]

prev_lefts, prev_rights, motor_command_lefts, motor_command_rights, my_xs, my_ys, collisions = \
    [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]

motor_command_left, motor_command_right = 0, 0
resetting_odo_turn_counter, resetting_odo_turn_cell = 0, None
compass, front_sensor, left_sensor, right_sensor, back_sensor, ground, my_dir = 0, 0, 0, 0, 0, False, 0

turn, did_left = 0, False
while True:
    # Read sensors
    cif.readSensors()
    if cif.measures.irSensorReady[0]:
        front_sensor = filter_buffer(front_buffer, cif.measures.irSensor[0])
    if cif.measures.irSensorReady[1]:
        left_sensor = filter_buffer(left_buffer, cif.measures.irSensor[1])
    if cif.measures.irSensorReady[2]:
        right_sensor = filter_buffer(right_buffer, cif.measures.irSensor[2])
    if cif.measures.irSensorReady[3]:
        back_sensor = filter_buffer(back_buffer, cif.measures.irSensor[3])
    delayed_compass = -cif.measures.compass if cif.measures.compassReady else my_dir
    ground = cif.measures.ground if cif.measures.groundReady else None

    if turn % 2 == 0:
        cif.requestSensors(['IRSensor0', 'IRSensor1', 'IRSensor2', 'Compass'])
    else:
        if did_left:
            cif.requestSensors(['IRSensor0', 'IRSensor1', 'Ground', 'IRSensor3'])
        else:
            cif.requestSensors(['IRSensor0', 'IRSensor2', 'Ground', 'IRSensor3'])
        did_left = not did_left
    turn += 1
    # print("--")

    """# Read cheese ground
    if pos_cheese is not None and ground is not None:
        if ground == 0:
            pos_cheese = (my_x, my_y)
            cif.setVisitingLed(1)
        else:
            cif.setVisitingLed(0)"""

    # update this cycle's compass from delayed reading
    prev_lefts = prev_lefts[1:] + [prev_left]
    prev_rights = prev_rights[1:] + [prev_right]
    motor_command_lefts = motor_command_lefts[1:] + [motor_command_left]
    motor_command_rights = motor_command_rights[1:] + [motor_command_right]
    compass = update_delayed_compass(prev_lefts, prev_rights,
                                     motor_command_lefts, motor_command_rights, delayed_compass)

    # odometry, mapping
    my_x, my_y, my_dir, prev_left, prev_right = \
        update_robot_pos(prev_left, prev_right, motor_command_left, motor_command_right,
                         my_x, my_y, compass, cif.measures.collision)

    # update odometry from 4 cycles ago: creates jitter, not sure why
    #my_xs = my_xs[1:] + [my_x]
    #my_ys = my_ys[1:] + [my_y]
    #collisions = collisions[1:] + [cif.measures.collision]
    #my_x, my_y, my_dir, prev_left, prev_right = \
    #    update_robot_pos_time_delay(prev_lefts[0], prev_rights[0], motor_command_lefts, motor_command_rights,
    #                            my_xs[0], my_ys[0], delayed_compass, collisions)

    my_map.update(my_x, my_y, left_sensor, front_sensor, right_sensor, back_sensor,
                  my_dir, cif.measures.ground, cif.measures.collision)
    my_map.render()

    # print("%4.2f %4.2f %4.2f %4.2f %4.2fº %d" % (
    #    left_sensor, front_sensor, right_sensor, compass, my_dir, cif.measures.ground))
    # print()

    # wait until simulation has started
    if cif.measures.time == 0:
        cif.driveMotors(0, 0)
        continue

    # if going on a tunnel for some time, then reset odometry based on side sensors
    if np.var(right_buffer) < 0.0005 and np.var(left_buffer) < 0.0005 and left_sensor < 0.6 and right_sensor < 0.6:
        my_x, my_y = my_map.reset_side_odometry(my_x, my_y, left_sensor, right_sensor, my_dir)

    # explore
    if pos_cheese is None:
        target_path = list(path_planner.astar(my_map.my_cell, my_map.pick_exploration_target(AStar(), my_dir)))
        my_map.reset_debug_dots()
        for cell in target_path:
            my_map.add_debug_dot(cell.coords)

        # start following the second cell in the path (since the first cell is likely to be the current one)
        next_cell_coords = my_map.get_gps_coords_from_cell_coords(target_path[1].coords)
        # if we're too far from second cell, follow first cell to its center
        if dist_manhattan([my_x, my_y], next_cell_coords) > 2.4:
            next_cell_coords = my_map.get_gps_coords_from_cell_coords(target_path[0].coords)

        # actual turning direction
        target_dir = get_angle_between_points([my_x, my_y], next_cell_coords)  # from my_pos to center of target cell

        max_speed = 0.10  # max is 0.15
        target_dir = normalize_angle(target_dir - my_dir)
        # print(pos_start, next_cell_coords, target_dir)

    # rotate in place
    if target_dir < -45 and resetting_odo_turn_counter == 0:
        motor_command_left, motor_command_right = -max_speed, max_speed
    elif target_dir > 45 and resetting_odo_turn_counter == 0:
        motor_command_left, motor_command_right = max_speed, -max_speed
    # or move forward or turning slightly
    else:
        motor_command_left = (target_dir + 90) / 180 * (max_speed * 2)
        motor_command_right = (max_speed * 2) - motor_command_left

        # reactive obstacle dodge!
        speed = np.abs(motor_command_left) + np.abs(motor_command_right)
        min_sensor = np.min([front_sensor, left_sensor, right_sensor])

        # if wall in front, lets wait a few cycles until we've filled the front buffer with the same reading
        # and reset odometry
        if front_sensor < 0.3:
            if resetting_odo_turn_cell != my_map.my_cell:
                resetting_odo_turn_counter += 1

            motor_command_left, motor_command_right = stop_speed(prev_left, prev_right)

            if resetting_odo_turn_counter > len(front_buffer) + 1:
                my_x, my_y = my_map.reset_odometry(my_x, my_y, front_sensor, my_dir)
                resetting_odo_turn_counter = 0
                resetting_odo_turn_cell = my_map.my_cell
        else:
            resetting_odo_turn_counter = 0

        # close to left wall, ensure we are moving lil bit to the right
        if left_sensor < 0.3:
            motor_command_right = min(motor_command_right, motor_command_left * 0.9)
        # close to right wall, ensure we are moving lil bit to the left
        if right_sensor < 0.3:
            motor_command_left = min(motor_command_right * 0.9, motor_command_left)

    # print(motor_command_left, motor_command_right)
    cif.driveMotors(motor_command_left, motor_command_right)

my_map.render(close=True)
