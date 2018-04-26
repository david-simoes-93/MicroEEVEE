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

motor_command_left, motor_command_right = 0, 0
resetting_odo_turn_counter, resetting_odo_turn_cell = 0, None

while True:
    # Read sensors
    cif.readSensors()
    front_sensor = filter_buffer(front_buffer, cif.measures.irSensor[0])
    left_sensor = filter_buffer(left_buffer, cif.measures.irSensor[1])
    right_sensor = filter_buffer(right_buffer, cif.measures.irSensor[2])
    back_sensor = filter_buffer(back_buffer, cif.measures.irSensor[3])
    compass = -cif.measures.compass  # TODO compass has 4 cycle delay

    # Read cheese ground
    if pos_cheese is not None:
        if cif.measures.ground == 0:
            pos_cheese = (my_x, my_y)
            cif.setVisitingLed(1)
        else:
            cif.setVisitingLed(0)

    my_x, my_y, my_dir, prev_left, prev_right = \
        update_robot_pos(prev_left, prev_right, motor_command_left, motor_command_right,
                         my_x, my_y, compass, cif.measures.collision)
    my_map.update(my_x, my_y, left_sensor, front_sensor, right_sensor, back_sensor,
                  my_dir, cif.measures.ground, cif.measures.collision)
    my_map.render()

    if np.var(right_buffer) < 0.0005 and np.var(left_buffer) < 0.0005 and left_sensor < 0.6 and right_sensor < 0.6:
        my_x, my_y = my_map.reset_side_odometry(my_x, my_y, left_sensor, right_sensor, my_dir)

    # print("%4.2f %4.2f %4.2f %4.2f %4.2fº %d" % (
    #    left_sensor, front_sensor, right_sensor, compass, my_dir, cif.measures.ground))
    # print()

    # input("rdy?")

    if cif.measures.time == 0:
        continue

    if pos_cheese is None:
        target_path = list(path_planner.astar(my_map.my_cell, my_map.pick_exploration_target(AStar(), my_dir)))
        my_map.reset_debug_dots()
        for cell in target_path:
            my_map.add_debug_dot(cell.coords)

        next_cell_coords = my_map.get_gps_coords_from_cell_coords(target_path[1].coords)
        # if we're too far from second cell, follow first cell
        if dist_manhattan([my_x, my_y], next_cell_coords) > 2.4:
            next_cell_coords = my_map.get_gps_coords_from_cell_coords(target_path[0].coords)

        target_dir = get_angle_between_points([my_x, my_y], next_cell_coords)  # from my_pos to center of target cell

        max_speed = 0.10  # max is 0.15
        target_dir = normalize_angle(target_dir - my_dir)
        # print(pos_start, next_cell_coords, target_dir)

        # rotate in place
        if target_dir < -45 and resetting_odo_turn_counter == 0:
            motor_command_left, motor_command_right = -max_speed, max_speed
        elif target_dir > 45 and resetting_odo_turn_counter == 0:
            motor_command_left, motor_command_right = max_speed, -max_speed
        else:
            motor_command_left = (target_dir + 90) / 180 * (max_speed * 2)
            motor_command_right = (max_speed * 2) - motor_command_left

            # reactive obstacle dodge!
            speed = np.abs(motor_command_left) + np.abs(motor_command_right)
            min_sensor = np.min([front_sensor, left_sensor, right_sensor])

            # wall in front
            if front_sensor < 0.3:
                if resetting_odo_turn_cell != my_map.my_cell:
                    resetting_odo_turn_counter += 1

                motor_command_left, motor_command_right = stop_speed(prev_left, prev_right)

                if resetting_odo_turn_counter > len(front_buffer) + 1:
                    my_x, my_y = my_map.reset_odometry(my_x, my_y, front_sensor, my_dir, resetting_odo_turn_counter)
                    resetting_odo_turn_counter = 0
                    resetting_odo_turn_cell = my_map.my_cell
            else:
                resetting_odo_turn_counter = 0

            # TODO if horizontal/vertical and left/right sensors are stable (so going through some tunel),
            #   adjust odometry

            # close to left wall, ensure we are moving lil bit to the right
            if left_sensor < 0.3:
                motor_command_right = min(motor_command_right, motor_command_left * 0.9)
            # close to right wall, ensure we are moving lil bit to the left
            if right_sensor < 0.3:
                motor_command_left = min(motor_command_right * 0.9, motor_command_left)
                # el
                # if speed > min_sensor:
                #    motor_command_left *= min_sensor/speed
                #    motor_command_right *= min_sensor/speed
        # print(motor_command_left, motor_command_right)
        cif.driveMotors(motor_command_left, motor_command_right)


        # if sim_state == STOP:
        #    stop()
        #    print("End.")
        #    exit()

my_map.render(close=True)
