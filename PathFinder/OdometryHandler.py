import math
from typing import List

import Utils
from MapHandler import Maze


# Motor parameters
# "Gear box ratio" times "Encoder, pulses per revolution"
GEAR_RATIO_times_ENCODER_PULSES = 236  # 34 * 11 but with 1.58 ratio?

# Robot dimensions
WHEEL2WHEEL_DIST = 15.9  # 14.7  # cm
WHEEL_DIAM = 6.7  # cm
WHEEL_PER = math.pi * WHEEL_DIAM

ODOMETRY_THETA_THRESHOLD = Utils.to_radian(10)
CM_PER_CELL = 12.5
HALF_LINE_WIDTH = 1.25
SQUARED_GROUND_SENSOR_DIST = Utils.GROUND_SENSOR_DISTANCE*Utils.GROUND_SENSOR_DISTANCE
GPS_ADJUSTMENT_WEIGHT = 0.3
THETA_ADJUSTMENT_WEIGHT = 0.5


class OdometryHandler():
    def __init__(self, map: Maze):
        self.map = map

    def odometry(self, encLeft: int, encRight: int, gps_x: float, gps_y: float, theta: float, ground_sensors: List[bool]):
        dLeft = (encLeft * WHEEL_PER) / GEAR_RATIO_times_ENCODER_PULSES
        dRight = (encRight * WHEEL_PER) / GEAR_RATIO_times_ENCODER_PULSES

        dCenter = (dLeft + dRight) / 2.0
        phi = (dRight - dLeft) / WHEEL2WHEEL_DIST

        gps_y = gps_y + dCenter * math.sin(theta)
        gps_x = gps_x + dCenter * math.cos(theta)
        theta = Utils.normalize_radian_angle(theta + phi)
        
        #kf = KalmanFilter()
        # param x y theta and sensors? how do i find covariance matrices tho
        # can I just guess lol

        return self.adjust_sensors(gps_x, gps_y, theta, ground_sensors)

    
    def adjust_sensors(self, gps_x: float, gps_y: float, theta: float, ground_sensors):
        # how about instead we draw lines across our positive sensors
        # line across far ones must be rigid_angle
        # line across mids must be rigid angle
        # other lines don't say much
        if (ground_sensors[0] and ground_sensors[4]) or (ground_sensors[1] and ground_sensors[3]):
            rigid_compass = Utils.get_rigid_compass(theta)
            # if we are rotated more than we should be, but not TOO much (to avoid messing with turns)
            if ODOMETRY_THETA_THRESHOLD < abs(rigid_compass-theta) < 3 * ODOMETRY_THETA_THRESHOLD:
                if theta > rigid_compass:
                    new_theta = rigid_compass + ODOMETRY_THETA_THRESHOLD
                else:
                    new_theta = rigid_compass - ODOMETRY_THETA_THRESHOLD
                print(f"right alignmnet: adjusting [theta] from {Utils.to_degree(theta):.2f} to {Utils.to_degree(new_theta):.2f}")
                theta = new_theta

        sensors_gps = [Utils.far_left_sensor_gps(gps_x, gps_y, theta),
                       Utils.left_sensor_gps(gps_x, gps_y, theta),
                       Utils.front_sensor_gps(gps_x, gps_y, theta),
                       Utils.right_sensor_gps(gps_x, gps_y, theta),
                       Utils.far_right_sensor_gps(gps_x, gps_y, theta)]

        theta_deltas = [self.get_sensor_theta_delta(ground_sensors[i], sensors_gps[i], gps_x, gps_y) for i in range(len(ground_sensors))]
        avg_theta_delta = sum([t for t in theta_deltas if t is not None])

        gps_deltas = [self.get_sensor_delta(ground_sensors[i], sensors_gps[i]) for i in range(len(ground_sensors))]
        avg_gps_delta_x = sum([t[0] for t in gps_deltas if t is not None])
        avg_gps_delta_y = sum([t[1] for t in gps_deltas if t is not None])
        if avg_gps_delta_x != 0 or avg_gps_delta_y != 0 or avg_theta_delta != 0:
            avg_gps_delta_x /= len(gps_deltas)
            avg_gps_delta_y /= len(gps_deltas)
            avg_theta_delta /= len(theta_deltas)
            print(f"adjusting [x,y,theta] from [{gps_x:.2f},{gps_y:.2f},{Utils.to_degree(theta):.2f}º] with sensors {[int(s) for s in ground_sensors]} by [{avg_gps_delta_x:.2f},{avg_gps_delta_y:.2f},{Utils.to_degree(avg_theta_delta):.2f}º]")
            gps_x = gps_x + avg_gps_delta_x*GPS_ADJUSTMENT_WEIGHT
            gps_y = gps_y + avg_gps_delta_y*GPS_ADJUSTMENT_WEIGHT
            theta = theta + avg_theta_delta*THETA_ADJUSTMENT_WEIGHT
        
        return gps_x, gps_y, theta
    
    
    def get_sensor_delta(self, sensor_val, sensor_coords):
        if not sensor_val:
            # sensor might be on a line that isn't black or anywhere else, nothing to do
            # if sensor is on a line marked in the map, then we need to actually take it into account!
            return self.get_sensor_missing_delta(sensor_coords)
        # get sensor pos in relation to nearest lines' X and Y
        sensor_delta_x = sensor_coords[0] - round(sensor_coords[0] / CM_PER_CELL)*CM_PER_CELL
        sensor_delta_y = sensor_coords[1] - round(sensor_coords[1] / CM_PER_CELL)*CM_PER_CELL

        # either sensor x or y must be a multiple of 12.5+-1.25, but not off by too far
        if HALF_LINE_WIDTH < min(abs(sensor_delta_x), abs(sensor_delta_y)) < 3 * HALF_LINE_WIDTH:
            if abs(sensor_delta_x) < abs(sensor_delta_y):
                # x is closer, so adjust x
                if sensor_delta_x > HALF_LINE_WIDTH:
                    return [-sensor_delta_x+HALF_LINE_WIDTH, 0]
                else:
                    return [-sensor_delta_x-HALF_LINE_WIDTH, 0]
            else:
                # y is closer, so adjust y
                if sensor_delta_y > HALF_LINE_WIDTH:
                    return [0, -sensor_delta_y+HALF_LINE_WIDTH]
                else:
                    return [0, -sensor_delta_y-HALF_LINE_WIDTH]
    
        return None
    
    
    def get_sensor_missing_delta(self, sensor_coords):
        # TODO test

        # we want to get sensor position in cell coords relative to center of cell
        # if map_line isn't filled in, return
        cell_coords_x, cell_coords_y = Maze.get_cell_coords_from_gps_coords(sensor_coords)                        # [5.1, 2.3]
        cell_index_x, cell_index_y = Maze.get_cell_indexes_from_cell_coords([cell_coords_x, cell_coords_y])       # [ 5 ,  2 ]
        sensor_cell_coords_rel_to_center = [cell_coords_x-cell_index_x, cell_coords_y-cell_index_y]               # [0.1, 0.3]
        sensor_cell = self.map.maze[cell_index_x][cell_index_y]
        # find closest line in cell to sensor, check if we think it should be painted black
        # use sensor_cell_coords_rel_to_center to find if sensor is on top of some line and check if that line is black!
        if abs(sensor_cell_coords_rel_to_center[0]) < HALF_LINE_WIDTH and sensor_cell_coords_rel_to_center[1] > 0:
            if not sensor_cell.down_line:
                return None
        elif abs(sensor_cell_coords_rel_to_center[0]) < HALF_LINE_WIDTH and sensor_cell_coords_rel_to_center[1] < 0:
            if not sensor_cell.up_line:
                return None
        elif sensor_cell_coords_rel_to_center[0] > 0 and abs(sensor_cell_coords_rel_to_center[1]) < HALF_LINE_WIDTH:
            if not sensor_cell.right_line:
                return None
        elif sensor_cell_coords_rel_to_center[0] < 0 and abs(sensor_cell_coords_rel_to_center[1]) < HALF_LINE_WIDTH:
            if not sensor_cell.left_line:
                return None
        
        # get sensor pos in relation to nearest lines' X and Y
        closest_line_x, closest_line_y = [round(sensor_coords[0] / CM_PER_CELL)*CM_PER_CELL,
                                          round(sensor_coords[1] / CM_PER_CELL)*CM_PER_CELL]
        sensor_delta_x = sensor_coords[0] - closest_line_x
        sensor_delta_y = sensor_coords[1] - closest_line_y

        # sensor x or y must both not be a multiple of 12.5+-1.25
        if abs(sensor_delta_x) < HALF_LINE_WIDTH and abs(sensor_delta_y) < HALF_LINE_WIDTH:
            if abs(sensor_delta_x) < abs(sensor_delta_y):
                # x is closer, so adjust x
                if sensor_delta_x > 0:
                    return [HALF_LINE_WIDTH, 0]
                else:
                    return [-HALF_LINE_WIDTH, 0]
            else:
                # y is closer, so adjust y
                if sensor_delta_y > 0:
                    return [0, HALF_LINE_WIDTH]
                else:
                    return [0, -HALF_LINE_WIDTH]
    
        return None
    
    
    def get_sensor_theta_delta(self, sensor_val, sensor_coords, gps_x, gps_y):
        if not sensor_val:
            # sensor might be on a line that isn't black or anywhere else, nothing to do
            return None
        
        # circle is given by (x - h)2 + (y - k)2 = r2
        # (x-gps_x)*(x-gps_x) + (y-gps_y)*(y-gps_y) = SQUARED_GROUND_SENSOR_DIST
        
        closest_vertical_line_x = round(sensor_coords[0] / CM_PER_CELL)*CM_PER_CELL
        closest_horizontal_line_y = round(sensor_coords[1] / CM_PER_CELL)*CM_PER_CELL

        sensor_dist_to_horizontal_line = abs(sensor_coords[1]-closest_horizontal_line_y)
        sensor_dist_to_vertical_line = abs(sensor_coords[0]-closest_vertical_line_x)
        
        # sensor coords must be a multiple of 12.5+-1.25, but not off by too far
        if sensor_dist_to_horizontal_line < sensor_dist_to_vertical_line:
            if HALF_LINE_WIDTH < sensor_dist_to_horizontal_line < 3 * HALF_LINE_WIDTH:
                if sensor_coords[1] > closest_horizontal_line_y:
                    horizontal_line_edge_y = closest_horizontal_line_y + HALF_LINE_WIDTH
                else:
                    horizontal_line_edge_y = closest_horizontal_line_y - HALF_LINE_WIDTH
                
                # solve circle equation, find intersecting X
                y_minus_gps_y = horizontal_line_edge_y-gps_y
                r2_minus_delta_y = SQUARED_GROUND_SENSOR_DIST - y_minus_gps_y*y_minus_gps_y
                if r2_minus_delta_y < 0:
                    # no contact with line? something is massively wrong
                    return None
                intersection_x_pos = math.sqrt(SQUARED_GROUND_SENSOR_DIST - y_minus_gps_y*y_minus_gps_y) + gps_x
                intersection_x_neg = -math.sqrt(SQUARED_GROUND_SENSOR_DIST - y_minus_gps_y*y_minus_gps_y) + gps_x
                
                # choose closest intersecting point
                if Utils.dist2([intersection_x_pos, horizontal_line_edge_y], sensor_coords) < Utils.dist2([intersection_x_neg, horizontal_line_edge_y], sensor_coords):
                    intersection_x = intersection_x_pos
                else:
                    intersection_x = intersection_x_neg

                angle = Utils.radian_angle_between_line_segments([gps_x, gps_y], sensor_coords,
                                                                [gps_x, gps_y], [intersection_x, horizontal_line_edge_y])
                #print(f"robot at [{gps_x:.2f},{gps_y:.2f}], sensor at [{sensor_coords[0]:.2f},{sensor_coords[1]:.2f}], should rotate {Utils.to_degree(angle):.2f} to get to intersection [{intersection_x:.2f},{horizontal_line_edge_y:.2f}]")
                return angle
        else:
            # TODO
            pass

        # GROUND_SENSOR_DISTANCE