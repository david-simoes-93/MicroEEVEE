import math
import copy
from typing import List

import Utils
from Utils import Location
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
HALF_LINE_WIDTH_CELL = HALF_LINE_WIDTH / CM_PER_CELL
SQUARED_GROUND_SENSOR_DIST = Utils.GROUND_SENSOR_DISTANCE*Utils.GROUND_SENSOR_DISTANCE
GPS_ADJUSTMENT_WEIGHT = 0.5
THETA_ADJUSTMENT_WEIGHT = 0.5

# TODO: map intersections só a cada 2 cells
# TODO: evitar escolher uma target cell inacessivel
# TODO: goal detection
# TODO: really avoid going over blank lines, fuck that

class OdometryHandler():
    def __init__(self, map: Maze, sim):
        self.map = map
        self.sim = sim

    def odometry(self, encLeft: int, encRight: int, gps: Location, theta: float, sensor_positions: List[Location], ground_sensors: List[bool]):
        dLeft = (encLeft * WHEEL_PER) / GEAR_RATIO_times_ENCODER_PULSES
        dRight = (encRight * WHEEL_PER) / GEAR_RATIO_times_ENCODER_PULSES

        dCenter = (dLeft + dRight) / 2.0
        phi = (dRight - dLeft) / WHEEL2WHEEL_DIST

        new_theta = Utils.normalize_radian_angle(theta + phi)
        avg_theta = Utils.avg_between_radian_angles(theta, new_theta)
        new_gps = Location(gps.x + dCenter * math.cos(avg_theta), gps.y + dCenter * math.sin(avg_theta))

        #if self.sim:
        #    print(f"    actual ground truth is  [{(self.sim.eevee_coords.x-self.sim.starting_pos.x)*CM_PER_CELL:.2f},{(self.sim.eevee_coords.y-self.sim.starting_pos.y)*CM_PER_CELL:.2f},{Utils.to_degree(self.sim.eevee_theta):.2f}º]")

        return self.adjust_sensors(new_gps, new_theta, sensor_positions, ground_sensors)

    
    def adjust_sensors(self, gps: Location, theta: float, sensors_gps, ground_sensors):
        theta_deltas = [self.get_sensor_theta_delta(ground_sensors[i], sensors_gps[i], gps) for i in range(len(ground_sensors))]
        avg_theta_delta = sum([max(min(t,ODOMETRY_THETA_THRESHOLD),-ODOMETRY_THETA_THRESHOLD) for t in theta_deltas if t is not None])

        gps_deltas = [self.get_sensor_gps_delta(ground_sensors[i], sensors_gps[i]) for i in range(len(ground_sensors))]
        avg_gps_delta_x = sum([t[0] for t in gps_deltas if t is not None])
        avg_gps_delta_y = sum([t[1] for t in gps_deltas if t is not None])

        if avg_gps_delta_x != 0 or avg_gps_delta_y != 0 or avg_theta_delta != 0:
            avg_gps_delta_x /= len(gps_deltas)
            avg_gps_delta_y /= len(gps_deltas)
            avg_theta_delta /= len(theta_deltas)
            print(f"adjusting [x,y,theta] from [{gps.x:.2f},{gps.y:.2f},{Utils.to_degree(theta):.2f}º] with sensors {[int(s) for s in ground_sensors]} by [{avg_gps_delta_x:.2f},{avg_gps_delta_y:.2f},{Utils.to_degree(avg_theta_delta):.2f}º]")
            new_gps = Location(gps.x + avg_gps_delta_x*GPS_ADJUSTMENT_WEIGHT, gps.y + avg_gps_delta_y*GPS_ADJUSTMENT_WEIGHT)
            theta = theta + avg_theta_delta*THETA_ADJUSTMENT_WEIGHT
        else:
            new_gps = gps
        
        return new_gps, theta
    
    
    def get_sensor_gps_delta(self, sensor_val, sensor_coords):
        if not sensor_val:
            # sensor might be on a line that isn't black or anywhere else, nothing to do
            # if sensor is on a line marked in the map, then we need to actually take it into account!
            return self.get_missing_sensor_gps_delta(sensor_coords)

        # get sensor pos in relation to nearest lines' X and Y
        closest_vertical_line_x = round(sensor_coords.x / CM_PER_CELL)*CM_PER_CELL
        closest_horizontal_line_y = round(sensor_coords.y / CM_PER_CELL)*CM_PER_CELL
        sensor_dist_to_vertical_line = sensor_coords.x - closest_vertical_line_x
        sensor_dist_to_horizontal_line = sensor_coords.y - closest_horizontal_line_y
        # for example, [0.5, -0.1] means sensor is 0.5cm to the right of a v-line, and 0.1cm above an h-line

        # either sensor x or y must be a multiple of 12.5+-1.25, but not off by too far
        if HALF_LINE_WIDTH < min(abs(sensor_dist_to_vertical_line), abs(sensor_dist_to_horizontal_line)) < 3 * HALF_LINE_WIDTH:
            if abs(sensor_dist_to_vertical_line) < abs(sensor_dist_to_horizontal_line):
                # x is closer, so adjust x
                if sensor_dist_to_vertical_line > HALF_LINE_WIDTH:
                    # if we believe sensor is to the right of line, we should move our estimate to the left
                    return [-sensor_dist_to_vertical_line+HALF_LINE_WIDTH, 0]
                else:
                    # if we believe sensor is to the left of line, we should move our estimate to the right
                    return [-sensor_dist_to_vertical_line-HALF_LINE_WIDTH, 0]
            else:
                # y is closer, so adjust y
                if sensor_dist_to_horizontal_line > HALF_LINE_WIDTH:
                    # if we believe sensor is below line, we should move our estimate above
                    return [0, -sensor_dist_to_horizontal_line+HALF_LINE_WIDTH]
                else:
                    # if we believe sensor is above line, we should move our estimate below
                    return [0, -sensor_dist_to_horizontal_line-HALF_LINE_WIDTH]
    
        return None
    
    
    def get_missing_sensor_gps_delta(self, sensor_coords):
        if not self.should_sensor_see_black(sensor_coords):
            return None
        
        # get sensor pos in relation to nearest lines' X and Y
        closest_vertical_line_x = round(sensor_coords.x / CM_PER_CELL)*CM_PER_CELL
        closest_horizontal_line_y = round(sensor_coords.y / CM_PER_CELL)*CM_PER_CELL
        sensor_dist_to_vertical_line = sensor_coords.x - closest_vertical_line_x
        sensor_dist_to_horizontal_line = sensor_coords.y - closest_horizontal_line_y
        # for example, [0.5, -0.1] means sensor is 0.5cm to the right of a v-line, and 0.1cm above an h-line

        # sensor x or y must both not be a multiple of 12.5+-1.25
        if abs(sensor_dist_to_vertical_line) < HALF_LINE_WIDTH and abs(sensor_dist_to_horizontal_line) < HALF_LINE_WIDTH:
            if abs(sensor_dist_to_vertical_line) < abs(sensor_dist_to_horizontal_line):
                # x is closer, so adjust x
                if sensor_dist_to_vertical_line > 0:
                    #  |  .  |  X
                    #  |  0  2  5
                    return [-sensor_dist_to_vertical_line+HALF_LINE_WIDTH, 0]
                else:
                    #  X  |  .  |
                    # -5 -2  0
                    return [-sensor_dist_to_vertical_line-HALF_LINE_WIDTH, 0]
            else:
                # y is closer, so adjust y
                if sensor_dist_to_horizontal_line > 0:
                    # ---
                    #  .   0
                    # ---  2
                    #  X   5
                    return [0, -sensor_dist_to_horizontal_line+HALF_LINE_WIDTH]
                else:
                    #  X   -5
                    # ---  -2
                    #  .    0
                    # --- 
                    return [0, -sensor_dist_to_horizontal_line-HALF_LINE_WIDTH]
    
        return None
    
    
    def get_sensor_theta_delta(self, sensor_val, sensor_coords, gps):
        if not sensor_val:
            return self.get_missing_sensor_theta_delta(sensor_coords, gps)
        
        # circle is given by (x - h)2 + (y - k)2 = r2
        # (x-gps.x)*(x-gps.x) + (y-gps.y)*(y-gps.y) = SQUARED_GROUND_SENSOR_DIST
        
        # get sensor pos in relation to nearest lines' X and Y
        closest_vertical_line_x = round(sensor_coords.x / CM_PER_CELL)*CM_PER_CELL
        closest_horizontal_line_y = round(sensor_coords.y / CM_PER_CELL)*CM_PER_CELL
        sensor_dist_to_vertical_line = abs(sensor_coords.x-closest_vertical_line_x)
        sensor_dist_to_horizontal_line = abs(sensor_coords.y-closest_horizontal_line_y)
        # for example, [0.5, 0.1] means sensor is 0.5cm to the left/right of a v-line, and 0.1cm above/below an h-line
        
        # sensor coords must be a multiple of 12.5+-1.25, but not off by too far
        if sensor_dist_to_horizontal_line < sensor_dist_to_vertical_line:
            # x is closer, so adjust x
            if HALF_LINE_WIDTH < sensor_dist_to_horizontal_line < 3 * HALF_LINE_WIDTH:
                if sensor_coords.y > closest_horizontal_line_y:
                    # sensor below line
                    horizontal_line_edge_y = closest_horizontal_line_y + HALF_LINE_WIDTH
                else:
                    # sensor above line
                    horizontal_line_edge_y = closest_horizontal_line_y - HALF_LINE_WIDTH
                
                # solve circle equation, find intersecting X
                y_minus_gps_y = horizontal_line_edge_y-gps.y
                r2_minus_delta_y = SQUARED_GROUND_SENSOR_DIST - y_minus_gps_y*y_minus_gps_y
                if r2_minus_delta_y < 0:
                    # no contact with line? something is massively wrong
                    return None
                intersection_x_pos = math.sqrt(SQUARED_GROUND_SENSOR_DIST - y_minus_gps_y*y_minus_gps_y) + gps.x
                intersection_x_neg = -math.sqrt(SQUARED_GROUND_SENSOR_DIST - y_minus_gps_y*y_minus_gps_y) + gps.x
                
                # choose closest intersecting point
                if Utils.dist2(Location(intersection_x_pos, horizontal_line_edge_y), sensor_coords) < Utils.dist2(Location(intersection_x_neg, horizontal_line_edge_y), sensor_coords):
                    intersection_x = intersection_x_pos
                else:
                    intersection_x = intersection_x_neg

                angle = Utils.get_radian_between_points(gps, Location(intersection_x, horizontal_line_edge_y)) - Utils.get_radian_between_points(gps, sensor_coords)
                #print(f"robot at [{gps.x:.2f},{gps.y:.2f}], sensor at [{sensor_coords.x:.2f},{sensor_coords.y:.2f}], should rotate {Utils.to_degree(angle):.2f} to get to intersection [{intersection_x:.2f},{horizontal_line_edge_y:.2f}]")
                return angle
        else:
            # y is closer, so adjust y
            if HALF_LINE_WIDTH < sensor_dist_to_vertical_line < 3 * HALF_LINE_WIDTH:
                if sensor_coords.x > closest_vertical_line_x:
                    # sensor to right of line
                    vertical_line_edge_x = closest_vertical_line_x + HALF_LINE_WIDTH
                else:
                    # senser to left of line
                    vertical_line_edge_x = closest_vertical_line_x - HALF_LINE_WIDTH
                
                # solve circle equation, find intersecting X
                x_minus_gps_x = vertical_line_edge_x-gps.x
                r2_minus_delta_x = SQUARED_GROUND_SENSOR_DIST - x_minus_gps_x*x_minus_gps_x
                if r2_minus_delta_x < 0:
                    # no contact with line? something is massively wrong
                    return None
                intersection_y_pos = math.sqrt(SQUARED_GROUND_SENSOR_DIST - x_minus_gps_x*x_minus_gps_x) + gps.y
                intersection_y_neg = -math.sqrt(SQUARED_GROUND_SENSOR_DIST - x_minus_gps_x*x_minus_gps_x) + gps.y
                
                # choose closest intersecting point
                if Utils.dist2(Location(vertical_line_edge_x, intersection_y_pos), sensor_coords) < Utils.dist2(Location(vertical_line_edge_x, intersection_y_neg), sensor_coords):
                    intersection_y = intersection_y_pos
                else:
                    intersection_y = intersection_y_neg

                angle = Utils.get_radian_between_points(gps, Location(vertical_line_edge_x, intersection_y)) - Utils.get_radian_between_points(gps, sensor_coords)
                #print(f"robot at [{gps.x:.2f},{gps.y:.2f}], sensor at [{sensor_coords.x:.2f},{sensor_coords.y:.2f}], should rotate {Utils.to_degree(angle):.2f}º to get to intersection [{vertical_line_edge_x:.2f},{intersection_y:.2f}]")
                return angle
        return None
            
    def get_missing_sensor_theta_delta(self, sensor_coords, gps):
        if not self.should_sensor_see_black(sensor_coords):
            return None

        # circle is given by (x - h)2 + (y - k)2 = r2
        # (x-gps.x)*(x-gps.x) + (y-gps.y)*(y-gps.y) = SQUARED_GROUND_SENSOR_DIST
        
        # get sensor pos in relation to nearest lines' X and Y
        closest_vertical_line_x = round(sensor_coords.x / CM_PER_CELL)*CM_PER_CELL
        closest_horizontal_line_y = round(sensor_coords.y / CM_PER_CELL)*CM_PER_CELL
        sensor_dist_to_vertical_line = abs(sensor_coords.x-closest_vertical_line_x)
        sensor_dist_to_horizontal_line = abs(sensor_coords.y-closest_horizontal_line_y)
        # for example, [0.5, 0.1] means sensor is 0.5cm to the left/right of a v-line, and 0.1cm above/below an h-line
        
        # sensor coords must be a multiple of 12.5+-1.25, but not off by too far
        if sensor_dist_to_horizontal_line < sensor_dist_to_vertical_line:
            # x is closer, so adjust x
            if sensor_coords.y > closest_horizontal_line_y:
                # sensor below line
                horizontal_line_edge_y = closest_horizontal_line_y + HALF_LINE_WIDTH
            else:
                # sensor above line
                horizontal_line_edge_y = closest_horizontal_line_y - HALF_LINE_WIDTH
            
            # solve circle equation, find intersecting X
            y_minus_gps_y = horizontal_line_edge_y-gps.y
            r2_minus_delta_y = SQUARED_GROUND_SENSOR_DIST - y_minus_gps_y*y_minus_gps_y
            if r2_minus_delta_y < 0:
                # no contact with line? something is massively wrong
                return None
            intersection_x_pos = math.sqrt(SQUARED_GROUND_SENSOR_DIST - y_minus_gps_y*y_minus_gps_y) + gps.x
            intersection_x_neg = -math.sqrt(SQUARED_GROUND_SENSOR_DIST - y_minus_gps_y*y_minus_gps_y) + gps.x
            
            # choose closest intersecting point
            if Utils.dist2(Location(intersection_x_pos, horizontal_line_edge_y), sensor_coords) < Utils.dist2(Location(intersection_x_neg, horizontal_line_edge_y), sensor_coords):
                intersection_x = intersection_x_pos
            else:
                intersection_x = intersection_x_neg

            angle = Utils.get_radian_between_points(gps, Location(intersection_x, horizontal_line_edge_y)) - Utils.get_radian_between_points(gps, sensor_coords)
            #print(f"robot at [{gps.x:.2f},{gps.y:.2f}], sensor at [{sensor_coords.x:.2f},{sensor_coords.y:.2f}], should rotate {Utils.to_degree(angle):.2f}º to get to intersection [{intersection_x:.2f},{horizontal_line_edge_y:.2f}]")
            return angle
        else:
            # y is closer, so adjust y
            if sensor_coords.x > closest_vertical_line_x:
                # sensor to right of line
                vertical_line_edge_x = closest_vertical_line_x + HALF_LINE_WIDTH
            else:
                # senser to left of line
                vertical_line_edge_x = closest_vertical_line_x - HALF_LINE_WIDTH
            
            # solve circle equation, find intersecting X
            x_minus_gps_x = vertical_line_edge_x-gps.x
            r2_minus_delta_x = SQUARED_GROUND_SENSOR_DIST - x_minus_gps_x*x_minus_gps_x
            if r2_minus_delta_x < 0:
                # no contact with line? something is massively wrong
                return None
            intersection_y_pos = math.sqrt(SQUARED_GROUND_SENSOR_DIST - x_minus_gps_x*x_minus_gps_x) + gps.y
            intersection_y_neg = -math.sqrt(SQUARED_GROUND_SENSOR_DIST - x_minus_gps_x*x_minus_gps_x) + gps.y
            
            # choose closest intersecting point
            if Utils.dist2(Location(vertical_line_edge_x, intersection_y_pos), sensor_coords) < Utils.dist2(Location(vertical_line_edge_x, intersection_y_neg), sensor_coords):
                intersection_y = intersection_y_pos
            else:
                intersection_y = intersection_y_neg

            angle = Utils.get_radian_between_points(gps, Location(vertical_line_edge_x, intersection_y)) - Utils.get_radian_between_points(gps, sensor_coords)
            #print(f"robot at [{gps.x:.2f},{gps.y:.2f}], sensor at [{sensor_coords.x:.2f},{sensor_coords.y:.2f}], should rotate {Utils.to_degree(angle):.2f}º to get to intersection [{vertical_line_edge_x:.2f},{intersection_y:.2f}]")
            return angle
    
        return None
        
    def should_sensor_see_black(self, sensor_coords):
        cell_coords = Maze.get_cell_coords_from_gps_coords(sensor_coords)                # [5.1, 2.3]
        cell_index = Maze.get_cell_indexes_from_cell_coords(cell_coords)                 # [ 5 ,  2 ]
        sensor_cell_coords_rel_to_center = cell_coords - cell_index                      # [0.1, 0.3]
        sensor_cell = self.map.maze[cell_index.x][cell_index.y]

        # find closest line in cell to sensor, check if we think it should be painted black
        # use sensor_cell_coords_rel_to_center to find if sensor is on top of some line and check if that line is black!
        if abs(sensor_cell_coords_rel_to_center.x) < HALF_LINE_WIDTH_CELL and sensor_cell_coords_rel_to_center.y > 0:
            return sensor_cell.down_line
        elif abs(sensor_cell_coords_rel_to_center.x) < HALF_LINE_WIDTH_CELL and sensor_cell_coords_rel_to_center.y < 0:
            return sensor_cell.up_line
        elif sensor_cell_coords_rel_to_center.x > 0 and abs(sensor_cell_coords_rel_to_center.y) < HALF_LINE_WIDTH_CELL:
            return sensor_cell.right_line
        elif sensor_cell_coords_rel_to_center.x < 0 and abs(sensor_cell_coords_rel_to_center.y) < HALF_LINE_WIDTH_CELL:
            return sensor_cell.left_line
        return False
