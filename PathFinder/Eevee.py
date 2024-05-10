from typing import List

import Utils
from MapHandler import Maze
from Utils import Location


class Eevee(object):
    def __init__(self, odom) -> None:
        self.gps: Location = Location(0,0)
        self.cell_coords: Location = Maze.get_cell_coords_from_gps_coords(self.gps)
        self.theta: float = 0
        self.my_odom = odom

        self.update_sensors()
    
    def update(self, new_gps, new_theta):
        self.gps = new_gps
        self.theta = new_theta
        self.update_sensors()

    def update_sensors(self):
        self.sensor_positions: List[Location] = [Utils.far_left_sensor_gps(self.gps, self.theta),
                       Utils.left_sensor_gps(self.gps, self.theta),
                       Utils.front_sensor_gps(self.gps, self.theta),
                       Utils.right_sensor_gps(self.gps, self.theta),
                       Utils.far_right_sensor_gps(self.gps, self.theta)]
        
    def odom(self, m2_encoder, m1_encoder, ground_sensors):
        gps, theta = self.my_odom.odometry(m2_encoder, m1_encoder, self.gps, self.theta, self.sensor_positions, ground_sensors)
        self.update(gps, theta)
        
    