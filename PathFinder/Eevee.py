from typing import List

import Utils
from MapHandler import Maze
from Utils import Location


class Eevee(object):
    def __init__(self) -> None:
        self.gps: Location = Location(0,0)
        self.cell_coords: Location = Maze.get_cell_coords_from_gps_coords(self.gps)
        self.theta: float

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
    