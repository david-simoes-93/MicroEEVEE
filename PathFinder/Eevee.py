from Utils import Location

class Eevee(object):
    gps: Location = Location(0,0)
    cell_coords: Location = Location(0,0)
    theta: float
    
    sensor_positions: List(Location) = []
    sensor_vals: List(bool) = []
    