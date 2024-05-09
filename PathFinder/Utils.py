import math
import numpy as np

MEDIAN_SIZE = 5


def normalize_radian_angle(angle):
    # [-pi, pi]
    while angle < -math.pi:
        angle += 2.0 * math.pi

    while angle > math.pi:
        angle -= 2.0 * math.pi

    return angle


def normalize_degree_angle(angle):
    # [-180, 180]
    while angle < -180:
        angle += 360

    while angle > 180:
        angle -= 360

    return angle


def to_degree(radian):
    return normalize_degree_angle(radian*180/math.pi)


def to_radian(degree):
    return normalize_radian_angle(degree*math.pi/180)


# ----------------cyberEEVEE------------------

# returns dot product of "v" and "w"
def dist2(v, w):
    return (v[0] - w[0]) ** 2 + (v[1] - w[1]) ** 2


# returns euclidian distance between "v" and "w"
def dist(v, w):
    return np.sqrt(dist2(v, w))


# returns manhattan distance between "v" and "w"
def dist_manhattan(v, w):
    return np.abs(v[0] - w[0]) + np.abs(v[1] - w[1])


# returns the square of the distance of point "p" to line segment between "v" and "w"
def dist_to_segment_squared(p, v, w):
    l2 = dist2(v, w)
    if l2 == 0:
        return dist2(p, v)

    t = ((p[0] - v[0]) * (w[0] - v[0]) + (p[1] - v[1]) * (w[1] - v[1])) / l2
    t = np.max([0, np.min([1, t])])

    return dist2(p, [v[0] + t * (w[0] - v[0]), v[1] + t * (w[1] - v[1])])


# returns distance of point "p" to line segment between "v" and "w"
def dist_to_line_segment(p, v, w):
    return np.sqrt(dist_to_segment_squared(p, v, w))


# returns true iff the line from (a,b)->(c,d) intersects with (p,q)->(r,s)
def intersects(AB, CD, PQ, RS):
    det = (CD[0] - AB[0]) * (RS[1] - PQ[1]) - (RS[0] - PQ[0]) * (CD[1] - AB[1])
    if det == 0:
        return False

    lambda_ = ((RS[1] - PQ[1]) * (RS[0] - AB[0]) +
               (PQ[0] - RS[0]) * (RS[1] - AB[1])) / det
    gamma = ((AB[1] - CD[1]) * (RS[0] - AB[0]) +
             (CD[0] - AB[0]) * (RS[1] - AB[1])) / det

    return 0 < lambda_ < 1 and 0 < gamma < 1


# returns angle a (in degrees) within range [-180, 180]
def normalize_angle(a):
    while a < -180:
        a += 360
    while a > 180:
        a -= 360
    return a

# returns angle between line [c,e] and x-axis in degrees
def get_angle_between_points(c, e):
    # dy = e[1] - c[1]
    # dx = e[0] - c[0]
    theta = np.arctan2(e[1] - c[1], e[0] - c[0])  # np.arctan(dy / dx)
    return to_degree(theta)  # rads to degs

# returns angle between line [c,e] and x-axis in radians
def get_radian_between_points(c, e):
    # dy = e[1] - c[1]
    # dx = e[0] - c[0]
    theta = np.arctan2(e[1] - c[1], e[0] - c[0])  # np.arctan(dy / dx)
    return normalize_radian_angle(theta)

def radian_angle_between_vectors(vector1, vector2):
    x1, y1 = vector1
    x2, y2 = vector2
    inner_product = x1*x2 + y1*y2
    len1 = math.hypot(x1, y1)
    len2 = math.hypot(x2, y2)
    return math.acos(inner_product/(len1*len2))

def radian_angle_between_line_segments(a1, a2, b1, b2):
    vector1 = [a2[0]-a1[0], a2[1]-a1[1]]
    vector2 = [b2[0]-b1[0], b2[1]-b1[1]]
    return radian_angle_between_vectors(vector1, vector2)


# aux func?
def perp(a):
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

# returns point of intersection between vectors [a1,a2] and [b1,b2]
def seg_intersect(a1, a2, b1, b2):
    da = a2-a1
    db = b2-b1
    dp = a1-b1
    dap = perp(da)
    denom = np.dot(dap, db)
    num = np.dot(dap, dp)
    return (num / denom.astype(float))*db + b1

def get_rigid_compass(compass):
    # convert compass from [0, 2pi] to [0, 4]
    compass = compass * 2 / math.pi
    # round to neared int and convert back to radian
    compass = round(compass) * math.pi / 2
    return compass

GROUND_SENSOR_DISTANCE = 7.45  # cm from ground sensor to robot center; should be 7.4, 7.42, 7.52
FAR_SENSOR_ANGLE = to_radian(30)
NEAR_SENSOR_ANGLE = to_radian(10)

def far_left_sensor_gps(gps_x: float, gps_y: float, compass: float):
    # 40º left
    return [gps_x + GROUND_SENSOR_DISTANCE * math.cos(compass - FAR_SENSOR_ANGLE),
            gps_y + GROUND_SENSOR_DISTANCE * math.sin(compass - FAR_SENSOR_ANGLE)]

def left_sensor_gps(gps_x: float, gps_y: float, compass: float):
    # 10º left
    return [gps_x + GROUND_SENSOR_DISTANCE * math.cos(compass - NEAR_SENSOR_ANGLE),
            gps_y + GROUND_SENSOR_DISTANCE * math.sin(compass - NEAR_SENSOR_ANGLE)]

def front_sensor_gps(gps_x: float, gps_y: float, compass: float):
    # 0º front
    return [gps_x + GROUND_SENSOR_DISTANCE * math.cos(compass),
            gps_y + GROUND_SENSOR_DISTANCE * math.sin(compass)]

def right_sensor_gps(gps_x: float, gps_y: float, compass: float):
    # 10º right
    return [gps_x + GROUND_SENSOR_DISTANCE * math.cos(compass + NEAR_SENSOR_ANGLE),
            gps_y + GROUND_SENSOR_DISTANCE * math.sin(compass + NEAR_SENSOR_ANGLE)]

def far_right_sensor_gps(gps_x: float, gps_y: float, compass: float):
    # 40º right
    return [gps_x + GROUND_SENSOR_DISTANCE * math.cos(compass + FAR_SENSOR_ANGLE),
            gps_y + GROUND_SENSOR_DISTANCE * math.sin(compass + FAR_SENSOR_ANGLE)]

