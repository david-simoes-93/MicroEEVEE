import math

MEDIAN_SIZE = 5

def normalize_radian_angle(angle):
    while angle < -math.pi:
        angle += 2.0 * math.pi

    while angle > math.pi:
        angle -= 2.0 * math.pi

    return angle


def normalize_degree_angle(angle):
    while angle < -180:
        angle += 360

    while angle > 180:
        angle -= 360

    return angle


def to_degree(radian):
    return radian*180/math.pi


def to_radian(degree):
    return degree*math.pi/180
