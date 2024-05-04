import math
import numpy as np

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


"""
# return a color scheme for each wall
def get_wall_color(wall):
    if wall.confirmed_no_wall:
        return (255, 255, 255)
    if wall.confirmed_wall:
        return (125, 0, 0)
    if wall.wall:
        return (0, 0, 0)
    if wall.no_wall:
        return (225, 225, 225)
    return (128, 128, 128)


# returns new pose based on previous pose and odometry model
def update_robot_pos(prev_left, prev_right, in_left, in_right, my_x, my_y, my_dir, colliding):
    my_dir_rad = my_dir * np.pi / 180  # to radian
    out_left = in_left * 0.5 + prev_left * 0.5
    out_right = in_right * 0.5 + prev_right * 0.5

    diam = 1.0
    rot = (out_right - out_left) / diam
    lin = (out_right + out_left) / 2.0

    if not colliding:
        new_my_x = my_x + lin * np.cos(my_dir_rad)
        new_my_y = my_y + lin * np.sin(my_dir_rad)
    else:
        new_my_x = my_x
        new_my_y = my_y
    new_my_dir = my_dir + rot

    #if compass is not None:
    #    new_my_dir = 0.5 * new_my_dir + 0.5 * (compass * np.pi / 180)

    return new_my_x, new_my_y, new_my_dir, out_left, out_right


# returns new dir based on previous dir from 4 cycles ago and odometry model
def update_delayed_compass(prev_left, prev_right, in_left, in_right, my_dir):
    diam = 1.0

    for i in range(4):
        out_left = in_left[i] * 0.5 + prev_left[i] * 0.5
        out_right = in_right[i] * 0.5 + prev_right[i] * 0.5
        rot = (out_right - out_left) / diam
        my_dir = my_dir + rot

    return my_dir


# returns new pose based on previous pose and odometry model
def update_robot_pos_time_delay(prev_left, prev_right, in_lefts, in_rights, my_x, my_y, my_dir, collidings):
    for i in range(4):
        my_x, my_y, my_dir, prev_left, prev_right = update_robot_pos(
            prev_left, prev_right, in_lefts[i], in_rights[i], my_x, my_y, my_dir, collidings[i])
    return my_x, my_y, my_dir, prev_left, prev_right
"""

# returns angle between line [c,e] and x-axis in degrees


def get_angle_between_points(c, e):
    # dy = e[1] - c[1]
    # dx = e[0] - c[0]
    theta = np.arctan2(e[1] - c[1], e[0] - c[0])  # np.arctan(dy / dx)
    return theta * 180 / np.pi  # rads to degs

# returns angle between line [c,e] and x-axis in degrees


def get_radian_between_points(c, e):
    # dy = e[1] - c[1]
    # dx = e[0] - c[0]
    theta = np.arctan2(e[1] - c[1], e[0] - c[0])  # np.arctan(dy / dx)
    return theta


"""

def filter_buffer(buffer, val):
    if val < 0.4:
        val = 0.4
    val = 1 / val

    buffer.pop(0)
    buffer.append(val)
    return sorted(buffer)[int(len(buffer) / 2)]  # median
    # return np.mean(buffer)                  # average


def stop_speed(prev_left, prev_right):
    stop_speed_val = -0.5 * prev_left - 0.5 * prev_right
    return stop_speed_val, stop_speed_val
"""


def perp(a):
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

# line segment a given by endpoints a1, a2
# line segment b given by endpoints b1, b2
# return


def seg_intersect(a1, a2, b1, b2):
    da = a2-a1
    db = b2-b1
    dp = a1-b1
    dap = perp(da)
    denom = np.dot(dap, db)
    num = np.dot(dap, dp)
    return (num / denom.astype(float))*db + b1
