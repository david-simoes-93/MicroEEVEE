import numpy as np


def dist2(v, w):
    return (v[0] - w[0]) ** 2 + (v[1] - w[1]) ** 2


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

    lambda_ = ((RS[1] - PQ[1]) * (RS[0] - AB[0]) + (PQ[0] - RS[0]) * (RS[1] - AB[1])) / det
    gamma = ((AB[1] - CD[1]) * (RS[0] - AB[0]) + (CD[0] - AB[0]) * (RS[1] - AB[1])) / det

    return 0 < lambda_ < 1 and 0 < gamma < 1


#
def updateRobotPos(prev_left, prev_right, in_left, in_right, my_x, my_y, my_dir):
    my_dir = my_dir * np.pi / 180
    out_left = in_left * 0.5 + prev_left * 0.5
    out_right = in_right * 0.5 + prev_right * 0.5

    diam = 1.0
    rot = (out_right - out_left) / diam
    lin = (out_right + out_left) / 2.0

    new_my_x = my_x + lin * np.cos(my_dir)
    new_my_y = my_y + lin * np.sin(my_dir)
    new_my_dir = my_dir + rot

    return new_my_x, new_my_y, new_my_dir, out_left, out_right
