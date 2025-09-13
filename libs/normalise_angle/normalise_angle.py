from math import pi


def normalise_angle(angle_radians):

    while angle_radians > pi:
        angle_radians -= 2 * pi
    while angle_radians < -pi:
        angle_radians += 2 * pi
    return angle_radians


