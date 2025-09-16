import numpy as np
from math import cos, sin

class DifferentialDriveModel:
    def __init__(self, track_width, dt):
        """
        Differential Drive Model
        :param track_width: (float) distance between the two wheels [m]
        :param dt: (float) discrete time period [s]
        """
        self.track_width = track_width
        self.dt = dt

    def model(self, x, y, yaw, v_L, v_R):
        """
        Updates the state of the differential drive model.
        :param x: (float) vehicle's x-coordinate [m]
        :param y: (float) vehicle's y-coordinate [m]
        :param yaw: (float) vehicle's heading [rad]
        :param v_L: (float) left wheel velocity [m/s]
        :param v_R: (float) right wheel velocity [m/s]
        :return: new_x, new_y, new_yaw
        """
        # Calculate linear and angular velocities from wheel speeds
        v = (v_R + v_L) / 2.0
        omega = (v_R - v_L) / self.track_width

        # Update states
        x_new = x + v * cos(yaw) * self.dt
        y_new = y + v * sin(yaw) * self.dt
        yaw_new = yaw + omega * self.dt
        
        return x_new, y_new, yaw_new