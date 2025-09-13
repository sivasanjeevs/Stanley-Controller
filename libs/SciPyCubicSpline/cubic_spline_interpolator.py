import numpy as np
from scipy.interpolate import CubicSpline


def generate_cubic_spline(x_points, y_points, ds=0.05):

    x_points = np.asarray(x_points, dtype=float)
    y_points = np.asarray(y_points, dtype=float)

    s = np.zeros_like(x_points)
    s[1:] = np.cumsum(np.hypot(np.diff(x_points), np.diff(y_points)))

    cs_x = CubicSpline(s, x_points)
    cs_y = CubicSpline(s, y_points)

    s_new = np.arange(0.0, s[-1], ds)

    px = cs_x(s_new)
    py = cs_y(s_new)

    dx = cs_x(s_new, 1)
    dy = cs_y(s_new, 1)
    pyaw = np.arctan2(dy, dx)

    return px, py, pyaw, s_new


