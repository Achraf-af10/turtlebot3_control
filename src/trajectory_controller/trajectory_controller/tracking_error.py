
"""Body-frame tracking errors for the midpoint of the wheel axle."""

import math

from trajectory_controller.trajectory import require_finite


def wrap_angle(angle):
    """Ramene un angle dans l'intervalle [-pi, pi]."""
    require_finite(angle)
    return math.atan2(math.sin(angle), math.cos(angle))


def compute_tracking_error(x, y, theta, x_ref, y_ref, theta_ref):
    """Calculate tracking errors in the robot frame."""
    require_finite(x, y, theta, x_ref, y_ref, theta_ref)
    dx = x_ref - x
    dy = y_ref - y

    e_x = dx * math.cos(theta) + dy * math.sin(theta)
    e_y = -dx * math.sin(theta) + dy * math.cos(theta)
    e_theta = wrap_angle(theta_ref - theta)

    return e_x, e_y, e_theta
