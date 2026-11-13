
import math


def wrap_angle(angle):
    """Ramene un angle dans l'intervalle [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def compute_tracking_error(x, y, theta, x_ref, y_ref, theta_ref):
    """Calcule les erreurs de suivi dans le repere du robot."""

    dx = x_ref - x
    dy = y_ref - y

    e_x = dx * math.cos(theta) + dy * math.sin(theta)
    e_y = -dx * math.sin(theta) + dy * math.cos(theta)
    e_theta = wrap_angle(theta_ref - theta)

    return e_x, e_y, e_theta

