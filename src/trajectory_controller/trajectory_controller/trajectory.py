"""Smooth planar references; lengths in metres and time in seconds."""

import math


def require_finite(*values):
    """Reject nonfinite inputs before they can reach a command limiter."""
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Toutes les valeurs doivent etre finies.')


class CircularTrajectory:
    """Counterclockwise circle tangent to the specified initial pose."""

    def __init__(self, x0=0.0, y0=0.0, theta0=0.0,
                 radius=0.5, linear_speed=0.08):
        """Set a circle starting at the specified pose."""
        require_finite(x0, y0, theta0, radius, linear_speed)
        if min(radius, linear_speed) <= 0.0:
            raise ValueError('Rayon et vitesse strictement positifs requis.')
        self.radius = radius
        self.linear_speed = linear_speed
        self.angular_speed = linear_speed / radius
        self.theta0 = theta0
        self.center_x = x0 - radius * math.sin(theta0)
        self.center_y = y0 + radius * math.cos(theta0)

    def reference(self, t):
        """Return x, y, yaw, forward speed and yaw rate."""
        require_finite(t)
        angle = self.theta0 + self.angular_speed * t
        return (self.center_x + self.radius * math.sin(angle),
                self.center_y - self.radius * math.cos(angle),
                math.atan2(math.sin(angle), math.cos(angle)),
                self.linear_speed, self.angular_speed)


class FigureEightTrajectory:
    """Gerono-type eight with nonzero tangent at its crossing."""

    def __init__(self, x0=0.0, y0=0.0, theta0=0.0,
                 a=0.6, b=0.3, peak_speed=0.08):
        """Rotate the analytic eight to the specified initial heading."""
        require_finite(x0, y0, theta0, a, b, peak_speed)
        if min(a, b, peak_speed) <= 0.0:
            raise ValueError('Dimensions et vitesse strictement positives.')
        self.x0, self.y0, self.a, self.b = x0, y0, a, b
        self.frequency = peak_speed / math.hypot(a, 2.0 * b)
        self.rotation = theta0 - math.atan2(2.0 * b, a)

    def reference(self, t):
        """Compute heading and curvature from analytic derivatives."""
        require_finite(t)
        f, a, b = self.frequency, self.a, self.b
        phase = f * t
        x, y = a * math.sin(phase), b * math.sin(2.0 * phase)
        dx, dy = a * f * math.cos(phase), 2.0 * b * f * math.cos(2.0 * phase)
        ddx = -a * f * f * math.sin(phase)
        ddy = -4.0 * b * f * f * math.sin(2.0 * phase)
        c, s = math.cos(self.rotation), math.sin(self.rotation)
        heading = self.rotation + math.atan2(dy, dx)
        speed = math.hypot(dx, dy)
        yaw_rate = (dx * ddy - dy * ddx) / (speed * speed)
        return (self.x0 + c * x - s * y, self.y0 + s * x + c * y,
                math.atan2(math.sin(heading), math.cos(heading)),
                speed, yaw_rate)


class RampedTrajectory:
    """Time reparametrization with zero reference speed at startup."""

    def __init__(self, trajectory, ramp_time=2.0):
        """Set the time constant of the startup ramp."""
        require_finite(ramp_time)
        if ramp_time <= 0.0:
            raise ValueError('Temps de rampe strictement positif requis.')
        self.trajectory, self.ramp_time = trajectory, ramp_time

    def reference(self, t):
        """Apply the chain rule to both reference velocities."""
        require_finite(t)
        if t < 0.0:
            raise ValueError('Temps de trajectoire negatif.')
        rate = -math.expm1(-t / self.ramp_time)
        path_time = t - self.ramp_time * rate
        x, y, theta, v, w = self.trajectory.reference(path_time)
        return x, y, theta, rate * v, rate * w
