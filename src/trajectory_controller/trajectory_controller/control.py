"""Kinematic feedback only: no torque or current interface is assumed."""

from dataclasses import dataclass
import math

from trajectory_controller.trajectory import require_finite


@dataclass(frozen=True)
class Gains:
    """Positive gains for the nominal continuous-time feedback."""

    k1: float = 0.5
    k2: float = 8.0
    k3: float = 1.0

    def __post_init__(self):
        """Validate finite positive configuration values."""
        require_finite(self.k1, self.k2, self.k3)
        if min(self.k1, self.k2, self.k3) <= 0.0:
            raise ValueError('Les gains doivent etre strictement positifs.')


@dataclass(frozen=True)
class Limits:
    """Commissioning limits, not a motor certification or torque bound."""

    linear: float = 0.10
    angular: float = 0.8
    wheel_linear: float = 0.15
    separation: float = 0.160
    acceleration: float = 0.05
    angular_acceleration: float = 0.5

    def __post_init__(self):
        """Validate finite positive configuration values."""
        values = tuple(vars(self).values())
        require_finite(*values)
        if min(values) <= 0.0:
            raise ValueError('Limites strictement positives requises.')


def nominal_command(errors, v_ref, omega_ref, gains):
    """Evaluate the feedback without saturation or reverse suppression."""
    e1, e2, e3 = errors
    require_finite(*errors, v_ref, omega_ref)
    v = v_ref * math.cos(e3) + gains.k1 * e1
    w = omega_ref + gains.k2 * v_ref * e2 + gains.k3 * math.sin(e3)
    require_finite(v, w)
    return v, w


def limit_command(v, w, previous, dt, limits):
    """Bound body/wheel speeds, then slew along a feasible line segment."""
    require_finite(v, w, *previous, dt)
    if dt <= 0.0:
        raise ValueError('Pas de temps strictement positif requis.')
    half = limits.separation / 2.0
    scale = max(1.0, abs(v) / limits.linear, abs(w) / limits.angular,
                abs(v - half * w) / limits.wheel_linear,
                abs(v + half * w) / limits.wheel_linear)
    target = (v / scale, w / scale)
    dv, dw = target[0] - previous[0], target[1] - previous[1]
    slew = max(1.0, abs(dv) / (limits.acceleration * dt),
               abs(dw) / (limits.angular_acceleration * dt))
    result = (previous[0] + dv / slew, previous[1] + dw / slew)
    require_finite(*result)
    return result, scale > 1.0, slew > 1.0


def lyapunov_metrics(errors, v_ref, omega_ref, measured_v, measured_w, gains):
    """Return L and its model derivative using measured body velocities.

    This is a kinematic diagnostic, not a proof for the sampled robot.
    """
    e1, e2, e3 = errors
    require_finite(*errors, v_ref, omega_ref, measured_v, measured_w)
    nominal_v, nominal_w = nominal_command(errors, v_ref, omega_ref, gains)
    value = 0.5 * (e1 * e1 + e2 * e2) + (1.0 - math.cos(e3)) / gains.k2
    ideal = -gains.k1 * e1 * e1 - gains.k3 / gains.k2 * math.sin(e3)**2
    residual = (-e1 * (measured_v - nominal_v)
                - math.sin(e3) / gains.k2 * (measured_w - nominal_w))
    require_finite(value, ideal, residual)
    return value, ideal, ideal + residual
