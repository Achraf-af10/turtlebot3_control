"""Regression tests for geometry, limits and fail-closed state transitions."""

from dataclasses import replace
import math
import random

import pytest

from trajectory_controller.control import (
    Gains, Limits, limit_command, lyapunov_metrics, nominal_command,
)
from trajectory_controller.safety import Sample, Supervisor, quaternion_yaw
from trajectory_controller.tracking_error import compute_tracking_error, wrap_angle
from trajectory_controller.trajectory import (
    CircularTrajectory, FigureEightTrajectory, RampedTrajectory,
)


@pytest.mark.parametrize('factory', [CircularTrajectory, FigureEightTrajectory])
def test_reference_anchoring_and_kinematic_consistency(factory):
    """Check position derivatives and heading derivative over a full cycle."""
    path = factory(x0=1.2, y0=-0.7, theta0=1.1)
    first = path.reference(0.0)
    assert first[:3] == pytest.approx((1.2, -0.7, 1.1))
    h = 1e-4
    for t in range(150):
        x, y, theta, v, w = path.reference(t)
        before, after = path.reference(t - h), path.reference(t + h)
        assert (after[0] - before[0]) / (2 * h) == pytest.approx(v * math.cos(theta))
        assert (after[1] - before[1]) / (2 * h) == pytest.approx(v * math.sin(theta))
        assert wrap_angle(after[2] - before[2]) / (2 * h) == pytest.approx(w, abs=1e-9)
        assert v > 0.0
        assert math.isfinite(x + y + theta + w)


@pytest.mark.parametrize('factory', [CircularTrajectory, FigureEightTrajectory])
def test_reference_ramp_chain_rule(factory):
    """Ramping changes path time and both velocities consistently."""
    path = RampedTrajectory(factory())
    assert path.reference(0.0) == pytest.approx((0.0, 0.0, 0.0, 0.0, 0.0))
    h = 1e-4
    for t in (0.01, 0.5, 2.0, 20.0, 60.0):
        x, y, theta, v, w = path.reference(t)
        before, after = path.reference(t - h), path.reference(t + h)
        assert (after[0] - before[0]) / (2 * h) == pytest.approx(
            v * math.cos(theta), abs=1e-9)
        assert (after[1] - before[1]) / (2 * h) == pytest.approx(
            v * math.sin(theta), abs=1e-9)
        assert wrap_angle(after[2] - before[2]) / (2 * h) == pytest.approx(w, abs=1e-9)
    with pytest.raises(ValueError):
        path.reference(-1.0)


def test_errors_are_in_robot_frame():
    """A world +y displacement is longitudinal when the robot faces +y."""
    assert compute_tracking_error(1, 2, math.pi / 2, 1, 3, math.pi / 2) == (
        pytest.approx(1.0), pytest.approx(0.0), pytest.approx(0.0))
    assert compute_tracking_error(0, 0, math.pi - 0.01, 0, 0, -math.pi + 0.01)[2] == (
        pytest.approx(0.02))


def test_lyapunov_identity_uses_actual_velocity():
    """Compare the residual identity against differentiation of L directly."""
    rng = random.Random(482)
    gains = Gains()
    for _ in range(200):
        e1, e2, e3, vr, wr, v, w = [rng.uniform(-2, 2) for _ in range(7)]
        errors = (e1, e2, e3)
        derivative = (e1 * (-v + w * e2 + vr * math.cos(e3))
                      + e2 * (-w * e1 + vr * math.sin(e3))
                      + math.sin(e3) / gains.k2 * (wr - w))
        value, ideal, measured = lyapunov_metrics(errors, vr, wr, v, w, gains)
        assert value >= 0.0
        assert ideal <= 0.0
        assert measured == pytest.approx(derivative, abs=1e-12)
        command = nominal_command(errors, vr, wr, gains)
        assert lyapunov_metrics(errors, vr, wr, *command, gains)[2] == ideal


def test_saturation_can_increase_lyapunov_function():
    """Preserve the counterexample: saturation invalidates the nominal proof."""
    gains = Gains(1.0, 30.0, 1.5)
    errors = (0.0, 0.32, 0.5)
    v, w = nominal_command(errors, 0.1, 0.1, gains)
    assert w > 1.0
    value, ideal, actual = lyapunov_metrics(errors, 0.1, 0.1, v, 1.0, gains)
    assert value < 2.0 / gains.k2
    assert ideal < 0.0 < actual


def test_limiter_bounds_body_wheels_and_slew_with_reverse():
    """Random command sequences must respect simultaneous actuator constraints."""
    rng, limits = random.Random(10), Limits()
    previous = (0.0, 0.0)
    reverse_observed = False
    for _ in range(2000):
        dt = rng.uniform(0.01, 0.08)
        command, speed_flag, slew_flag = limit_command(
            rng.uniform(-1, 1), rng.uniform(-10, 10), previous, dt, limits)
        v, w = command
        assert abs(v) <= limits.linear + 1e-12
        assert abs(w) <= limits.angular + 1e-12
        assert abs(v - limits.separation * w / 2) <= limits.wheel_linear + 1e-12
        assert abs(v + limits.separation * w / 2) <= limits.wheel_linear + 1e-12
        assert abs(v - previous[0]) <= limits.acceleration * dt + 1e-12
        assert abs(w - previous[1]) <= limits.angular_acceleration * dt + 1e-12
        assert isinstance(speed_flag, bool) and isinstance(slew_flag, bool)
        reverse_observed |= v < 0
        previous = command
    assert reverse_observed


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_values_never_become_commands(bad):
    """Guard the old NaN-to-maximum-command failure mode."""
    calls = [lambda: nominal_command((bad, 0, 0), 0.1, 0.0, Gains()),
             lambda: limit_command(bad, 0, (0, 0), 0.05, Limits()),
             lambda: compute_tracking_error(bad, 0, 0, 0, 0, 0),
             lambda: CircularTrajectory(radius=bad),
             lambda: FigureEightTrajectory(a=bad),
             lambda: Gains(k1=bad), lambda: Limits(linear=bad),
             lambda: quaternion_yaw(0, 0, bad, 1)]
    for call in calls:
        with pytest.raises(ValueError):
            call()


@pytest.mark.parametrize('kwargs', [{'radius': 0}, {'linear_speed': -1}])
def test_invalid_circle_parameters(kwargs):
    """Reject degenerate references before starting."""
    with pytest.raises(ValueError):
        CircularTrajectory(**kwargs)


def observation(stamp=100.0, received=10.0, **kwargs):
    """Construct a stationary observation without ROS dependencies."""
    return replace(Sample(stamp, received, 0, 0, 0, 0, 0,
                          'odom', 'base_footprint'), **kwargs)


def armed_supervisor():
    """Return a supervisor after an explicit start."""
    supervisor = Supervisor()
    supervisor.accept(observation(), 100.0)
    supervisor.start(10.0, 100.0)
    return supervisor


def test_start_requires_fresh_stationary_measurement():
    """Missing, stale and moving observations must not arm the robot."""
    supervisor = Supervisor()
    with pytest.raises(ValueError):
        supervisor.start(10.0, 100.0)
    supervisor.accept(observation(v=0.1), 100.0)
    with pytest.raises(ValueError):
        supervisor.start(10.0, 100.0)
    supervisor.accept(observation(stamp=100.1, received=10.1), 100.1)
    with pytest.raises(ValueError):
        supervisor.start(11.0, 101.0)
    assert not supervisor.active


@pytest.mark.parametrize('steady,epoch', [(10.5, 100.0), (10.1, 100.5), (10.1, 99.0)])
def test_watchdog_uses_both_clocks_and_latches_stop(steady, epoch):
    """Frozen ROS time does not defeat the reception watchdog."""
    supervisor = armed_supervisor()
    assert not supervisor.check(steady, epoch)
    supervisor.accept(observation(stamp=101.0, received=11.0), 101.0)
    assert not supervisor.check(11.0, 101.0)
    supervisor.start(11.0, 101.0)
    assert supervisor.active


@pytest.mark.parametrize('sample,now', [
    (observation(), 100.0),                         # duplicate
    (observation(stamp=99.9), 100.0),               # backwards after start
    (observation(stamp=100.1, x=float('nan')), 100.1),
    (observation(stamp=100.1, frame='map'), 100.1),
    (observation(stamp=100.1, child_frame='wrong'), 100.1),
    (observation(stamp=100.1), 100.5),              # stale at reception
    (observation(stamp=101.0), 100.1),              # future
])
def test_invalid_measurements_latch_fault(sample, now):
    """Invalid data cannot refresh the watchdog or keep motion enabled."""
    supervisor = armed_supervisor()
    with pytest.raises(ValueError):
        supervisor.accept(sample, now)
    assert not supervisor.active
    assert supervisor.sample is None
    with pytest.raises(ValueError):
        supervisor.start(10.1, now)


def test_repeated_stamp_remains_invalid_and_time_reset_can_recover():
    """Require a strictly newer sample and an explicit start after a reset."""
    supervisor = armed_supervisor()
    for _ in range(3):
        with pytest.raises(ValueError):
            supervisor.accept(observation(), 100.0)
    with pytest.raises(ValueError):
        supervisor.accept(observation(stamp=50.0, received=11.0), 50.0)
    supervisor.accept(observation(stamp=50.1, received=11.1), 50.1)
    assert not supervisor.active
    supervisor.start(11.1, 50.1)
    assert supervisor.active


def test_quaternion_validation_and_normalization():
    """Normalize rounding errors but reject an undefined orientation."""
    theta = 1.2
    assert quaternion_yaw(0, 0, math.sin(theta / 2) * 1.001,
                          math.cos(theta / 2) * 1.001) == pytest.approx(theta)
    for q in [(0, 0, 0, 0), (0, 0, 0, 2)]:
        with pytest.raises(ValueError):
            quaternion_yaw(*q)
