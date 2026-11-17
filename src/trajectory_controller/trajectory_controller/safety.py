"""Input validation and latched execution state, independent of ROS."""

from dataclasses import dataclass
import math

from trajectory_controller.trajectory import require_finite


def quaternion_yaw(x, y, z, w):
    """Reject invalid/non-unit quaternions; normalize small rounding errors."""
    require_finite(x, y, z, w)
    norm = math.hypot(x, y, z, w)
    if abs(norm - 1.0) > 0.01:
        raise ValueError('Quaternion non unitaire ou nul.')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


@dataclass(frozen=True)
class Sample:
    """Planar pose and body velocity, with epoch stamp and steady receipt."""

    stamp: float
    received: float
    x: float
    y: float
    theta: float
    v: float
    w: float
    frame: str
    child_frame: str


class Supervisor:
    """Require fresh observations and an explicit start after every stop."""

    def __init__(self, timeout=0.3, future_tolerance=0.05,
                 frame='odom', child_frame='base_footprint'):
        """Configure deadlines and the expected odometry frames."""
        require_finite(timeout, future_tolerance)
        if timeout <= 0.0 or future_tolerance < 0.0:
            raise ValueError('Delais de surveillance invalides.')
        if not frame or not child_frame:
            raise ValueError('Reperes obligatoires.')
        self.timeout, self.future_tolerance = timeout, future_tolerance
        self.frame, self.child_frame = frame, child_frame
        self.sample = None
        self.active = False
        self.reason = 'En attente de /tracking_node/start.'
        self.last_stamp = None

    def stop(self, reason):
        """Latch a stop; fresh odometry alone must never restart motion."""
        self.active = False
        self.reason = reason

    def invalidate(self, reason):
        """Require a new observation before a subsequent explicit start."""
        self.stop(reason)
        self.sample = None
        self.last_stamp = None

    def accept(self, sample, ros_now):
        """Validate measurement time, frames and numbers before caching."""
        try:
            require_finite(ros_now, sample.stamp, sample.received,
                           sample.x, sample.y, sample.theta, sample.v, sample.w)
            if sample.frame != self.frame or sample.child_frame != self.child_frame:
                raise ValueError('Reperes odometriques inattendus.')
            age = ros_now - sample.stamp
            if sample.stamp <= 0.0 or not -self.future_tolerance <= age <= self.timeout:
                raise ValueError('Horodatage odometrique perime ou futur.')
            if self.last_stamp is not None and sample.stamp <= self.last_stamp:
                raise ValueError('Horodatage odometrique duplique ou retrograde.')
        except ValueError as exc:
            self.invalidate(str(exc))
            # A repeated invalid stamp must not become a fresh sample on retry.
            if math.isfinite(sample.stamp):
                self.last_stamp = sample.stamp
            raise
        self.sample = sample
        self.last_stamp = sample.stamp

    def fresh(self, steady_now, ros_now):
        """Check both message age and time since last valid reception."""
        require_finite(steady_now, ros_now)
        if self.sample is None:
            return False
        age = ros_now - self.sample.stamp
        receipt_age = steady_now - self.sample.received
        return (0.0 <= receipt_age <= self.timeout
                and -self.future_tolerance <= age <= self.timeout)

    def start(self, steady_now, ros_now):
        """Arm only when stationary and supported by a fresh observation."""
        if not self.fresh(steady_now, ros_now):
            raise ValueError('Demarrage refuse : odometrie absente ou perimee.')
        if abs(self.sample.v) > 0.01 or abs(self.sample.w) > 0.05:
            raise ValueError('Demarrage refuse : le robot doit etre immobile.')
        self.active = True
        self.reason = 'Suivi actif.'

    def check(self, steady_now, ros_now):
        """Latch a fault if either clock indicates stale odometry."""
        if self.active and not self.fresh(steady_now, ros_now):
            self.invalidate('Arret : odometrie absente ou perimee.')
        return self.active
