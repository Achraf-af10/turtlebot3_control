"""ROS 2 hardware velocity tracking for a stock TurtleBot3 Burger."""

import math
import signal

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_srvs.srv import Trigger

from trajectory_controller.control import (
    Gains, Limits, limit_command, lyapunov_metrics, nominal_command,
)
from trajectory_controller.safety import Sample, Supervisor, quaternion_yaw
from trajectory_controller.tracking_error import compute_tracking_error
from trajectory_controller.trajectory import (
    CircularTrajectory, FigureEightTrajectory, RampedTrajectory, require_finite,
)


class TrackingNode(Node):
    """Separate observation, control and telemetry; latch every fault."""

    def __init__(self, **kwargs):
        """Validate parameters and create hardware tracking interfaces."""
        super().__init__('tracking_node', **kwargs)
        defaults = {
            'trajectory': 'circle', 'radius': 0.5, 'eight_a': 0.6,
            'eight_b': 0.3, 'reference_speed': 0.08, 'ramp_time': 2.0,
            'k1': 0.5, 'k2': 8.0, 'k3': 1.0, 'control_rate': 20.0,
            'max_linear_speed': 0.10, 'max_angular_speed': 0.8,
            'max_wheel_linear_speed': 0.15, 'wheel_separation': 0.160,
            'wheel_radius': 0.033, 'max_acceleration': 0.05,
            'max_angular_acceleration': 0.5, 'odom_timeout': 0.3,
            'future_tolerance': 0.05, 'max_control_gap': 0.2,
            'clock_jump_tolerance': 0.1, 'max_position_error': 0.35,
            'max_heading_error': 1.2, 'run_duration': 90.0,
            'odom_frame': 'odom', 'base_frame': 'base_footprint',
            'max_path_poses': 2000,
        }
        self.config = {}
        for name, value in defaults.items():
            descriptor = ParameterDescriptor(read_only=True)
            self.config[name] = self.declare_parameter(
                name, value, descriptor).value
        p = self.config
        for name, value in defaults.items():
            if isinstance(value, (int, float)):
                require_finite(p[name])
                if p[name] <= 0:
                    raise ValueError(f'{name} doit etre strictement positif.')
        if p['trajectory'] not in ('circle', 'eight'):
            raise ValueError('trajectory doit valoir circle ou eight.')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Ce lancement materiel exige use_sim_time=false.')
        if 1.0 / p['control_rate'] >= p['max_control_gap']:
            raise ValueError('max_control_gap doit depasser la periode de commande.')
        self.gains = Gains(p['k1'], p['k2'], p['k3'])
        self.limits = Limits(p['max_linear_speed'], p['max_angular_speed'],
                             p['max_wheel_linear_speed'], p['wheel_separation'],
                             p['max_acceleration'], p['max_angular_acceleration'])
        self.supervisor = Supervisor(p['odom_timeout'], p['future_tolerance'],
                                     p['odom_frame'], p['base_frame'])
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.previous_command = (0.0, 0.0)
        self.previous_tick = self.steady_now()
        self.previous_ros = self.ros_now()
        self.start_time = None
        self.trajectory = None
        self.reference = None
        self.metrics = {}
        self.reference_path = Path()
        self.actual_path = Path()
        self.shutdown_requested = False
        self.cmd_pub = self.create_publisher(TwistStamped, 'tracking_cmd_vel', 1)
        self.ref_path_pub = self.create_publisher(Path, 'reference_path', 1)
        self.actual_path_pub = self.create_publisher(Path, 'actual_path', 1)
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, 'tracking_diagnostics', 1)
        odom_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, odom_qos)
        self.start_service = self.create_service(Trigger, '~/start', self.start_callback)
        self.stop_service = self.create_service(Trigger, '~/stop', self.stop_callback)
        self.control_timer = self.create_timer(
            1.0 / p['control_rate'], self.control_callback, clock=self.steady_clock)
        self.watchdog = self.create_timer(
            0.05, self.watchdog_callback, clock=self.steady_clock)
        self.telemetry_timer = self.create_timer(
            0.2, self.telemetry_callback, clock=self.steady_clock)
        self.get_logger().info(
            'Burger reel, commande en vitesse uniquement. '
            'Arrete jusqu au service /tracking_node/start.')

    def steady_now(self):
        """Read the clock used for periods and reception deadlines."""
        return self.steady_clock.now().nanoseconds * 1e-9

    def ros_now(self):
        """Read the epoch clock used by the hardware odometry publisher."""
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def stamp_to_sec(stamp):
        """Convert a ROS stamp without treating reception as measurement time."""
        return stamp.sec + stamp.nanosec * 1e-9

    def publish_command(self, command):
        """Publish only finite planar velocity commands."""
        require_finite(*command)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.config['base_frame']
        msg.twist.linear.x, msg.twist.angular.z = command
        self.cmd_pub.publish(msg)
        self.previous_command = command

    def halt(self, reason, invalidate=False):
        """Send zero immediately and require a subsequent explicit start."""
        changed = self.supervisor.active or self.supervisor.reason != reason
        if invalidate:
            self.supervisor.invalidate(reason)
        else:
            self.supervisor.stop(reason)
        self.publish_command((0.0, 0.0))
        self.metrics = {}
        if changed:
            self.get_logger().warning(reason)

    def odom_callback(self, msg):
        """Validate and cache odometry; never command from a sensor callback."""
        try:
            pos, q = msg.pose.pose.position, msg.pose.pose.orientation
            vel = msg.twist.twist
            require_finite(pos.x, pos.y, pos.z, vel.linear.x, vel.linear.y,
                           vel.linear.z, vel.angular.x, vel.angular.y, vel.angular.z)
            theta = quaternion_yaw(q.x, q.y, q.z, q.w)
            sample = Sample(self.stamp_to_sec(msg.header.stamp), self.steady_now(),
                            pos.x, pos.y, theta, vel.linear.x, vel.angular.z,
                            msg.header.frame_id, msg.child_frame_id)
            self.supervisor.accept(sample, self.ros_now())
        except (ValueError, OverflowError) as exc:
            last_stamp = self.supervisor.last_stamp
            self.halt(f'Odometre invalide : {exc}', invalidate=True)
            self.supervisor.last_stamp = last_stamp

    def start_callback(self, request, response):
        """Anchor the path at the measured pose, with no hidden offset."""
        del request
        if self.supervisor.active:
            response.success = False
            response.message = 'Deja actif : arreter avant de recommencer.'
            return response
        try:
            if self.get_parameter('use_sim_time').value:
                raise ValueError('use_sim_time doit rester false sur le robot reel.')
            if self.cmd_pub.get_subscription_count() == 0:
                raise ValueError('Aucun recepteur de tracking_cmd_vel : lancer command_guard.')
            now = self.steady_now()
            self.supervisor.start(now, self.ros_now())
            sample, p = self.supervisor.sample, self.config
            initial = dict(x0=sample.x, y0=sample.y, theta0=sample.theta)
            if p['trajectory'] == 'circle':
                base = CircularTrajectory(**initial, radius=p['radius'],
                                          linear_speed=p['reference_speed'])
            else:
                base = FigureEightTrajectory(**initial, a=p['eight_a'],
                                             b=p['eight_b'],
                                             peak_speed=p['reference_speed'])
            self.trajectory = RampedTrajectory(base, p['ramp_time'])
            self.start_time = now
            self.previous_tick, self.previous_ros = now, self.ros_now()
            self.publish_command((0.0, 0.0))
            self.reference = self.trajectory.reference(0.0)
            self.reference_path, self.actual_path = Path(), Path()
            self.metrics = {}
            response.success = True
            response.message = 'Suivi demarre depuis la pose mesuree, avec rampe.'
        except (ValueError, OverflowError) as exc:
            self.halt(str(exc))
            response.success, response.message = False, str(exc)
        return response

    def stop_callback(self, request, response):
        """Handle an operator-requested stop without an acceleration ramp."""
        del request
        self.halt('Arret demande par operateur.')
        response.success, response.message = True, self.supervisor.reason
        return response

    def control_callback(self):
        """Run bounded feedback at a fixed steady-clock rate."""
        try:
            now, ros_now = self.steady_now(), self.ros_now()
            dt, ros_dt = now - self.previous_tick, ros_now - self.previous_ros
            self.previous_tick, self.previous_ros = now, ros_now
            if self.get_parameter('use_sim_time').value:
                self.halt('use_sim_time interdit sur le robot reel.', invalidate=True)
                return
            if abs(ros_dt - dt) > self.config['clock_jump_tolerance']:
                self.halt('Saut de l horloge ROS.', invalidate=True)
                return
            if not self.supervisor.check(now, ros_now):
                self.publish_command((0.0, 0.0))
                self.metrics = {}
                return
            if not 0.0 < dt <= self.config['max_control_gap']:
                self.halt('Retard excessif de la boucle de commande.')
                return
            elapsed = now - self.start_time
            if elapsed >= self.config['run_duration']:
                self.halt('Duree de suivi terminee.')
                return
            sample = self.supervisor.sample
            self.reference = self.trajectory.reference(elapsed)
            xr, yr, tr, vr, wr = self.reference
            errors = compute_tracking_error(sample.x, sample.y, sample.theta, xr, yr, tr)
            if (math.hypot(errors[0], errors[1]) > self.config['max_position_error']
                    or abs(errors[2]) > self.config['max_heading_error']):
                self.halt('Erreur de suivi au dela du domaine experimental configure.')
                return
            raw = nominal_command(errors, vr, wr, self.gains)
            command, saturated, slew_limited = limit_command(
                *raw, self.previous_command, dt, self.limits)
            value, ideal, measured = lyapunov_metrics(
                errors, vr, wr, sample.v, sample.w, self.gains)
            self.publish_command(command)
            half, radius = self.limits.separation / 2.0, self.config['wheel_radius']
            self.metrics = dict(
                elapsed_s=elapsed, control_dt_s=dt, odom_age_s=ros_now - sample.stamp,
                e1_m=errors[0], e2_m=errors[1], e3_rad=errors[2],
                v_ref_mps=vr, w_ref_radps=wr,
                v_raw_mps=raw[0], w_raw_radps=raw[1],
                v_command_mps=command[0], w_command_radps=command[1],
                v_measured_mps=sample.v, w_measured_radps=sample.w,
                wheel_left_command_radps=(command[0] - half * command[1]) / radius,
                wheel_right_command_radps=(command[0] + half * command[1]) / radius,
                speed_limited=saturated, slew_limited=slew_limited,
                L=value, Ldot_ideal=ideal, Ldot_kinematic_measured=measured,
            )
        except (ValueError, OverflowError) as exc:
            self.halt(f'Calcul invalide : {exc}', invalidate=True)

    def watchdog_callback(self):
        """Use steady time even if ROS time freezes; not an embedded watchdog."""
        was_active = self.supervisor.active
        if not self.supervisor.check(self.steady_now(), self.ros_now()):
            self.publish_command((0.0, 0.0))
            if was_active:
                self.metrics = {}
                self.get_logger().warning(self.supervisor.reason)

    def telemetry_callback(self):
        """Publish bounded paths and named diagnostics outside the control callback."""
        stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name, status.hardware_id = 'trajectory_tracking', 'turtlebot3_burger'
        limited = self.metrics.get('speed_limited') or self.metrics.get('slew_limited')
        status.level = (DiagnosticStatus.OK if self.supervisor.active and not limited
                        else DiagnosticStatus.WARN)
        status.message = self.supervisor.reason
        status.values = [KeyValue(key=key, value=str(value))
                         for key, value in self.metrics.items()]
        status.values.append(KeyValue(key='active', value=str(self.supervisor.active)))
        diagnostic = DiagnosticArray()
        diagnostic.header.stamp, diagnostic.status = stamp, [status]
        self.diagnostic_pub.publish(diagnostic)
        if not self.supervisor.active or self.reference is None:
            return
        sample = self.supervisor.sample
        poses = ((self.actual_path, self.actual_path_pub,
                  (sample.x, sample.y, sample.theta), sample.stamp),
                 (self.reference_path, self.ref_path_pub,
                  self.reference[:3], self.previous_ros))
        for path, publisher, (x, y, theta), pose_time in poses:
            pose = PoseStamped()
            pose.header.frame_id = self.config['odom_frame']
            pose.header.stamp = Time(seconds=pose_time).to_msg()
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.z = math.sin(theta / 2.0)
            pose.pose.orientation.w = math.cos(theta / 2.0)
            path.header = pose.header
            path.poses.append(pose)
            path.poses = path.poses[-self.config['max_path_poses']:]
            publisher.publish(path)

    def stop_robot(self):
        """Issue a best-effort stop while the ROS context is still alive."""
        self.halt('Arret du controleur.')


def main(args=None):
    """Keep the ROS context alive long enough to attempt a final zero command."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    old_handlers = {}
    try:
        node = TrackingNode()

        def request_shutdown(signum, frame):
            del signum, frame
            node.shutdown_requested = True

        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, request_shutdown)
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop_robot()
            node.destroy_node()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
