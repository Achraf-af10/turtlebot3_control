"""Separate local process: expire controller commands before the base driver."""

import signal

from geometry_msgs.msg import Twist, TwistStamped
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger

from trajectory_controller.control import Limits
from trajectory_controller.safety import Sample, Supervisor
from trajectory_controller.trajectory import require_finite


class CommandGuard(Node):
    """Relay fresh commands only after explicit enable from a zero command.

    This protects against a failed tracker process, not against failure of this
    process, the OS, the base driver or the serial link. It is not an E-stop.
    """

    def __init__(self, **kwargs):
        """Create a disabled relay with independent command deadlines."""
        super().__init__('command_guard', **kwargs)
        defaults = dict(stamped_cmd_vel=False, timeout=0.3, future_tolerance=0.05,
                        base_frame='base_footprint', max_linear_speed=0.10,
                        max_angular_speed=0.8, max_wheel_linear_speed=0.15,
                        wheel_separation=0.160)
        p = {name: self.declare_parameter(
            name, value, ParameterDescriptor(read_only=True)).value
             for name, value in defaults.items()}
        if self.get_parameter('use_sim_time').value:
            raise ValueError('command_guard exige use_sim_time=false.')
        self.stamped, self.frame = p['stamped_cmd_vel'], p['base_frame']
        self.limits = Limits(linear=p['max_linear_speed'], angular=p['max_angular_speed'],
                             wheel_linear=p['max_wheel_linear_speed'],
                             separation=p['wheel_separation'])
        self.supervisor = Supervisor(p['timeout'], p['future_tolerance'],
                                     self.frame, self.frame)
        self.supervisor.reason = 'Attente de /command_guard/enable.'
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.previous_steady, self.previous_ros = self.times()
        self.shutdown_requested = False
        output_type = TwistStamped if self.stamped else Twist
        self.publisher = self.create_publisher(output_type, 'cmd_vel', 1)
        self.subscription = self.create_subscription(
            TwistStamped, 'tracking_cmd_vel', self.command_callback, 1)
        self.enable_service = self.create_service(Trigger, '~/enable', self.enable_callback)
        self.disable_service = self.create_service(Trigger, '~/disable', self.disable_callback)
        self.timer = self.create_timer(0.02, self.tick, clock=self.steady_clock)

    def times(self):
        """Read reception and measurement clocks separately."""
        return (self.steady_clock.now().nanoseconds * 1e-9,
                self.get_clock().now().nanoseconds * 1e-9)

    def publish(self, v=0.0, w=0.0):
        """Adapt the base driver's configured Twist or TwistStamped interface."""
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        if self.stamped:
            stamped = TwistStamped()
            stamped.header.stamp = self.get_clock().now().to_msg()
            stamped.header.frame_id, stamped.twist = self.frame, msg
            self.publisher.publish(stamped)
        else:
            self.publisher.publish(msg)

    def halt(self, reason):
        """Latch a fault and send zero immediately."""
        changed = self.supervisor.active or self.supervisor.reason != reason
        self.supervisor.invalidate(reason)
        self.publish()
        if changed:
            self.get_logger().warning(reason)

    def command_callback(self, msg):
        """Reject stale, malformed or out-of-envelope commands."""
        try:
            linear, angular = msg.twist.linear, msg.twist.angular
            require_finite(linear.x, linear.y, linear.z, angular.x, angular.y, angular.z)
            if any(value != 0.0 for value in (linear.y, linear.z, angular.x, angular.y)):
                raise ValueError('Commande non planaire.')
            v, w, limit = linear.x, angular.z, self.limits
            if (abs(v) > limit.linear + 1e-9 or abs(w) > limit.angular + 1e-9
                    or abs(v - limit.separation * w / 2) > limit.wheel_linear + 1e-9
                    or abs(v + limit.separation * w / 2) > limit.wheel_linear + 1e-9):
                raise ValueError('Commande hors limites.')
            now, epoch = self.times()
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.supervisor.accept(Sample(stamp, now, 0, 0, 0, v, w,
                                          msg.header.frame_id, self.frame), epoch)
        except (ValueError, OverflowError) as exc:
            # Preserve the rejected stamp recorded by accept when applicable.
            last_stamp = self.supervisor.last_stamp
            self.halt(f'Commande rejetee : {exc}')
            self.supervisor.last_stamp = last_stamp

    def enable_callback(self, request, response):
        """Require a fresh, exactly zero command before enabling the relay."""
        del request
        if self.supervisor.active:
            response.success, response.message = True, 'Relais deja actif.'
            return response
        try:
            sample = self.supervisor.sample
            if sample is None or sample.v != 0.0 or sample.w != 0.0:
                raise ValueError('Arreter tracking_node avant d activer le relais.')
            if self.publisher.get_subscription_count() == 0:
                raise ValueError('Aucun recepteur cmd_vel compatible : verifier le driver/type.')
            if self.get_parameter('use_sim_time').value:
                raise ValueError('use_sim_time doit rester false.')
            self.supervisor.start(*self.times())
            response.success, response.message = True, 'Relais active, attente de start.'
        except ValueError as exc:
            response.success, response.message = False, str(exc)
        return response

    def disable_callback(self, request, response):
        """Stop the relay until the next explicit enable."""
        del request
        self.halt('Relais desactive par operateur.')
        response.success, response.message = True, self.supervisor.reason
        return response

    def tick(self):
        """Expire commands even if the tracker stops running or publishing."""
        now, epoch = self.times()
        dt, epoch_dt = now - self.previous_steady, epoch - self.previous_ros
        self.previous_steady, self.previous_ros = now, epoch
        if (self.get_parameter('use_sim_time').value
                or abs(dt - epoch_dt) > 0.1 or dt > self.supervisor.timeout):
            self.halt('Horloge ou boucle du relais interrompue.')
            return
        was_active = self.supervisor.active
        if self.supervisor.check(now, epoch):
            self.publish(self.supervisor.sample.v, self.supervisor.sample.w)
        else:
            self.publish()
            if was_active:
                self.get_logger().warning('Commande expiree : enable explicite requis.')


def main(args=None):
    """Attempt a final stop before closing the ROS context."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node, old_handlers = None, {}
    try:
        node = CommandGuard()

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
                node.halt('Arret du relais.')
            node.destroy_node()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        if rclpy.ok():
            rclpy.shutdown()
