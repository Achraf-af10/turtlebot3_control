"""ROS transport tests in an isolated namespace, without robot or simulator."""

import time
import uuid

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist, TwistStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from trajectory_controller.command_guard import CommandGuard  # noqa: E402
from trajectory_controller.tracking_node import TrackingNode  # noqa: E402

pytestmark = pytest.mark.ros


class Rig:
    """Exercise real ROS publishers, services and timers on synthetic inputs."""

    def __init__(self, stamped):
        namespace = '/tracking_test_' + uuid.uuid4().hex
        self.peer = Node('test_peer', namespace=namespace)
        self.tracker = TrackingNode(namespace=namespace)
        self.guard = CommandGuard(namespace=namespace, parameter_overrides=[
            Parameter('stamped_cmd_vel', value=stamped)])
        self.executor = SingleThreadedExecutor()
        for node in (self.peer, self.tracker, self.guard):
            self.executor.add_node(node)
        self.commands = []
        output_type = TwistStamped if stamped else Twist
        self.sub = self.peer.create_subscription(
            output_type, 'cmd_vel',
            lambda msg: self.commands.append(msg.twist if stamped else msg), 10)
        self.odom = self.peer.create_publisher(Odometry, 'odom', 1)
        self.clients = {
            name: self.peer.create_client(Trigger, service)
            for name, service in [
                ('start', 'tracking_node/start'), ('stop', 'tracking_node/stop'),
                ('enable', 'command_guard/enable'), ('disable', 'command_guard/disable')]
        }
        self.bad_odometry = False
        self.send_odometry = True
        self.last_publication = 0.0

    def pump(self, duration):
        """Publish odometry at 30 Hz while dispatching real callbacks."""
        until = time.monotonic() + duration
        while time.monotonic() < until:
            now = time.monotonic()
            if self.send_odometry and now - self.last_publication >= 1 / 30:
                msg = Odometry()
                msg.header.stamp = self.peer.get_clock().now().to_msg()
                msg.header.frame_id, msg.child_frame_id = 'odom', 'base_footprint'
                msg.pose.pose.orientation.w = 1.0
                if self.bad_odometry:
                    msg.pose.pose.position.x = float('nan')
                self.odom.publish(msg)
                self.last_publication = now
            self.executor.spin_once(timeout_sec=0.005)

    def call(self, name):
        """Invoke a service and fail if it does not respond promptly."""
        client = self.clients[name]
        assert client.wait_for_service(timeout_sec=2.0)
        future = client.call_async(Trigger.Request())
        until = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < until:
            self.pump(0.01)
        assert future.done()
        return future.result()

    def close(self):
        """Destroy only the isolated test nodes."""
        self.executor.shutdown()
        for node in (self.tracker, self.guard, self.peer):
            node.destroy_node()


@pytest.fixture(params=[False, True])
def rig(request):
    """Test both supported base message types with real ROS middleware."""
    rclpy.init()
    instance = Rig(request.param)
    try:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            instance.pump(0.05)
            if (instance.commands and instance.tracker.supervisor.sample is not None
                    and instance.guard.supervisor.sample is not None):
                break
        yield instance
    finally:
        instance.close()
        rclpy.shutdown()


def stopped(command):
    """Recognize a zero planar command."""
    return command.linear.x == 0.0 and command.angular.z == 0.0


def test_explicit_start_stop_and_invalid_odometry(rig):
    """No startup motion, working command transport, and latched NaN stop."""
    assert rig.commands and all(stopped(command) for command in rig.commands)
    assert rig.call('enable').success
    assert rig.call('start').success
    rig.pump(0.3)
    assert any(command.linear.x > 0 for command in rig.commands)
    assert rig.call('stop').success
    rig.pump(0.1)
    assert stopped(rig.commands[-1])
    assert rig.call('start').success
    rig.bad_odometry = True
    rig.pump(0.2)
    assert stopped(rig.commands[-1])
    assert not rig.tracker.supervisor.active
    rig.bad_odometry = False
    rig.pump(0.2)
    assert not rig.tracker.supervisor.active
    assert stopped(rig.commands[-1])


def test_guard_expires_commands_when_tracker_stalls(rig):
    """A live guard must stop even if every tracker timer is disabled."""
    assert rig.call('enable').success
    assert rig.call('start').success
    rig.pump(0.25)
    assert any(command.linear.x > 0 for command in rig.commands)
    for timer in (rig.tracker.control_timer, rig.tracker.watchdog,
                  rig.tracker.telemetry_timer):
        timer.cancel()
    rig.pump(0.45)
    assert not rig.guard.supervisor.active
    assert stopped(rig.commands[-1])
    # A new nonzero command alone must not re-enable the relay.
    rig.tracker.publish_command((0.03, 0.0))
    rig.pump(0.05)
    assert stopped(rig.commands[-1])
    assert not rig.call('enable').success


def test_measurement_loss_stops_tracker_and_requires_start(rig):
    """Fresh command heartbeats cannot mask missing odometry."""
    assert rig.call('enable').success
    assert rig.call('start').success
    rig.pump(0.2)
    rig.send_odometry = False
    rig.pump(0.45)
    assert stopped(rig.commands[-1])
    assert not rig.tracker.supervisor.active
    rig.send_odometry = True
    rig.pump(0.15)
    assert not rig.tracker.supervisor.active


def test_guard_rejects_nonfinite_command_and_stays_disabled(rig):
    """Fresh tracker traffic must not undo a fault in the output relay."""
    assert rig.call('enable').success
    assert rig.call('start').success
    rig.pump(0.2)
    # Stop normal publication so the malformed sample cannot be overwritten
    # in the one-element input queue before delivery.
    rig.tracker.control_timer.cancel()
    rig.tracker.watchdog.cancel()
    msg = TwistStamped()
    msg.header.stamp = rig.peer.get_clock().now().to_msg()
    msg.header.frame_id = 'base_footprint'
    msg.twist.linear.x = float('nan')
    rig.tracker.cmd_pub.publish(msg)
    rig.pump(0.1)
    assert not rig.guard.supervisor.active
    assert stopped(rig.commands[-1])
    rig.tracker.publish_command((0.02, 0.0))
    rig.pump(0.1)
    assert not rig.guard.supervisor.active
    assert stopped(rig.commands[-1])
