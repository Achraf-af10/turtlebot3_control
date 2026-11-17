"""Read-only odometry inspection for commissioning the hardware tracker."""

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from trajectory_controller.safety import quaternion_yaw
from trajectory_controller.trajectory import require_finite


class OdometryReader(Node):
    """Log validated planar pose and body velocities without commanding motion."""

    def __init__(self):
        """Subscribe using a QoS compatible with best-effort odometry."""
        super().__init__('odometry_reader')
        self.subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos_profile_sensor_data)

    def odom_callback(self, msg):
        """Check numbers and quaternion before displaying an observation."""
        pos, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        try:
            require_finite(pos.x, pos.y, v, w)
            theta = quaternion_yaw(q.x, q.y, q.z, q.w)
        except ValueError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=1.0)
            return
        self.get_logger().info(
            f'frame={msg.header.frame_id}, child={msg.child_frame_id} | '
            f'x={pos.x:.3f}, y={pos.y:.3f} m | theta={theta:.3f} rad | '
            f'v={v:.3f} m/s, omega={w:.3f} rad/s', throttle_duration_sec=0.5)


def main(args=None):
    """Run the read-only inspection node."""
    rclpy.init(args=args)
    node = OdometryReader()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
