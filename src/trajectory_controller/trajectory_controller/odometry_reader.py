
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdometryReader(Node):
    def __init__(self):
        super().__init__('odometry_reader')

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info('Lecteur odometrie demarre.')

    def odom_callback(self, msg):
        # Position du robot
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Orientation : quaternion vers angle de lacet
        q = msg.pose.pose.orientation
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(sin_yaw, cos_yaw)

        # Vitesses mesurees
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        self.get_logger().info(
            f'x={x:.3f} m | y={y:.3f} m | '
            f'theta={theta:.3f} rad | '
            f'v={v:.3f} m/s | omega={omega:.3f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdometryReader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
