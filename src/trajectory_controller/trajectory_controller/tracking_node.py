import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped

from trajectory_controller.trajectory import CircularTrajectory
from trajectory_controller.tracking_error import compute_tracking_error


class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')

        # Gains (loi de type Kanayama).
        # Linearisation : s^2 + k3*s + k2*v_ref^2 = 0
        # avec v_ref = 0.1 et k2 = 30 -> wn ~ 0.55 rad/s, zeta ~ 1.4
        self.k1 = 1.0
        self.k2 = 30.0
        self.k3 = 1.5

        # Limites de vitesse.
        self.max_linear_speed = 0.25
        self.max_angular_speed = 1.0

        # Parametres de la trajectoire.
        self.radius = 1.0
        self.ref_speed = 0.1

        # Limitation des Path (evite une croissance sans fin).
        self.max_path_poses = 2000
        self.path_publish_period = 0.2  # secondes (temps simule)
        self.last_path_pub_time = -1.0

        # Securite : arret si l'odometrie disparait.
        self.odom_timeout = 0.5  # secondes
        self.last_odom_wall = None

        self.trajectory = None
        self.start_time = None
        self.last_log_time = 0.0

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ref_path_pub = self.create_publisher(Path, '/reference_path', 10)
        self.actual_path_pub = self.create_publisher(Path, '/actual_path', 10)

        self.reference_path = Path()
        self.actual_path = Path()

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

        # Watchdog sur l'odometrie (temps mur, independant de /clock).
        self.watchdog = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info(
            'Controleur cinematique en attente de la premiere odometrie.'
        )

    @staticmethod
    def stamp_to_sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def odom_callback(self, msg):
        self.last_odom_wall = self.get_clock().now()

        position = msg.pose.pose.position
        q = msg.pose.pose.orientation

        x = position.x
        y = position.y

        # Conversion du quaternion en angle de lacet theta.
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(sin_yaw, cos_yaw)

        # Temps de simulation issu de l'odometrie (et non time.monotonic()).
        t_now = self.stamp_to_sec(msg.header.stamp)

        # Initialiser le cercle sur la pose reelle au premier message.
        if self.trajectory is None:
            self.start_time = t_now
            self.trajectory = CircularTrajectory(
                x0=x - 0.5,
                y0=y + 0.3,
                theta0=theta + 0.5,
                radius=self.radius,
                linear_speed=self.ref_speed,
            )
            self.get_logger().info(
                f'Depart depuis la pose mesuree : '
                f'x={x:.3f}, y={y:.3f}, theta={theta:.3f}'
            )

        elapsed = t_now - self.start_time

        # Reset de simulation : le temps recule, on repart de zero.
        if elapsed < 0.0:
            self.trajectory = None
            self.reference_path = Path()
            self.actual_path = Path()
            self.last_log_time = 0.0
            self.last_path_pub_time = -1.0
            return

        x_ref, y_ref, theta_ref, v_ref, omega_ref = (
            self.trajectory.reference(elapsed)
        )

        e_x, e_y, e_theta = compute_tracking_error(
            x, y, theta,
            x_ref, y_ref, theta_ref,
        )

        # Loi de commande cinematique.
        v_cmd = v_ref * math.cos(e_theta) + self.k1 * e_x
        omega_cmd = (
            omega_ref
            + self.k2 * v_ref * e_y
            + self.k3 * math.sin(e_theta)
        )

        # Saturation des commandes.
        v_cmd = max(0.0, min(self.max_linear_speed, v_cmd))
        omega_cmd = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, omega_cmd),
        )

        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = omega_cmd
        self.cmd_pub.publish(cmd)

        # Publication des chemins a frequence reduite.
        if elapsed - self.last_path_pub_time >= self.path_publish_period:
            self.update_paths(msg.header.stamp, x, y, theta,
                              x_ref, y_ref, theta_ref)
            self.last_path_pub_time = elapsed

        # Journaliser deux fois par seconde environ (temps simule).
        if elapsed - self.last_log_time >= 0.5:
            self.get_logger().info(
                f't={elapsed:.1f}s | '
                f'pose=({x:.2f}, {y:.2f}, {theta:.2f}) | '
                f'ref=({x_ref:.2f}, {y_ref:.2f}, {theta_ref:.2f}) | '
                f'erreurs=({e_x:.3f}, {e_y:.3f}, {e_theta:.3f}) | '
                f'cmd=(v={v_cmd:.3f}, omega={omega_cmd:.3f})'
            )
            self.last_log_time = elapsed

    def update_paths(self, stamp, x, y, theta, x_ref, y_ref, theta_ref):
        actual_pose = PoseStamped()
        actual_pose.header.stamp = stamp
        actual_pose.header.frame_id = 'odom'
        actual_pose.pose.position.x = x
        actual_pose.pose.position.y = y
        actual_pose.pose.orientation.z = math.sin(theta / 2.0)
        actual_pose.pose.orientation.w = math.cos(theta / 2.0)

        ref_pose = PoseStamped()
        ref_pose.header.stamp = stamp
        ref_pose.header.frame_id = 'odom'
        ref_pose.pose.position.x = x_ref
        ref_pose.pose.position.y = y_ref
        ref_pose.pose.orientation.z = math.sin(theta_ref / 2.0)
        ref_pose.pose.orientation.w = math.cos(theta_ref / 2.0)

        self.actual_path.header.stamp = stamp
        self.actual_path.header.frame_id = 'odom'
        self.actual_path.poses.append(actual_pose)

        self.reference_path.header.stamp = stamp
        self.reference_path.header.frame_id = 'odom'
        self.reference_path.poses.append(ref_pose)

        # Limiter la taille des chemins.
        if len(self.actual_path.poses) > self.max_path_poses:
            del self.actual_path.poses[0]
        if len(self.reference_path.poses) > self.max_path_poses:
            del self.reference_path.poses[0]

        self.actual_path_pub.publish(self.actual_path)
        self.ref_path_pub.publish(self.reference_path)

    def watchdog_callback(self):
        # Si l'odometrie s'arrete, on envoie une commande nulle.
        if self.last_odom_wall is None:
            return
        age = (
            self.get_clock().now() - self.last_odom_wall
        ).nanoseconds * 1e-9
        if age > self.odom_timeout:
            self.cmd_pub.publish(Twist())

    def stop_robot(self):
        # Envoyer plusieurs commandes nulles avant l'arret du noeud.
        stop = Twist()
        for _ in range(3):
            self.cmd_pub.publish(stop)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_robot()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()