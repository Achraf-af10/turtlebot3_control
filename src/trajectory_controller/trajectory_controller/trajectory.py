import math


class CircularTrajectory:
    """Genere une trajectoire circulaire pour TurtleBot3."""

    def __init__(
        self,
        x0=-2.0,
        y0=-0.5,
        theta0=0.0,
        radius=1.0,
        linear_speed=0.2,
    ):
        if radius <= 0.0:
            raise ValueError("Le rayon doit etre strictement positif.")
        if linear_speed <= 0.0:
            raise ValueError("La vitesse doit etre strictement positive.")

        self.radius = radius
        self.linear_speed = linear_speed
        self.angular_speed = linear_speed / radius
        self.theta0 = theta0

        # Centre du cercle pour demarrer a la pose initiale.
        self.center_x = x0 - radius * math.sin(theta0)
        self.center_y = y0 + radius * math.cos(theta0)

    def reference(self, t):
        phi = (
            self.angular_speed * t
            + self.theta0
            - math.pi / 2.0
        )

        x_ref = self.center_x + self.radius * math.cos(phi)
        y_ref = self.center_y + self.radius * math.sin(phi)

        theta_ref = math.atan2(
            math.sin(phi + math.pi / 2.0),
            math.cos(phi + math.pi / 2.0),
        )

        v_ref = self.linear_speed
        omega_ref = self.angular_speed

        return x_ref, y_ref, theta_ref, v_ref, omega_ref
