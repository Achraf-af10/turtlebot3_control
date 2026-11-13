from trajectory import CircularTrajectory

trajectory = CircularTrajectory()

for t in [0.0, 5.0, 10.0, 15.0]:
    x, y, theta, v, omega = trajectory.reference(t)

    print(
        f"t={t:5.1f} s | "
        f"x_ref={x:7.3f} m | "
        f"y_ref={y:7.3f} m | "
        f"theta_ref={theta:6.3f} rad | "
        f"v_ref={v:.2f} m/s | "
        f"omega_ref={omega:.2f} rad/s"
    )
