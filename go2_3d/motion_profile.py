"""3D navigation envelope, based on the successful 0.30m/s Go2 hardware test."""
LINEAR_SPEED = 0.30
YAW_SPEED = 0.30
LINEAR_ACCELERATION = 0.40
YAW_ACCELERATION = 0.60
TRAJECTORY_TIME = 1.20
FORWARD_SAMPLES = 2


def configure_navigation(nav):
    follow = nav['controller_server']['ros__parameters']['FollowPath']
    # StandardTrajectoryGenerator samples across sim_time, rather than one
    # controller tick. From rest, 0.4 * 1.2 reaches the verified 0.30m/s.
    # Two X samples retain 0 for turns and 0.30 for forward trajectories.
    # Do not clamp a planned slow command upward in the actuator bridge.
    follow.update(min_vel_x=0.0, max_vel_x=LINEAR_SPEED,
                  min_vel_y=0.0, max_vel_y=0.0, max_vel_theta=YAW_SPEED,
                  min_speed_xy=0.0, max_speed_xy=LINEAR_SPEED,
                  min_speed_theta=0.0, vx_samples=FORWARD_SAMPLES, vy_samples=1,
                  trajectory_generator_name='dwb_plugins::StandardTrajectoryGenerator',
                  limit_vel_cmd_in_traj=False, sim_time=TRAJECTORY_TIME,
                  acc_lim_x=LINEAR_ACCELERATION, acc_lim_y=LINEAR_ACCELERATION,
                  acc_lim_theta=YAW_ACCELERATION,
                  decel_lim_x=-LINEAR_ACCELERATION, decel_lim_y=-LINEAR_ACCELERATION,
                  decel_lim_theta=-YAW_ACCELERATION)
    # Avoid entering RotateToGoal's translation-stop window before the actual
    # goal checker accepts XY arrival (the SDK preset has mismatched values).
    goal_checker = nav['controller_server']['ros__parameters']['general_goal_checker']
    follow['xy_goal_tolerance'] = goal_checker['xy_goal_tolerance']
    smoother = nav['velocity_smoother']['ros__parameters']
    smoother.update(max_velocity=[LINEAR_SPEED, 0.0, YAW_SPEED],
                    min_velocity=[0.0, 0.0, -YAW_SPEED],
                    max_accel=[LINEAR_ACCELERATION, LINEAR_ACCELERATION, YAW_ACCELERATION],
                    max_decel=[-LINEAR_ACCELERATION, -LINEAR_ACCELERATION, -YAW_ACCELERATION])
