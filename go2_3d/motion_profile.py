"""3D navigation envelope, based on the successful 0.30m/s Go2 hardware test."""
import math

LINEAR_SPEED = 0.30
YAW_SPEED = 0.30
LINEAR_ACCELERATION = 0.40
YAW_ACCELERATION = 0.60
TRAJECTORY_TIME = 1.20
FORWARD_SAMPLES = 2
ARRIVAL_RADIUS = 0.30


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
    # Patrol waypoints mean reaching a location; the user does not require a
    # final heading. SimpleGoalChecker uses shortest-angle yaw errors in [-pi,
    # pi], so pi accepts every heading. Still check current XY on every call.
    goal_checker = nav['controller_server']['ros__parameters']['general_goal_checker']
    goal_checker.update(xy_goal_tolerance=ARRIVAL_RADIUS,
                        yaw_goal_tolerance=math.pi, stateful=False)
    follow['xy_goal_tolerance'] = goal_checker['xy_goal_tolerance']
    # RotateToGoal latches its own near-goal state and forbids translation.
    # Position-only waypoints do not need that terminal rotation stage.
    follow['critics'] = [name for name in follow['critics'] if name != 'RotateToGoal']
    smoother = nav['velocity_smoother']['ros__parameters']
    smoother.update(max_velocity=[LINEAR_SPEED, 0.0, YAW_SPEED],
                    min_velocity=[0.0, 0.0, -YAW_SPEED],
                    max_accel=[LINEAR_ACCELERATION, LINEAR_ACCELERATION, YAW_ACCELERATION],
                    max_decel=[-LINEAR_ACCELERATION, -LINEAR_ACCELERATION, -YAW_ACCELERATION])
