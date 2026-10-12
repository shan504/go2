"""3D navigation envelope, based on the successful 0.30m/s Go2 hardware test."""
import math

LINEAR_SPEED = 0.30
YAW_SPEED = 0.30
LINEAR_ACCELERATION = 0.40
YAW_ACCELERATION = 0.60
TRAJECTORY_TIME = 1.20
FORWARD_SAMPLES = 2
ARRIVAL_RADIUS = 0.30
PATH_HEADING_THRESHOLD = 0.35
PATH_HEADING_RELEASE = 0.20
PROGRESS_ANGLE = 0.20


def configure_navigation(nav):
    controller = nav['controller_server']['ros__parameters']
    follow = controller['FollowPath']
    # DWB's short-horizon scoring can stall when the next waypoint is behind
    # the body. The official shim aligns with the NEW PATH before handing
    # control back to the same DWB tracker and real footprint collision check.
    follow.update(plugin='nav2_rotation_shim_controller::RotationShimController',
                  primary_controller='dwb_core::DWBLocalPlanner',
                  angular_dist_threshold=PATH_HEADING_THRESHOLD,
                  angular_disengage_threshold=PATH_HEADING_RELEASE,
                  forward_sampling_distance=0.25,
                  rotate_to_heading_angular_vel=YAW_SPEED,
                  max_angular_accel=YAW_ACCELERATION,
                  simulate_ahead_time=1.0, rotate_to_goal_heading=False,
                  closed_loop=False)
    # A 180-degree turn at 0.30rad/s takes over 10s. Recognize actual heading
    # progress instead of mistaking an in-place turn for a positional stall.
    # A genuinely stationary robot still fails after the same 10s deadline.
    controller['progress_checker'].update(
        plugin='nav2_controller::PoseProgressChecker',
        required_movement_radius=0.10, required_movement_angle=PROGRESS_ANGLE,
        movement_time_allowance=10.0)
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
