cmake_minimum_required(VERSION 3.16)
project(go2_waypoint_turn_validation)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
add_executable(check_waypoint_turn check_turn.cpp)
ament_target_dependencies(check_waypoint_turn rclcpp rclcpp_lifecycle geometry_msgs
  nav_msgs tf2_ros tf2_geometry_msgs pluginlib)
if(GO2_NAV2_SOURCE)
  find_package(nav_2d_msgs REQUIRED)
  ament_target_dependencies(check_waypoint_turn nav_2d_msgs)
  # Compile untouched official Humble control, progress, TF and footprint
  # implementations. Only the costmap ROS lifecycle wrapper is adapted.
  target_include_directories(check_waypoint_turn PRIVATE adapter
    "${GO2_NAV2_SOURCE}/nav2_rotation_shim_controller/include"
    "${GO2_NAV2_SOURCE}/nav2_controller/include"
    "${GO2_NAV2_SOURCE}/nav2_core/include"
    "${GO2_NAV2_SOURCE}/nav2_costmap_2d/include"
    "${GO2_NAV2_SOURCE}/nav2_util/include"
    "${GO2_NAV2_SOURCE}/nav2_dwb_controller/nav_2d_utils/include"
    "${GO2_ANGLES_INCLUDE}")
  target_sources(check_waypoint_turn PRIVATE
    "${GO2_NAV2_SOURCE}/nav2_rotation_shim_controller/src/nav2_rotation_shim_controller.cpp"
    "${GO2_NAV2_SOURCE}/nav2_controller/plugins/position_goal_checker.cpp"
    "${GO2_NAV2_SOURCE}/nav2_controller/plugins/simple_goal_checker.cpp"
    "${GO2_NAV2_SOURCE}/nav2_controller/plugins/simple_progress_checker.cpp"
    "${GO2_NAV2_SOURCE}/nav2_controller/plugins/pose_progress_checker.cpp"
    "${GO2_NAV2_SOURCE}/nav2_costmap_2d/src/costmap_2d.cpp"
    "${GO2_NAV2_SOURCE}/nav2_costmap_2d/src/footprint_collision_checker.cpp"
    "${GO2_NAV2_SOURCE}/nav2_util/src/robot_utils.cpp"
    "${GO2_NAV2_SOURCE}/nav2_dwb_controller/nav_2d_utils/src/conversions.cpp")
else()
  find_package(nav2_rotation_shim_controller REQUIRED)
  find_package(nav2_controller REQUIRED)
  find_package(nav2_costmap_2d REQUIRED)
  ament_target_dependencies(check_waypoint_turn nav2_rotation_shim_controller
    nav2_controller nav2_costmap_2d)
endif()
target_compile_features(check_waypoint_turn PRIVATE cxx_std_17)
