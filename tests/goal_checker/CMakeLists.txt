cmake_minimum_required(VERSION 3.16)
project(go2_arrival_validation)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
add_executable(check_arrival check_arrival.cpp)
ament_target_dependencies(check_arrival rclcpp rclcpp_lifecycle geometry_msgs nav_msgs tf2_geometry_msgs pluginlib)
if(GO2_NAV2_SOURCE)
  # Compile the untouched upstream checker in the lightweight cloud ROS image.
  # Its Costmap2DROS argument is unused, so only that opaque type needs an
  # adapter. ROS parameters, quaternion math and the checker remain real.
  target_sources(check_arrival PRIVATE "${GO2_NAV2_SOURCE}/nav2_controller/plugins/simple_goal_checker.cpp")
  target_include_directories(check_arrival PRIVATE opaque
    "${GO2_NAV2_SOURCE}/nav2_controller/include"
    "${GO2_NAV2_SOURCE}/nav2_core/include"
    "${GO2_NAV2_SOURCE}/nav2_util/include" "${GO2_ANGLES_INCLUDE}")
else()
  # On the robot image, exercise the installed official Nav2 implementation.
  find_package(nav2_controller REQUIRED)
  ament_target_dependencies(check_arrival nav2_controller)
endif()
