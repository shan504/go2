cmake_minimum_required(VERSION 3.16)
project(go2_path_alignment_validation)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(angles REQUIRED)
add_executable(check_path_alignment check_alignment.cpp)
ament_target_dependencies(check_path_alignment rclcpp rclcpp_lifecycle geometry_msgs
  nav_msgs tf2_ros tf2_geometry_msgs pluginlib angles)
if(GO2_NAV2_SOURCE)
  find_package(nav_2d_msgs REQUIRED)
  ament_target_dependencies(check_path_alignment nav_2d_msgs)
  target_include_directories(check_path_alignment PRIVATE adapter
    "${CMAKE_CURRENT_LIST_DIR}/../../navigation/go2_navigation/include"
    "${GO2_NAV2_SOURCE}/nav2_rotation_shim_controller/include"
    "${GO2_NAV2_SOURCE}/nav2_controller/include"
    "${GO2_NAV2_SOURCE}/nav2_core/include"
    "${GO2_NAV2_SOURCE}/nav2_costmap_2d/include"
    "${GO2_NAV2_SOURCE}/nav2_util/include"
    "${GO2_NAV2_SOURCE}/nav2_dwb_controller/nav_2d_utils/include"
    "${GO2_ANGLES_INCLUDE}")
  target_sources(check_path_alignment PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../../navigation/go2_navigation/src/path_alignment_controller.cpp"
    "${GO2_NAV2_SOURCE}/nav2_rotation_shim_controller/src/nav2_rotation_shim_controller.cpp"
    "${GO2_NAV2_SOURCE}/nav2_controller/plugins/position_goal_checker.cpp"
    "${GO2_NAV2_SOURCE}/nav2_costmap_2d/src/costmap_2d.cpp"
    "${GO2_NAV2_SOURCE}/nav2_costmap_2d/src/footprint_collision_checker.cpp"
    "${GO2_NAV2_SOURCE}/nav2_util/src/robot_utils.cpp"
    "${GO2_NAV2_SOURCE}/nav2_dwb_controller/nav_2d_utils/src/conversions.cpp")
else()
  find_package(go2_navigation REQUIRED)
  find_package(nav2_rotation_shim_controller REQUIRED)
  find_package(nav2_controller REQUIRED)
  find_package(nav2_costmap_2d REQUIRED)
  ament_target_dependencies(check_path_alignment go2_navigation nav2_rotation_shim_controller
    nav2_controller nav2_costmap_2d)
  target_compile_definitions(check_path_alignment PRIVATE GO2_TEST_INSTALLED_PLUGIN=1)
endif()
target_compile_features(check_path_alignment PRIVATE cxx_std_17)
