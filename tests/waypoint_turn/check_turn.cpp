#include <cmath>
#include <iostream>
#include <stdexcept>
#include "rcl/time.h"
#include "nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp"
#include "nav2_controller/plugins/pose_progress_checker.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

static void require(bool value, const char * message)
{
  if (!value) {throw std::runtime_error(message);}
}

// A deterministic handoff observer, not a replacement for DWB in production.
class ForwardObserver : public nav2_core::Controller {
public:
  double speed;
  explicit ForwardObserver(double value) : speed(value) {}
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, std::string,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void activate() override {}
  void deactivate() override {}
  void cleanup() override {}
  void setPlan(const nav_msgs::msg::Path &) override {}
  void setSpeedLimit(const double &, const bool &) override {}
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *) override {
    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    command.twist.linear.x = speed;
    return command;
  }
};

class TurnFixture : public nav2_rotation_shim_controller::RotationShimController {
public:
  void initialize(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map,
    double threshold, double release, double sample, double yaw, double accel,
    double ahead, bool closed_loop, double forward) {
    node_ = node; clock_ = node->get_clock(); tf_ = tf; costmap_ros_ = map;
    plugin_name_ = "FollowPath";
    angular_dist_threshold_ = threshold; angular_disengage_threshold_ = release;
    forward_sampling_distance_ = sample; rotate_to_heading_angular_vel_ = yaw;
    max_angular_accel_ = accel; simulate_ahead_time_ = ahead; control_duration_ = 0.1;
    closed_loop_ = closed_loop; rotate_to_goal_heading_ = false;
    primary_controller_ = std::make_unique<ForwardObserver>(forward);
    collision_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<
      nav2_costmap_2d::Costmap2D *>>(map->getCostmap());
    position_goal_checker_ = std::make_unique<nav2_controller::PositionGoalChecker>();
    activate();
  }
  auto rotate(double error, const geometry_msgs::msg::PoseStamped & pose) {
    geometry_msgs::msg::Twist still;
    return computeRotateToHeadingCommand(error, pose, still);
  }
};

static geometry_msgs::msg::PoseStamped pose(double yaw)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = "map";
  result.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
  return result;
}

static void clockAt(const rclcpp::Clock::SharedPtr & clock, double seconds)
{
  require(rcl_set_ros_time_override(clock->get_clock_handle(), int64_t(seconds*1e9)) == RCL_RET_OK,
    "Cannot advance fixture ROS clock");
}

int main(int argc, char ** argv)
{
  if (argc != 12) {return 2;}
  const double threshold=std::stod(argv[1]), release=std::stod(argv[2]), sample=std::stod(argv[3]);
  const double yaw_speed=std::stod(argv[4]), accel=std::stod(argv[5]), ahead=std::stod(argv[6]);
  const bool closed_loop=std::string(argv[7]) == "true";
  const double forward=std::stod(argv[8]), radius=std::stod(argv[9]);
  const double angle=std::stod(argv[10]), allowance=std::stod(argv[11]);
  rclcpp::init(0, nullptr);
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("waypoint_turn_fixture");
  auto clock=node->get_clock();
  require(rcl_enable_ros_time_override(clock->get_clock_handle()) == RCL_RET_OK, "No test clock");
  clockAt(clock, 1);
  for (const auto & name : {"previous", "patrol"}) {
    node->declare_parameter(std::string(name)+".required_movement_radius", radius);
    node->declare_parameter(std::string(name)+".movement_time_allowance", allowance);
  }
  node->declare_parameter("patrol.required_movement_angle", angle);
  nav2_controller::SimpleProgressChecker previous;
  nav2_controller::PoseProgressChecker patrol;
  previous.initialize(node, "previous"); patrol.initialize(node, "patrol");
  auto p=pose(0);
  require(previous.check(p) && patrol.check(p), "Baseline refused");
  bool old_failed=false;
  for (int i=1; i<=130; ++i) {
    clockAt(clock, 1+i*0.1); p=pose(i*0.1*0.25);
    old_failed |= !previous.check(p);
    require(patrol.check(p), "In-place turning falsely treated as a stall");
  }
  require(old_failed, "Old XY checker did not reproduce the 10s turning timeout");
  clockAt(clock, 14+allowance+0.1);
  require(!patrol.check(p), "A truly stalled robot was allowed indefinitely");
  patrol.reset(); clockAt(clock, 40); p=pose(0); require(patrol.check(p), "Reset failed");
  p=pose(0.02); clockAt(clock, 40+allowance+0.1);
  require(!patrol.check(p), "Small heading jitter incorrectly counted as progress");
  patrol.reset(); clockAt(clock, 60); p=pose(3.12); require(patrol.check(p), "Reset failed");
  p=pose(-3.12); clockAt(clock, 60+allowance+0.1);
  require(!patrol.check(p), "Yaw wraparound incorrectly counted as a large turn");
  std::cout << "PASS official progress check: old XY-only timeout reproduced; real turns survive 10s, stalls/jitter/wraparound still fail\n";

  auto tf=std::make_shared<tf2_ros::Buffer>(clock);
  auto listener=std::make_shared<tf2_ros::TransformListener>(*tf, node, true);
  auto options=rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("plugins", std::vector<std::string>()),
    rclcpp::Parameter("track_unknown_space", true),
    rclcpp::Parameter("robot_base_frame", std::string("base_footprint"))});
  auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
  map->on_configure(rclcpp_lifecycle::State());
  auto grid=map->getCostmap();
  grid->resizeMap(160, 160, 0.05, -4, -4);
  std::fill(grid->getCharMap(), grid->getCharMap()+160*160, nav2_costmap_2d::FREE_SPACE);
  std::vector<geometry_msgs::msg::Point> footprint;
  for (auto xy : {std::pair<double,double>{-.41,-.23}, {-.41,.23}, {.41,.23}, {.41,-.23}}) {
    geometry_msgs::msg::Point point; point.x=xy.first; point.y=xy.second; footprint.push_back(point);
  }
  map->setRobotFootprint(footprint);
  TurnFixture shim;
  shim.initialize(node, tf, map, threshold, release, sample, yaw_speed, accel, ahead, closed_loop, forward);
  for (double target : {2.2, -2.2, 3.13, -3.13}) {
    double heading=0.0;
    nav_msgs::msg::Path path; path.header.frame_id="map";
    for (int i=0; i<=20; ++i) {
      auto point=pose(0); point.pose.position.x=i*.05*std::cos(target);
      point.pose.position.y=i*.05*std::sin(target); path.poses.push_back(point);
    }
    shim.deactivate(); shim.activate(); shim.setPlan(path);
    bool handed_off=false; int turns=0;
    for (int step=0; step<200; ++step) {
      clockAt(clock, 100+step*.1); p=pose(heading);
      geometry_msgs::msg::TransformStamped transform;
      transform.header.frame_id="map"; transform.child_frame_id="base_footprint";
      transform.header.stamp=clock->now(); transform.transform.rotation=p.pose.orientation;
      tf->setTransform(transform, "turn_fixture", true);
      geometry_msgs::msg::Twist measured_still; // Noisy/zero yaw odometry must not trap the commanded ramp.
      auto command=shim.computeVelocityCommands(p, measured_still, nullptr).twist;
      require(std::abs(command.angular.z)<=yaw_speed+1e-9, "Turn command exceeded yaw limit");
      if (command.linear.x > 0) {
        require(std::abs(angles::shortest_angular_distance(heading,target))<=release+1e-9,
          "Forward tracking handed off before aligning with new path");
        require(std::abs(command.linear.x-forward)<1e-9, "Walking speed lost at handoff");
        handed_off=true; break;
      }
      require(command.linear.x==0 && command.linear.y==0, "Translation during heading alignment");
      require(command.angular.z*target>0, "Turn sign points away from new path");
      heading += command.angular.z*.1; ++turns;
      if (step%10==0) {shim.setPlan(path);} // Match the real 1Hz replanning.
    }
    require(handed_off && turns>0, "New waypoint never transitioned from turning to walking");
  }
  std::cout << "PASS official rotation shim: left/right 126deg and 179deg paths, 1Hz replans, zero measured yaw; turn then handoff to 0.30m/s tracker\n";
  std::fill(grid->getCharMap(), grid->getCharMap()+160*160, nav2_costmap_2d::LETHAL_OBSTACLE);
  bool blocked=false;
  try {shim.rotate(2.2, pose(0));} catch (const std::runtime_error &) {blocked=true;}
  require(blocked, "Lethal costmap allowed an unchecked rotation");
  std::fill(grid->getCharMap(), grid->getCharMap()+160*160, nav2_costmap_2d::NO_INFORMATION);
  blocked=false;
  try {shim.rotate(2.2, pose(0));} catch (const std::runtime_error &) {blocked=true;}
  require(blocked, "Unknown tracked costmap allowed an unchecked rotation");
  shim.deactivate(); shim.cleanup(); rclcpp::shutdown();
  std::cout << "PASS official swept footprint: lethal and tracked unknown space reject rotation\n";
}
