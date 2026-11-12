#include <algorithm>
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
  auto declare_policy = [&](const std::string & name, double distance, double turn, double time) {
    node->declare_parameter(name+".required_movement_radius", distance);
    node->declare_parameter(name+".required_movement_angle", turn);
    node->declare_parameter(name+".movement_time_allowance", time);
  };
  // Keep the historical policies independent of the configured patrol values.
  declare_policy("previous", 0.10, 0.20, 10.0);
  declare_policy("prior_pose", 0.10, 0.20, 10.0);
  declare_policy("patrol", radius, angle, allowance);
  nav2_controller::SimpleProgressChecker previous;
  nav2_controller::PoseProgressChecker prior_pose, patrol;
  previous.initialize(node, "previous"); patrol.initialize(node, "patrol");
  prior_pose.initialize(node, "prior_pose");
  auto p=pose(0);
  require(previous.check(p) && patrol.check(p), "Baseline refused");
  bool old_failed=false;
  for (int i=1; i<=130; ++i) {
    clockAt(clock, 1+i*0.1); p=pose(i*0.1*0.25);
    old_failed |= !previous.check(p);
    require(patrol.check(p), "In-place turning falsely treated as a stall");
  }
  require(old_failed, "Old XY checker did not reproduce the 10s turning timeout");
  std::cout << "PASS official progress check: historical XY-only 10s turn timeout reproduced\n";

  prior_pose.reset(); patrol.reset(); clockAt(clock, 100); p=pose(0);
  require(prior_pose.check(p) && patrol.check(p), "Pause baseline refused");
  clockAt(clock, 115);
  require(!prior_pose.check(p), "Prior 10s Pose policy did not time out during a 15s pause");
  require(patrol.check(p), "Relaxed policy rejected a 15s pause");

  for (bool turning : {false, true}) {
    const double start=turning ? 300.0 : 200.0;
    prior_pose.reset(); patrol.reset(); clockAt(clock, start); p=pose(0);
    require(prior_pose.check(p) && patrol.check(p), "Slow-motion baseline refused");
    old_failed=false;
    for (int second=1; second<=60; ++second) {
      clockAt(clock, start+second);
      p=pose(turning ? second*0.006 : 0.0);
      p.pose.position.x=turning ? 0.0 : second*0.003;
      old_failed |= !prior_pose.check(p);
      require(patrol.check(p), "Relaxed policy falsely rejected sustained slow motion");
    }
    require(old_failed, "Prior Pose policy did not reproduce slow-motion timeout");
  }
  std::cout << "PASS official PoseProgressChecker comparison: old 0.10m/0.20rad/10s fails; configured "
            << radius << "m/" << angle << "rad/" << allowance
            << "s accepts 15s pause and 60s slow walking (0.003m/s) / turning (0.006rad/s)\n";

  patrol.reset(); clockAt(clock, 400); p=pose(0);
  require(patrol.check(p), "Stationary baseline refused");
  clockAt(clock, 400+allowance);
  require(patrol.check(p), "Stationary timeout fired before the configured allowance elapsed");
  clockAt(clock, 400+allowance+0.001);
  require(!patrol.check(p), "A truly stalled robot was allowed indefinitely");

  patrol.reset(); clockAt(clock, 500); p=pose(0);
  require(patrol.check(p), "Jitter baseline refused");
  for (int second=1; second<=int(std::ceil(allowance))+1; ++second) {
    clockAt(clock, 500+second);
    p=pose(second%2 ? 0.02 : -0.02);
    p.pose.position.x=second%2 ? 0.005 : -0.005;
    p.pose.position.y=-p.pose.position.x;
    if (second>allowance) {
      require(!patrol.check(p), "Small position/heading jitter incorrectly counted as progress");
    } else {
      require(patrol.check(p), "Jitter test timed out before its allowance");
    }
  }
  patrol.reset(); clockAt(clock, 600); p=pose(3.12);
  require(patrol.check(p), "Wraparound baseline refused");
  p=pose(-3.12); clockAt(clock, 600+allowance+0.1);
  require(!patrol.check(p), "Yaw wraparound incorrectly counted as a large turn");

  // A reset after failure and a reset for the next target both get a full allowance.
  patrol.reset(); clockAt(clock, 700); p=pose(0);
  require(patrol.check(p), "Reset after timeout did not clear the failed baseline");
  clockAt(clock, 700+allowance*0.75);
  require(patrol.check(p), "Reset did not grant the configured allowance");
  patrol.reset(); require(patrol.check(p), "Next target did not establish a new baseline");
  clockAt(clock, 700+allowance+1.0);
  require(patrol.check(p), "Next target inherited the previous target's elapsed time");
  clockAt(clock, 700+allowance*1.75+0.001);
  require(!patrol.check(p), "Next target lost stationary timeout protection");
  std::cout << "PASS official PoseProgressChecker: stationary timeout boundary, tiny XY/yaw jitter, yaw wraparound and new-target reset\n";

  // Exercise real rclcpp parameter callbacks on an initialized plugin, without restarting it.
  declare_policy("runtime", 0.10, 0.20, 10.0);
  nav2_controller::PoseProgressChecker runtime;
  runtime.initialize(node, "runtime"); clockAt(clock, 800); p=pose(0);
  require(runtime.check(p), "Runtime policy baseline refused");
  clockAt(clock, 815);
  require(!runtime.check(p), "Runtime fixture did not begin with the old 10s policy");
  const auto updated=node->set_parameters_atomically({
    rclcpp::Parameter("runtime.required_movement_radius", radius),
    rclcpp::Parameter("runtime.required_movement_angle", angle),
    rclcpp::Parameter("runtime.movement_time_allowance", allowance)});
  require(updated.successful, "Runtime progress parameter update was refused");
  require(runtime.check(p), "Runtime timeout change did not take effect without restart");
  clockAt(clock, 819); p.pose.position.x=radius+0.01;
  require(runtime.check(p), "Runtime movement-radius update refused a qualifying move");
  clockAt(clock, 819+allowance-1.0);
  require(runtime.check(p), "Runtime movement-radius callback failed to reset the baseline");
  clockAt(clock, 819+allowance+0.001);
  require(!runtime.check(p), "Runtime update removed stationary timeout protection");
  runtime.reset(); clockAt(clock, 900); p=pose(0);
  require(runtime.check(p), "Runtime turn baseline refused");
  clockAt(clock, 919); p=pose(angle+0.01);
  require(runtime.check(p), "Runtime movement-angle update refused a qualifying turn");
  clockAt(clock, 919+allowance-1.0);
  require(runtime.check(p), "Runtime movement-angle callback failed to reset the baseline");
  clockAt(clock, 919+allowance+0.001);
  require(!runtime.check(p), "Runtime turn reset removed stationary timeout protection");
  std::cout << "PASS official PoseProgressChecker: atomic live update changes time/radius/angle without node restart\n";

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
  double turn_start=1000.0;
  for (double target : {2.2, -2.2, 3.13, -3.13}) {
    double heading=0.0;
    nav_msgs::msg::Path path; path.header.frame_id="map";
    for (int i=0; i<=20; ++i) {
      auto point=pose(0); point.pose.position.x=i*.05*std::cos(target);
      point.pose.position.y=i*.05*std::sin(target); path.poses.push_back(point);
    }
    shim.deactivate(); shim.activate(); shim.setPlan(path);
    bool handed_off=false; int turns=0; double max_command=0.0;
    for (int step=0; step<200; ++step) {
      clockAt(clock, turn_start+step*.1); p=pose(heading);
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
      max_command=std::max(max_command,std::abs(command.angular.z));
      heading += command.angular.z*.1; ++turns;
      if (step%10==0) {shim.setPlan(path);} // Match the real 1Hz replanning.
    }
    require(handed_off && turns>0, "New waypoint never transitioned from turning to walking");
    require(std::abs(max_command-yaw_speed)<1e-9, "Dedicated path turn did not reach its requested yaw rate");
    turn_start += 30.0;
  }
  std::cout << "PASS official rotation shim: left/right 126deg and 179deg paths, 1Hz replans, zero measured yaw; reaches "
            << yaw_speed << "rad/s turn output then hands off to 0.30m/s tracker\n";
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
