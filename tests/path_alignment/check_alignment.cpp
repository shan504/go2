#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

#include "rcl/time.h"
#include "angles/angles.h"
#include "go2_navigation/path_alignment_controller.hpp"
#include "nav2_controller/plugins/position_goal_checker.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_loader.hpp"

static void require(bool result, const char * message)
{
  if (!result) {throw std::runtime_error(message);}
}

// An observer identifies the exact handoff to the primary controller. This is
// not a substitute for DWB; the independent path-tracking suite runs real DWB.
class ForwardObserver : public nav2_core::Controller
{
public:
  ForwardObserver(double speed, int * calls) : speed_(speed), calls_(calls) {}
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, std::string,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void activate() override {}
  void deactivate() override {}
  void cleanup() override {}
  void setPlan(const nav_msgs::msg::Path &) override {}
  void setSpeedLimit(const double &, const bool &) override {}
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *) override
  {
    ++*calls_;
    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    command.twist.linear.x = speed_;
    return command;
  }
private:
  double speed_;
  int * calls_;
};

struct Policy
{
  double threshold, release, sample, yaw, accel, ahead, forward;
};

template<class Base>
class Fixture : public Base
{
public:
  int primary_calls{0};
  void initialize(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
    const std::shared_ptr<tf2_ros::Buffer> & buffer,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & map, const Policy & policy)
  {
    this->node_ = node;
    this->clock_ = node->get_clock();
    this->tf_ = buffer;
    this->costmap_ros_ = map;
    this->plugin_name_ = "FollowPath";
    this->angular_dist_threshold_ = policy.threshold;
    this->angular_disengage_threshold_ = policy.release;
    this->forward_sampling_distance_ = policy.sample;
    this->rotate_to_heading_angular_vel_ = policy.yaw;
    this->max_angular_accel_ = policy.accel;
    this->simulate_ahead_time_ = policy.ahead;
    this->control_duration_ = 0.1;
    this->closed_loop_ = false;
    this->rotate_to_goal_heading_ = false;
    this->primary_controller_ = std::make_unique<ForwardObserver>(policy.forward, &primary_calls);
    this->collision_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<
      nav2_costmap_2d::Costmap2D *>>(map->getCostmap());
    this->position_goal_checker_ = std::make_unique<nav2_controller::PositionGoalChecker>();
    this->activate();
  }
  bool alignmentPending() const {return this->path_updated_ || this->in_rotation_;}
  void retainAlignment(double angular = 0.0) {
    this->in_rotation_ = true; this->last_angular_vel_ = angular;
  }
  void close() {this->deactivate(); this->cleanup();}
};

using SafeFixture = Fixture<go2_navigation::PathAlignmentController>;
using OriginalFixture = Fixture<nav2_rotation_shim_controller::RotationShimController>;

static void clockAt(const rclcpp::Clock::SharedPtr & clock, double seconds)
{
  require(rcl_set_ros_time_override(clock->get_clock_handle(), int64_t(seconds*1e9)) == RCL_RET_OK,
    "Cannot set deterministic test clock");
}

static geometry_msgs::msg::PoseStamped robotPose(double yaw, double x=0.0, double y=0.0)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
  return pose;
}

static nav_msgs::msg::Path pathTo(double heading)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i=0; i<=60; ++i) {
    auto point = robotPose(heading, i*.05*std::cos(heading), i*.05*std::sin(heading));
    path.poses.push_back(point);
  }
  return path;
}

static std::shared_ptr<tf2_ros::Buffer> buffer()
{
  // The lookup timeout needs a real advancing clock even while the controller
  // receives a deterministic ROS clock. Otherwise the intentionally failing
  // upstream future lookup would wait forever on a frozen simulated clock.
  auto result = std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  result->setUsingDedicatedThread(true);
  return result;
}

static bool transform(const std::shared_ptr<tf2_ros::Buffer> & tf, double stamp,
  double heading=0.0, double x=0.0, double y=0.0)
{
  geometry_msgs::msg::TransformStamped value;
  value.header.frame_id = "map";
  value.child_frame_id = "base_footprint";
  value.header.stamp = rclcpp::Time(int64_t(stamp*1e9), RCL_ROS_TIME);
  value.transform.translation.x = x;
  value.transform.translation.y = y;
  value.transform.rotation = nav2_util::geometry_utils::orientationAroundZAxis(heading);
  return tf->setTransform(value, "dynamic_alignment_test", false);
}

static void refuses(const std::function<void()> & call, const char * message)
{
  bool rejected = false;
  try {call();} catch (const std::exception &) {rejected = true;}
  require(rejected, message);
}

int main(int argc, char ** argv)
{
  if (argc != 8) {return 2;}
  Policy policy{std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3]),
    std::stod(argv[4]), std::stod(argv[5]), std::stod(argv[6]), std::stod(argv[7])};
  rclcpp::init(0, nullptr);
#ifdef GO2_TEST_INSTALLED_PLUGIN
  {
    pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
    auto actual = loader.createSharedInstance("go2_navigation::PathAlignmentController");
    require(dynamic_cast<go2_navigation::PathAlignmentController *>(actual.get()) != nullptr,
      "Installed pluginlib registration returned the wrong controller");
  }
  std::cout << "PASS installed go2_navigation pluginlib load and real derived controller type\n";
#endif
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("path_alignment_fixture");
  auto clock = node->get_clock();
  require(rcl_enable_ros_time_override(clock->get_clock_handle()) == RCL_RET_OK, "No fixture clock");
  auto options = rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("plugins", std::vector<std::string>()),
    rclcpp::Parameter("track_unknown_space", true),
    rclcpp::Parameter("global_frame", std::string("map")),
    rclcpp::Parameter("robot_base_frame", std::string("base_footprint"))});
  auto map = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
  map->on_configure(rclcpp_lifecycle::State());
  auto grid = map->getCostmap();
  grid->resizeMap(160, 160, 0.05, -4.0, -4.0);
  std::fill(grid->getCharMap(), grid->getCharMap()+160*160, nav2_costmap_2d::FREE_SPACE);
  std::vector<geometry_msgs::msg::Point> footprint;
  for (auto xy : {std::pair<double,double>{-.41,-.23}, {-.41,.23}, {.41,.23}, {.41,-.23}}) {
    geometry_msgs::msg::Point point;
    point.x = xy.first; point.y = xy.second; footprint.push_back(point);
  }
  map->setRobotFootprint(footprint);
  geometry_msgs::msg::Twist still;

  for (double lag : {0.01, 0.05, 0.10}) {
    clockAt(clock, 100.0);
    auto tf = buffer(); transform(tf, 100.0-lag);
    OriginalFixture old;
    old.initialize(node, tf, map, policy); old.setPlan(pathTo(2.2));
    const auto command = old.computeVelocityCommands(robotPose(0), still, nullptr).twist;
    require(old.primary_calls == 1 && command.linear.x > 0 && !old.alignmentPending(),
      "Original shim did not reproduce premature DWB handoff under delayed dynamic TF");
    old.close();
    for (double target : {2.2, -2.2, 3.13, -3.13}) {
      tf = buffer();
      SafeFixture safe; safe.initialize(node, tf, map, policy);
      const auto path = pathTo(target); safe.setPlan(path);
      double heading = 0.0, previous = 0.0, maximum = 0.0;
      bool handed_off = false;
      for (int step=0; step<200; ++step) {
        const double now = 200.0+step*.1;
        clockAt(clock, now); transform(tf, now-lag, heading);
        if (step%10 == 0) {safe.setPlan(path);}
        const auto cmd = safe.computeVelocityCommands(robotPose(heading), still, nullptr).twist;
        require(std::abs(cmd.angular.z) <= policy.yaw+1e-9, "Angular speed limit changed");
        if (cmd.linear.x > 0) {
          require(safe.primary_calls == 1, "Unexpected earlier primary-controller call");
          require(std::abs(angles::shortest_angular_distance(heading,target)) <= policy.release+1e-8,
            "Tracker handed off before angular release threshold");
          require(std::abs(cmd.linear.x-policy.forward) < 1e-9, "Forward speed changed at handoff");
          handed_off = true;
          break;
        }
        require(safe.primary_calls == 0 && cmd.linear.x == 0.0 && cmd.linear.y == 0.0,
          "Primary controller or translation ran before alignment completed");
        require(cmd.angular.z*target > 0.0, "Incorrect turn direction");
        require(std::abs(cmd.angular.z-previous) <= policy.accel*.1+1e-9, "Angular acceleration limit changed");
        maximum = std::max(maximum,std::abs(cmd.angular.z));
        heading += cmd.angular.z*.1;
        previous = cmd.angular.z;
      }
      require(handed_off && std::abs(maximum-policy.yaw) < 1e-9,
        "Delayed-TF alignment did not reach normal yaw speed and hand off");
      // Freshness is also mandatory after handing off to tracking.
      clockAt(clock, clock->now().seconds()+0.5);
      refuses([&] {safe.computeVelocityCommands(robotPose(heading),still,nullptr);},
        "Stale tracking TF allowed a primary controller call");
      require(safe.primary_calls == 1, "Stale TF reached primary controller after handoff");
      safe.close();
    }
  }
  std::cout << "PASS dynamic TF lag 10/50/100ms: original premature handoff reproduced; safe left/right 126/179deg alignment survives 1Hz replans, yaw/accel retained\n";

  for (const auto & mode : {"missing", "stale", "future", "nonfinite"}) {
    clockAt(clock, 500.0);
    auto tf = buffer();
    if (std::string(mode) == "stale") {transform(tf, 499.69);}
    if (std::string(mode) == "future") {transform(tf, 500.11);}
    if (std::string(mode) == "nonfinite") {
      require(!transform(tf, 500.0, 0.0, std::numeric_limits<double>::quiet_NaN()),
        "tf2 accepted a nonfinite transform");
    }
    SafeFixture safe; safe.initialize(node, tf, map, policy); safe.setPlan(pathTo(2.2));
    refuses([&] {safe.computeVelocityCommands(robotPose(0),still,nullptr);},
      "Invalid TF did not reject alignment");
    require(safe.primary_calls == 0 && safe.alignmentPending(),
      "Invalid TF cleared pending alignment or reached the primary controller");
    transform(tf, 500.0);
    auto bad = robotPose(0); bad.pose.position.x = std::numeric_limits<double>::quiet_NaN();
    refuses([&] {safe.computeVelocityCommands(bad,still,nullptr);}, "Nonfinite current pose accepted");
    auto path = pathTo(0); path.poses[5].pose.position.y = std::numeric_limits<double>::infinity();
    refuses([&] {safe.setPlan(path);}, "Nonfinite path accepted");
    safe.close();
  }
  std::cout << "PASS missing/stale/future/nonfinite TF and nonfinite poses/paths reject safely without DWB fallback\n";

  for (unsigned char cost : {nav2_costmap_2d::LETHAL_OBSTACLE, nav2_costmap_2d::NO_INFORMATION}) {
    std::fill(grid->getCharMap(), grid->getCharMap()+160*160, cost);
    clockAt(clock, 600.0); auto tf = buffer(); transform(tf, 599.95);
    SafeFixture safe; safe.initialize(node, tf, map, policy); safe.setPlan(pathTo(2.2));
    refuses([&] {safe.computeVelocityCommands(robotPose(0),still,nullptr);},
      "Unsafe footprint rotation was accepted");
    require(safe.primary_calls == 0 && safe.alignmentPending(),
      "Unsafe rotation switched to primary controller instead of stopping");
    safe.close();
  }
  // The upstream ahead-check subtracts the entry threshold even during
  // hysteresis. At |error|=.25 (<entry .35 but >release .20), it can exit
  // before checking a single footprint. The custom shim must still check the
  // current and immediate commanded sweep, for both signs and blocked types.
  for (unsigned char cost : {nav2_costmap_2d::LETHAL_OBSTACLE, nav2_costmap_2d::NO_INFORMATION}) {
    std::fill(grid->getCharMap(), grid->getCharMap()+160*160, cost);
    for (double error : {-0.25, 0.25}) {
      clockAt(clock, 650.0); auto tf = buffer(); transform(tf, 650.0);
      OriginalFixture old; old.initialize(node, tf, map, policy);
      old.setPlan(pathTo(error)); old.retainAlignment();
      const auto unchecked = old.computeVelocityCommands(robotPose(0),still,nullptr).twist;
      require(std::abs(unchecked.angular.z) > 0.0 && old.primary_calls == 0,
        "Upstream release-band unchecked rotation was not reproduced");
      old.close();
      SafeFixture safe; safe.initialize(node, tf, map, policy);
      safe.setPlan(pathTo(error)); safe.retainAlignment();
      refuses([&] {safe.computeVelocityCommands(robotPose(0),still,nullptr);},
        "Retained rotation skipped collision checking inside the release band");
      require(safe.primary_calls == 0 && safe.alignmentPending(),
        "Blocked release-band rotation discarded alignment or reached DWB");
      safe.close();
    }
  }
  std::cout << "PASS retained +/-0.25rad rotation: upstream unchecked release-band case reproduced; lethal/unknown current and immediate footprint reject safely\n";
  // A single blocked cell touches only the next rotated footprint, not the
  // current one. This prevents a current-pose-only check from passing the test.
  grid->resizeMap(1600,1600,0.005,-4.0,-4.0);
  const auto actual_footprint = map->getRobotFootprint();
  for (unsigned char cost : {nav2_costmap_2d::LETHAL_OBSTACLE, nav2_costmap_2d::NO_INFORMATION}) {
    for (double sign : {-1.0, 1.0}) {
      std::fill(grid->getCharMap(),grid->getCharMap()+1600*1600,nav2_costmap_2d::FREE_SPACE);
      const double rotation = sign*policy.yaw*.1;
      const auto corner = *std::max_element(actual_footprint.begin(),actual_footprint.end(),
        [sign](const auto & a,const auto & b) {return a.x+sign*a.y < b.x+sign*b.y;});
      const double x = corner.x*std::cos(rotation)-corner.y*std::sin(rotation);
      const double y = corner.x*std::sin(rotation)+corner.y*std::cos(rotation);
      unsigned int cell_x,cell_y;
      require(grid->worldToMap(x,y,cell_x,cell_y),"Immediate obstacle test cell outside grid");
      grid->setCost(cell_x,cell_y,cost);
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(grid);
      require(checker.footprintCostAtPose(0,0,0,actual_footprint) == 0,
        "Immediate obstacle fixture already blocks current footprint");
      require(checker.footprintCostAtPose(0,0,rotation,actual_footprint) >=
        static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE),
        "Immediate obstacle fixture does not block next footprint");
      clockAt(clock,675.0); auto tf = buffer(); transform(tf,675.0);
      SafeFixture safe; safe.initialize(node,tf,map,policy);
      safe.setPlan(pathTo(sign*.25)); safe.retainAlignment(sign*policy.yaw);
      refuses([&] {safe.computeVelocityCommands(robotPose(0),still,nullptr);},
        "Next-control-period footprint obstacle was missed inside release band");
      require(safe.primary_calls == 0 && safe.alignmentPending(),
        "Next-footprint obstacle allowed tracking handoff");
      safe.close();
    }
  }
  std::cout << "PASS immediate sweep: current footprint free but next footprint lethal/unknown blocks both rotation signs\n";
  grid->resizeMap(160,160,0.05,-4.0,-4.0);
  std::fill(grid->getCharMap(), grid->getCharMap()+160*160, nav2_costmap_2d::FREE_SPACE);
  clockAt(clock, 700.0); auto tf = buffer(); transform(tf, 699.95, 0.0, 2.0);
  SafeFixture moved; moved.initialize(node, tf, map, policy); moved.setPlan(pathTo(0));
  const auto cmd = moved.computeVelocityCommands(robotPose(0,2.0),still,nullptr).twist;
  require(moved.primary_calls == 1 && cmd.linear.x == policy.forward,
    "Path sampling pointed behind a robot that moved beyond the first path point");
  moved.close();
  std::cout << "PASS official swept footprint rejects lethal/unknown rotation without fallback; sampling follows current position rather than old path start\n";
  map->on_cleanup(rclcpp_lifecycle::State());
  rclcpp::shutdown();
}
