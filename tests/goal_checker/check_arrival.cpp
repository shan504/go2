#include <cmath>
#include <iostream>
#include <stdexcept>
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_util/geometry_utils.hpp"

static void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

int main(int argc, char ** argv)
{
  if (argc != 4) {return 2;}
  const double xy = std::stod(argv[1]), yaw = std::stod(argv[2]);
  const bool stateful = std::string(argv[3]) == "true";
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("arrival_fixture");
  for (const auto & prefix : {"previous", "patrol"}) {
    node->declare_parameter(std::string(prefix)+".xy_goal_tolerance", prefix == std::string("previous") ? 0.25 : xy);
    node->declare_parameter(std::string(prefix)+".yaw_goal_tolerance", prefix == std::string("previous") ? 0.25 : yaw);
    node->declare_parameter(std::string(prefix)+".stateful", prefix == std::string("previous") ? true : stateful);
  }
  nav2_controller::SimpleGoalChecker previous, patrol;
  previous.initialize(node, "previous", nullptr);
  patrol.initialize(node, "patrol", nullptr);
  geometry_msgs::msg::Pose goal, current;
  geometry_msgs::msg::Twist velocity;
  goal.orientation.w = 1.0;
  current.position.x = 0.20;
  current.orientation = nav2_util::geometry_utils::orientationAroundZAxis(M_PI/2);
  require(!previous.isGoalReached(current, goal, velocity), "Previous checker accepted an unaligned heading");
  require(patrol.isGoalReached(current, goal, velocity), "Patrol waypoint still requires terminal heading");
  for (double heading : {-M_PI, -2.9, -M_PI/2, 0.0, M_PI/2, 2.9, M_PI}) {
    current.orientation = nav2_util::geometry_utils::orientationAroundZAxis(heading);
    current.position.x = xy;
    require(patrol.isGoalReached(current, goal, velocity), "Inside-radius arrival rejected for a heading");
    current.position.x = xy+0.001;
    require(!patrol.isGoalReached(current, goal, velocity), "Outside-radius robot falsely marked arrived");
    current.position.x = 1.5;
    require(!patrol.isGoalReached(current, goal, velocity), "Faraway stalled robot falsely marked arrived");
  }
  current.position.x = current.position.y = xy*0.71;
  require(!patrol.isGoalReached(current, goal, velocity), "Arrival radius checked per axis rather than Euclidean distance");
  current.position.x = current.position.y = xy*0.70;
  require(patrol.isGoalReached(current, goal, velocity), "Diagonal in-radius waypoint rejected");
  patrol.reset();
  current.position.x = 0.4;
  current.position.y = 0.0;
  require(!patrol.isGoalReached(current, goal, velocity), "Reset carried prior arrival into a new goal");
  rclcpp::shutdown();
  std::cout << "PASS official SimpleGoalChecker: previous XY arrival rejects 90deg heading; patrol accepts all headings inside "
            << xy << "m, rejects current outside-radius/far/diagonal poses, and resets between goals\n";
}
