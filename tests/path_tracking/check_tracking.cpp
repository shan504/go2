#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

static void require(bool value, const std::string & message)
{
  if (!value) {throw std::runtime_error(message);}
}

static void clockAt(const rclcpp::Clock::SharedPtr & clock, double seconds)
{
  require(rcl_set_ros_time_override(clock->get_clock_handle(), int64_t(seconds*1e9)) == RCL_RET_OK,
    "Cannot set fixture clock");
}

static geometry_msgs::msg::PoseStamped position(double x, double y, double heading)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x; p.pose.position.y = y;
  p.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(heading);
  return p;
}

static std::vector<geometry_msgs::msg::PoseStamped> route(bool curved)
{
  std::vector<geometry_msgs::msg::PoseStamped> points;
  for (int i=0; i<=120; ++i) {
    double x = i*0.05;
    points.push_back(position(x, curved ? .35*(1-std::cos(x*.7)) : 0.0, 0));
  }
  return points;
}

static nav_msgs::msg::Path replan(
  const std::vector<geometry_msgs::msg::PoseStamped> & full, double x, double y)
{
  nav_msgs::msg::Path path;
  path.header.frame_id="map";
  std::size_t nearest=0;
  double best=1e9;
  for (std::size_t i=0; i<full.size(); ++i) {
    const auto & p=full[i].pose.position;
    double distance=std::hypot(x-p.x,y-p.y);
    if (distance<best) {best=distance; nearest=i;}
  }
  path.poses.push_back(position(x,y,0));
  path.poses.insert(path.poses.end(),full.begin()+nearest,full.end());
  return path;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto clock=node->get_clock();
  require(rcl_enable_ros_time_override(clock->get_clock_handle()) == RCL_RET_OK,
          "No fixture clock");
  clockAt(clock,100);
  const double linear=node->get_parameter("FollowPath.max_vel_x").as_double();
  const double yaw=node->get_parameter("FollowPath.rotate_to_heading_angular_vel").as_double();
  const double accel=node->get_parameter("FollowPath.acc_lim_x").as_double();
  const double angular_accel=node->get_parameter("FollowPath.max_angular_accel").as_double();
  auto options=rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("plugins",std::vector<std::string>()),
    rclcpp::Parameter("filters",std::vector<std::string>()),
    rclcpp::Parameter("global_frame",std::string("odom")),
    rclcpp::Parameter("robot_base_frame",std::string("base_footprint")),
    rclcpp::Parameter("track_unknown_space",true),
    rclcpp::Parameter("width",4), rclcpp::Parameter("height",4),
    rclcpp::Parameter("resolution",0.05),
    rclcpp::Parameter("footprint",std::string("[[-0.40,-0.22],[-0.40,0.22],[0.40,0.22],[0.40,-0.22]]")),
    rclcpp::Parameter("footprint_padding",0.01)});
  auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
  require(map->on_configure(rclcpp_lifecycle::State()) == nav2_util::CallbackReturn::SUCCESS,
          "Costmap configuration failed");
  auto grid=map->getCostmap();
  auto tf=std::make_shared<tf2_ros::Buffer>(clock);
  // DWB uses timed TF calls; match production's dedicated listener thread.
  auto listener=std::make_shared<tf2_ros::TransformListener>(*tf,node,true);
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core","nav2_core::Controller");
  const auto type=node->get_parameter("FollowPath.plugin").as_string();
  require(type=="go2_navigation::PathAlignmentController","Expected installed custom controller profile");
  auto controller=loader.createSharedInstance(type);
  controller->configure(node,"FollowPath",tf,map);
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(node,"general_goal_checker",map);
  // Use separate map/path and odom/costmap frames as production does. The
  // nonidentity map->odom transform ensures geometry cannot pass accidentally
  // because both frame names represented the same coordinates.
  const double map_odom_x=.7, map_odom_y=-.3, map_odom_yaw=.17;
  auto robotPose = [&](double x,double y,double heading) {
    double dx=x-map_odom_x,dy=y-map_odom_y;
    auto p=position(std::cos(map_odom_yaw)*dx+std::sin(map_odom_yaw)*dy,
                    -std::sin(map_odom_yaw)*dx+std::cos(map_odom_yaw)*dy,
                    heading-map_odom_yaw);
    p.header.frame_id="odom";
    return p;
  };
  auto sendTF = [&](const geometry_msgs::msg::PoseStamped & p) {
    geometry_msgs::msg::TransformStamped correction;
    correction.header.frame_id="map"; correction.child_frame_id="odom";
    correction.header.stamp=p.header.stamp;
    correction.transform.translation.x=map_odom_x;
    correction.transform.translation.y=map_odom_y;
    correction.transform.rotation=nav2_util::geometry_utils::orientationAroundZAxis(map_odom_yaw);
    require(tf->setTransform(correction,"tracking_fixture",false),"Map correction TF refused");
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id="odom"; transform.child_frame_id="base_footprint";
    transform.header.stamp=p.header.stamp;
    transform.transform.translation.x=p.pose.position.x;
    transform.transform.translation.y=p.pose.position.y;
    transform.transform.rotation=p.pose.orientation;
    require(tf->setTransform(transform,"tracking_fixture",false),"Odometry TF refused");
  };
  double epoch=100;
  for (double lag : {0.01,0.05,0.10}) {
    for (const auto & scenario : {std::string("straight"),std::string("curved"),std::string("large_heading")}) {
      controller->activate();
      checker.reset();
      auto full=route(scenario=="curved");
      double x=0,y=0,heading=scenario=="large_heading" ? 2.2 : 0.0;
      geometry_msgs::msg::Twist measured;
      int forward_ticks=0, post_start_stops=0, max_stop_streak=0, streak=0;
      bool walked=false;
      for (int tick=0; tick<500 && x<3.1; ++tick) {
        clockAt(clock,epoch+tick*.1);
        auto p=robotPose(x,y,heading);
        p.header.stamp=rclcpp::Time(int64_t((epoch+tick*.1-lag)*1e9));
        sendTF(p);
        grid->resizeMap(80,80,.05,p.pose.position.x-2,p.pose.position.y-2);
        std::fill(grid->getCharMap(),grid->getCharMap()+80*80,nav2_costmap_2d::FREE_SPACE);
        if (tick%10==0) {controller->setPlan(replan(full,x,y));}
        auto command=controller->computeVelocityCommands(p,measured,&checker).twist;
        require(std::isfinite(command.linear.x) && std::isfinite(command.angular.z),"Nonfinite output");
        require(command.linear.x>=-1e-9 && command.linear.x<=linear+1e-9,"Forward envelope changed");
        require(std::abs(command.angular.z)<=yaw+1e-9,"Yaw envelope changed");
        if (command.linear.x>.2) {++forward_ticks; walked=true; streak=0;}
        else if (walked) {++post_start_stops; max_stop_streak=std::max(max_stop_streak,++streak);}
        measured.linear.x=std::clamp(command.linear.x,measured.linear.x-accel*.1,measured.linear.x+accel*.1);
        measured.angular.z=std::clamp(command.angular.z,measured.angular.z-angular_accel*.1,measured.angular.z+angular_accel*.1);
        x+=measured.linear.x*std::cos(heading)*.1;
        y+=measured.linear.x*std::sin(heading)*.1;
        heading+=measured.angular.z*.1;
      }
      require(x>=3.1,"Real DWB failed to make 3m progress: "+scenario);
      require(forward_ticks>90,"Forward motion not sustained: "+scenario);
      require(max_stop_streak<20,"Repeated alignment/tracking stall: "+scenario);
      std::cout<<"PASS actual DWB "<<scenario<<" lag="<<lag<<"s x="<<x<<"m y="<<y
               <<"m forward_ticks="<<forward_ticks<<" post-start stops="<<post_start_stops<<"\n";
      controller->deactivate();
      epoch+=100;
    }
  }
  controller->activate();
  for (unsigned char cost : {nav2_costmap_2d::LETHAL_OBSTACLE,nav2_costmap_2d::NO_INFORMATION}) {
    clockAt(clock,epoch);
    auto p=robotPose(0,0,0); p.header.stamp=clock->now();
    sendTF(p);
    grid->resizeMap(80,80,.05,p.pose.position.x-2,p.pose.position.y-2);
    std::fill(grid->getCharMap(),grid->getCharMap()+80*80,cost);
    controller->setPlan(replan(route(false),0,0));
    bool blocked=false;
    try {
      auto command=controller->computeVelocityCommands(p,geometry_msgs::msg::Twist(),&checker).twist;
      blocked=std::abs(command.linear.x)<1e-9 && std::abs(command.linear.y)<1e-9;
    } catch (const std::exception &) {blocked=true;}
    require(blocked,"Actual DWB allowed unchecked translation in blocked/unknown map");
    ++epoch;
  }
  std::cout<<"PASS actual DWB rejects translation in lethal and tracked unknown costmaps\n";
  controller->deactivate(); controller->cleanup(); controller.reset();
  map->on_cleanup(rclcpp_lifecycle::State()); map.reset();
  node.reset(); rclcpp::shutdown();
}
