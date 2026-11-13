#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "angles/angles.h"
#include "go2_navigation/path_alignment_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace go2_navigation
{
namespace
{
bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}
}  // namespace

void PathAlignmentController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  RotationShimController::configure(parent, name, tf, costmap_ros);
  if (rotate_to_goal_heading_) {
    throw nav2_core::PlannerException(
            "PathAlignmentController supports position-only goals; rotate_to_goal_heading must be false");
  }
}

void PathAlignmentController::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.header.frame_id.empty() || path.poses.empty()) {
    throw nav2_core::PlannerException("Path alignment requires a nonempty framed path");
  }
  for (const auto & point : path.poses) {
    if (!finitePose(point.pose) ||
      (!point.header.frame_id.empty() && point.header.frame_id != path.header.frame_id))
    {
      throw nav2_core::PlannerException("Path alignment refuses nonfinite or mixed-frame path geometry");
    }
  }
  // The inherited implementation updates DWB's plan without resetting an
  // ongoing alignment. Replans therefore retain the release-angle hysteresis.
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(
    *costmap_ros_->getCostmap()->getMutex());
  std::lock_guard<std::mutex> lock(mutex_);
  RotationShimController::setPlan(path);
}

double PathAlignmentController::pathHeadingError()
{
  if (current_path_.poses.empty() || current_path_.header.frame_id.empty() ||
    !std::isfinite(forward_sampling_distance_) || forward_sampling_distance_ <= 0.0)
  {
    throw nav2_core::PlannerException("Invalid path alignment geometry or sampling distance");
  }
  const auto robot = tf_->lookupTransform(
    current_path_.header.frame_id, costmap_ros_->getBaseFrameID(), tf2::TimePointZero);
  const rclcpp::Time stamp(robot.header.stamp, clock_->get_clock_type());
  const double age = (clock_->now() - stamp).seconds();
  latest_tf_age_ = age;
  // Same finite TF freshness envelope used by the shared navigation readiness
  // path: do not wait for a nonexistent future transform or accept stale TF.
  if (stamp.nanoseconds() <= 0 || !std::isfinite(age) || age < -0.1 || age > 0.3) {
    throw nav2_core::PlannerException(
            "Path alignment robot TF is stale or future-dated; age=" + std::to_string(age) + "s");
  }
  const auto & p = robot.transform.translation;
  const auto & q = robot.transform.rotation;
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !std::isfinite(norm) || std::abs(norm-1.0) > 1e-3)
  {
    throw nav2_core::PlannerException("Path alignment refuses nonfinite or invalid robot TF");
  }
  std::size_t nearest = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < current_path_.poses.size(); ++i) {
    const auto & point = current_path_.poses[i].pose.position;
    const double distance = std::hypot(point.x-p.x, point.y-p.y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = i;
    }
  }
  // Start at the robot's nearest remaining path point. Sampling from the old
  // path's first point can point behind a robot that has already moved ahead.
  std::size_t sample = nearest;
  double arc_length = 0.0;
  while (sample+1 < current_path_.poses.size() && arc_length < forward_sampling_distance_) {
    const auto & a = current_path_.poses[sample].pose.position;
    const auto & b = current_path_.poses[sample+1].pose.position;
    arc_length += std::hypot(b.x-a.x, b.y-a.y);
    ++sample;
  }
  const auto & target = current_path_.poses[sample].pose.position;
  const double dx = target.x-p.x, dy = target.y-p.y;
  sampled_path_distance_ = std::hypot(dx, dy);
  if (sampled_path_distance_ < 1e-6) {
    return 0.0;  // At the last path point; let the normal goal checker finish.
  }
  return angles::shortest_angular_distance(tf2::getYaw(q), std::atan2(dy, dx));
}

void PathAlignmentController::checkImmediateRotation(
  const geometry_msgs::msg::TwistStamped & command,
  const geometry_msgs::msg::PoseStamped & pose) const
{
  const auto footprint = costmap_ros_->getRobotFootprint();
  const double resolution = costmap_ros_->getCostmap()->getResolution();
  const double rotation = command.twist.angular.z * control_duration_;
  if (footprint.size() < 3 || !std::isfinite(resolution) || resolution <= 0.0 ||
    !std::isfinite(rotation) || !std::isfinite(control_duration_) || control_duration_ <= 0.0)
  {
    throw nav2_core::PlannerException("Invalid immediate rotation footprint or control period");
  }
  double radius = 0.0;
  for (const auto & vertex : footprint) {
    if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y)) {
      throw nav2_core::PlannerException("Nonfinite immediate rotation footprint");
    }
    radius = std::max(radius, std::hypot(vertex.x, vertex.y));
  }
  // Upstream can stop its ahead-check before the first footprint sample when
  // retained rotation is between the entry and release thresholds. Always
  // check the current pose and the complete next commanded control period,
  // using the same real footprint checker. Substeps move every corner by at
  // most half a costmap cell and rotate no more than 0.01rad per sample.
  const double angular_step = radius > 0.0 ? std::min(0.01, resolution/(2.0*radius)) : 0.01;
  const double required_steps = std::ceil(std::abs(rotation)/angular_step);
  if (!std::isfinite(angular_step) || angular_step <= 0.0 ||
    !std::isfinite(required_steps) || required_steps > 10000.0)
  {
    throw nav2_core::PlannerException("Invalid immediate rotation sweep resolution");
  }
  const int steps = std::max(1, static_cast<int>(required_steps));
  const double initial_yaw = tf2::getYaw(pose.pose.orientation);
  for (int step = 0; step <= steps; ++step) {
    const double yaw = initial_yaw + rotation * static_cast<double>(step)/steps;
    const double cost = collision_checker_->footprintCostAtPose(
      pose.pose.position.x, pose.pose.position.y, yaw, footprint);
    if (!std::isfinite(cost) || cost < 0.0 ||
      cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE))
    {
      throw nav2_core::PlannerException(
              "Current or immediate swept footprint is lethal, unknown or outside the costmap");
    }
  }
}

geometry_msgs::msg::TwistStamped PathAlignmentController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * goal_checker)
{
  if (!finitePose(pose.pose) || pose.header.frame_id != costmap_ros_->getGlobalFrameID()) {
    throw nav2_core::PlannerException("Path alignment refuses invalid or wrong-frame current pose");
  }
  {
    // Match upstream lock ordering: the collision checker reads the costmap,
    // while dynamic parameter callbacks and setPlan share the shim mutex.
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(
      *costmap_ros_->getCostmap()->getMutex());
    std::lock_guard<std::mutex> state_lock(mutex_);
    try {
      const double error = pathHeadingError();
      if (path_updated_ || in_rotation_) {
        const double threshold = in_rotation_ ? angular_disengage_threshold_ : angular_dist_threshold_;
        if (std::abs(error) > threshold) {
          in_rotation_ = true;
          auto command = computeRotateToHeadingCommand(error, pose, velocity);
          checkImmediateRotation(command, pose);
          last_angular_vel_ = command.twist.angular.z;
          RCLCPP_INFO_THROTTLE(logger_, *clock_, 3000,
            "path_alignment phase=aligning heading_error=%.3frad latest_tf_age=%.3fs sampled_distance=%.3fm",
            error, latest_tf_age_, sampled_path_distance_);
          return command;
        }
        path_updated_ = false;
        in_rotation_ = false;
      }
      RCLCPP_INFO_THROTTLE(logger_, *clock_, 3000,
        "path_alignment phase=tracking heading_error=%.3frad latest_tf_age=%.3fs sampled_distance=%.3fm",
        error, latest_tf_age_, sampled_path_distance_);
    } catch (const std::exception & error) {
      // A missing TF or unsafe rotation is not permission to drive forward.
      // Retain alignment state so the next cycle rechecks the same path.
      throw nav2_core::PlannerException(std::string("Path alignment stopped: ") + error.what());
    }
  }
  auto command = primary_controller_->computeVelocityCommands(pose, velocity, goal_checker);
  last_angular_vel_ = command.twist.angular.z;
  return command;
}
}  // namespace go2_navigation

PLUGINLIB_EXPORT_CLASS(go2_navigation::PathAlignmentController, nav2_core::Controller)
