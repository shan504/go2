#ifndef GO2_NAVIGATION__PATH_ALIGNMENT_CONTROLLER_HPP_
#define GO2_NAVIGATION__PATH_ALIGNMENT_CONTROLLER_HPP_

#include "nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp"

namespace go2_navigation
{
// Preserve the official rotation ramp, collision checker and primary controller,
// but evaluate path geometry against a fresh latest robot transform. A path is
// geometry, not a sensor observation to relabel with the current wall time.
class PathAlignmentController : public nav2_rotation_shim_controller::RotationShimController
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

protected:
  double pathHeadingError();
  void checkImmediateRotation(
    const geometry_msgs::msg::TwistStamped & command,
    const geometry_msgs::msg::PoseStamped & pose) const;
  double latest_tf_age_{0.0};
  double sampled_path_distance_{0.0};
};
}  // namespace go2_navigation
#endif  // GO2_NAVIGATION__PATH_ALIGNMENT_CONTROLLER_HPP_
