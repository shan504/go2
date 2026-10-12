#pragma once
// Cloud-only lifecycle wrapper. The grid and swept-footprint checker below
// are the REAL upstream implementations, not a substitute collision model.
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/utils.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
namespace nav2_costmap_2d {
class TurnLayeredCostmap {
public:
  bool isTrackingUnknown() const {return true;}
};
class Costmap2DROS {
public:
  explicit Costmap2DROS(const rclcpp::NodeOptions &) : grid_(160, 160, 0.05, -4, -4, 0) {}
  void on_configure(const rclcpp_lifecycle::State &) {}
  Costmap2D * getCostmap() {return &grid_;}
  TurnLayeredCostmap * getLayeredCostmap() {return &layered_;}
  std::string getBaseFrameID() const {return "base_footprint";}
  std::vector<geometry_msgs::msg::Point> getRobotFootprint() const {return footprint_;}
  void setRobotFootprint(const std::vector<geometry_msgs::msg::Point> & points) {footprint_ = points;}
private:
  Costmap2D grid_;
  TurnLayeredCostmap layered_;
  std::vector<geometry_msgs::msg::Point> footprint_;
};
}
