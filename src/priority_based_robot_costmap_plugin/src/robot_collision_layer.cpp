/*********************************************************************
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 *********************************************************************/
#include "robot_collision_layer/robot_collision_layer.hpp"

#include "hupbrb_msgs/msg/collision.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <string>
#include <math.h>
#include <memory>

#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace priority_based_robot_costmap_plugin
{

double calcCost(double distance, double radius){
  return 255 * ( 1 + ((radius-distance) / (radius * 0.5)));
}

RobotCollisionLayer::RobotCollisionLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max()),
  is_enabled_(false)
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initializatidon
// of need_recalculation_ variable.
void
RobotCollisionLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node_->get_parameter(name_ + "." + "enabled", enabled_);
  
  need_recalculation_ = false;
  current_ = true;
  
  subscription_ = node_->create_subscription<hupbrb_msgs::msg::Collision>(
    "collision_point", 10, std::bind(&RobotCollisionLayer::topic_callback, this, std::placeholders::_1));
}

void RobotCollisionLayer::topic_callback(const hupbrb_msgs::msg::Collision::SharedPtr msg)
{
  RCLCPP_INFO(node_->get_logger(), "Collision expected at: '%lf, %lf'", msg->point.x, msg->point.y);
  collision_x = msg->point.x;
  collision_y = msg->point.y;
  radius_ = msg->radius;
  is_enabled_ = msg->enabled;
}

void
RobotCollisionLayer::updateBounds(
  
  double robot_x, double robot_y, double robot_yaw,
  double * min_x,  double * min_y, double * max_x, double * max_y)
{
  robot_x_ = robot_x;
  robot_y_ = robot_y;
  robot_yaw_ = robot_yaw;

  *min_x = -10.;
  *min_y = -10.;
  *max_x = 10.;
  *max_y = 10.;

}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void
RobotCollisionLayer::onFootprintChanged()
{
  RCLCPP_INFO(rclcpp::get_logger(
      "nav2_costmap_2d"), "RobotCollisionLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

void
RobotCollisionLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{

  if (!is_enabled_){
    RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "Costmap not enabled");
    return;
  }

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "RobotCollisionLayer::updateCosts: %lu, %lu, %lu, %lu",
    min_i, min_j, max_i, max_j);
  
  auto costmap = layered_costmap_->getCostmap();

  for (int i = min_i; i < max_i; i++) {
    for (int j = -min_j; j < max_j; j++) {
      double wi, wj;
      master_grid.mapToWorld(i,j, wi,wj);

      double dist = sqrt(pow(wi-collision_x, 2)+ pow(wj-collision_y, 2));

      double cost = calcCost(dist, radius_);

      unsigned char maxCostmap = std::clamp(cost, (double) costmap->getCost(i,j), 255.);

      costmap->setCost(i,j, maxCostmap);

    }
  }
}

}  // namespace priority_based_robot_costmap_plugin

// This is the macro allowing a priority_based_robot_costmap_plugin::RobotCollisionLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(priority_based_robot_costmap_plugin::RobotCollisionLayer, nav2_costmap_2d::Layer)
