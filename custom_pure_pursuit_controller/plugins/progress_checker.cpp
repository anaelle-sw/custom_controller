#include "custom_pure_pursuit_controller/plugins/progress_checker.hpp"
#include <cmath>
#include <string>
#include <memory>
#include "nav2_core/exceptions.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace custom_pure_pursuit_controller
{
static double pose_distance(const geometry_msgs::msg::Pose2D &, const geometry_msgs::msg::Pose2D &);

void ProgressChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();

  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".required_movement_radius", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".movement_time_allowance", rclcpp::ParameterValue(10.0));
  // Scale is set to 0 by default, so if it was not set otherwise, set to 0
  node->get_parameter_or(plugin_name + ".required_movement_radius", radius_, 0.5);
  double time_allowance_param = 0.0;
  node->get_parameter_or(plugin_name + ".movement_time_allowance", time_allowance_param, 10.0);
  time_allowance_ = rclcpp::Duration::from_seconds(time_allowance_param);
}

bool ProgressChecker::check(geometry_msgs::msg::PoseStamped & current_pose)
{
  // relies on short circuit evaluation to not call is_robot_moved_enough if
  // baseline_pose is not set.
  geometry_msgs::msg::Pose2D current_pose2d;
  current_pose2d = nav_2d_utils::poseToPose2D(current_pose.pose);

  if ((!baseline_pose_set_) || (is_robot_moved_enough(current_pose2d))) {
    reset_baseline_pose(current_pose2d);
    return true;
  }
  return !((clock_->now() - baseline_time_) > time_allowance_);
}

void ProgressChecker::reset()
{
  baseline_pose_set_ = false;
}

void ProgressChecker::reset_baseline_pose(const geometry_msgs::msg::Pose2D & pose)
{
  baseline_pose_ = pose;
  baseline_time_ = clock_->now();
  baseline_pose_set_ = true;
}

bool ProgressChecker::is_robot_moved_enough(const geometry_msgs::msg::Pose2D & pose)
{
  return pose_distance(pose, baseline_pose_) > radius_;
}

static double pose_distance(
  const geometry_msgs::msg::Pose2D & pose1,
  const geometry_msgs::msg::Pose2D & pose2)
{
  double dx = pose1.x - pose2.x;
  double dy = pose1.y - pose2.y;

  return std::hypot(dx, dy);
}


}  // namespace custom_pure_pursuit_controller

PLUGINLIB_EXPORT_CLASS(custom_pure_pursuit_controller::ProgressChecker, nav2_core::ProgressChecker)