#include <cmath>
#include <string>
#include <memory>

#include "nav2_core/exceptions.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "custom_pure_pursuit_controller/plugins/progress_checker.hpp"

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

  // Get parent node
  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // Declare and get parameters
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".required_movement_radius", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".movement_time_allowance", rclcpp::ParameterValue(10.0));
  node->get_parameter_or(plugin_name + ".required_movement_radius", radius_, 0.5);
  double time_allowance_param = 0.0;
  node->get_parameter_or(plugin_name + ".movement_time_allowance", time_allowance_param, 10.0);
  time_allowance_ = rclcpp::Duration::from_seconds(time_allowance_param);

  // Initialize publisher
  progress_status_pub_ =
    node->create_publisher<custom_msgs::msg::ProgressStatus>(
    "/follow_path/progress_status", rclcpp::QoS(1).transient_local());
  progress_status_pub_->on_activate();
  progress_status_.status = custom_msgs::msg::ProgressStatus::UNKNOWNED;

  // Initialize subscription
  plan_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    "/plan", 1, std::bind(&ProgressChecker::plan_callback, this, _1));
}

bool ProgressChecker::check(geometry_msgs::msg::PoseStamped & current_pose)
{
  // Relies on short circuit evaluation to not call is_robot_moved_enough if
  // baseline_pose is not set.
  geometry_msgs::msg::Pose2D current_pose2d;
  current_pose2d = nav_2d_utils::poseToPose2D(current_pose.pose);
  if ((!baseline_pose_set_) || (is_robot_moved_enough(current_pose2d))) {
    reset_baseline_pose(current_pose2d);
    pub_progress_status_on_change(custom_msgs::msg::ProgressStatus::ROBOT_PROGRESSING);
    return true;
  }

  if (!((clock_->now() - baseline_time_) > time_allowance_)) {
    pub_progress_status_on_change(custom_msgs::msg::ProgressStatus::ROBOT_PROGRESSING);
    return true;
  }

  pub_progress_status_on_change(custom_msgs::msg::ProgressStatus::ROBOT_STUCK);
  return false;
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

void ProgressChecker::pub_progress_status_on_change(const uint8_t progress_status)
{
  // Publish progress status on change only (transient local topic)
  if (progress_status_.status != progress_status) {
    progress_status_.status = progress_status;
    progress_status_pub_->publish(progress_status_);
  }
}

void ProgressChecker::plan_callback(const nav_msgs::msg::Path::SharedPtr /*path_msg*/)
{
  // Always publish progress status when receiving a new path
  progress_status_.status = custom_msgs::msg::ProgressStatus::FOLLOWING_NEW_PATH;
  progress_status_pub_->publish(progress_status_);
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
