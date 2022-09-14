#ifndef CUSTOM_PURE_PURSUIT_CONTROLLER__PLUGINS__PROGRESS_CHECKER_HPP_
#define CUSTOM_PURE_PURSUIT_CONTROLLER__PLUGINS__PROGRESS_CHECKER_HPP_

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/progress_checker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace custom_pure_pursuit_controller
{
/**
* @class ProgressChecker
* @brief This plugin is used to check the position of the robot to make sure
* that it is actually progressing towards a goal.
* It publishes a message if the robot appears to be stuck.
*/

class ProgressChecker : public nav2_core::ProgressChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override;
  bool check(geometry_msgs::msg::PoseStamped & current_pose) override;
  void reset() override;

protected:
  /**
   * @brief Calculates robots movement from baseline pose
   * @param pose Current pose of the robot
   * @return true, if movement is greater than radius_, or false
   */
  bool is_robot_moved_enough(const geometry_msgs::msg::Pose2D & pose);
  /**
   * @brief Resets baseline pose with the current pose of the robot
   * @param pose Current pose of the robot
   */
  void reset_baseline_pose(const geometry_msgs::msg::Pose2D & pose);

  rclcpp::Clock::SharedPtr clock_;

  double radius_;
  rclcpp::Duration time_allowance_{0, 0};

  geometry_msgs::msg::Pose2D baseline_pose_;
  rclcpp::Time baseline_time_;

  bool baseline_pose_set_{false};
  std::string plugin_name_;
};
}  // namespace nav2_controller

#endif  // CUSTOM_PURE_PURSUIT_CONTROLLER__PLUGINS__PROGRESS_CHECKER_HPP_
