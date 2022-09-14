#ifndef CUSTOM_PURE_PURSUIT_CONTROLLER__PLUGINS__PROGRESS_CHECKER_HPP_
#define CUSTOM_PURE_PURSUIT_CONTROLLER__PLUGINS__PROGRESS_CHECKER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/progress_checker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "custom_msgs/msg/progress_status.hpp"

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
  /**
   * @brief Publishes the new progress status if different from current one
   * @param progress_status New progress status
   */
  void pub_progress_status_on_change(const uint8_t progress_status);
  /**
   * @brief Callback executed when message published on "/plan" topic
   * @param path_msg Topic's message (unused)
   */
  void plan_callback(const nav_msgs::msg::Path::SharedPtr path_msg);

  std::string plugin_name_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
      custom_msgs::msg::ProgressStatus>> progress_status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Logger logger_ {rclcpp::get_logger("ProgressChecker")};

  // Parameters
  double radius_;
  rclcpp::Duration time_allowance_{0, 0};

  // Variables
  geometry_msgs::msg::Pose2D baseline_pose_;
  rclcpp::Time baseline_time_;
  bool baseline_pose_set_{false};
  custom_msgs::msg::ProgressStatus progress_status_;
};
}  // namespace custom_pure_pursuit_controller

#endif  // CUSTOM_PURE_PURSUIT_CONTROLLER__PLUGINS__PROGRESS_CHECKER_HPP_
