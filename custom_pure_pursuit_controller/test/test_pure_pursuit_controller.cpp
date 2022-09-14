#include <math.h>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "custom_pure_pursuit_controller/pure_pursuit_controller.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class BasicAPIPurePursuitController : public custom_pure_pursuit_controller::PurePursuitController
{
public:
  BasicAPIPurePursuitController()
  : custom_pure_pursuit_controller::PurePursuitController() {}

  nav_msgs::msg::Path getPlan() {return global_plan_;}

  double getSpeed() {return desired_linear_vel_;}
};

TEST(PurePursuitControllerTest, basicAPI)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");

  // instantiate
  auto ctrl = std::make_shared<BasicAPIPurePursuitController>();
  ctrl->configure(node, name, tf, costmap);
  ctrl->activate();
  ctrl->deactivate();
  ctrl->cleanup();

  // setPlan and get plan
  nav_msgs::msg::Path path;
  path.poses.resize(2);
  path.poses[0].header.frame_id = "fake_frame";
  ctrl->setPlan(path);
  EXPECT_EQ(ctrl->getPlan().poses.size(), 2ul);
  EXPECT_EQ(ctrl->getPlan().poses[0].header.frame_id, std::string("fake_frame"));

  // set speed limit
  const double base_speed = ctrl->getSpeed();
  EXPECT_EQ(ctrl->getSpeed(), base_speed);
  ctrl->setSpeedLimit(0.51, false);
  EXPECT_EQ(ctrl->getSpeed(), 0.51);
  ctrl->setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);
  EXPECT_EQ(ctrl->getSpeed(), base_speed);
  ctrl->setSpeedLimit(30, true);
  EXPECT_EQ(ctrl->getSpeed(), base_speed * 0.3);
  ctrl->setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, true);
  EXPECT_EQ(ctrl->getSpeed(), base_speed);
}
