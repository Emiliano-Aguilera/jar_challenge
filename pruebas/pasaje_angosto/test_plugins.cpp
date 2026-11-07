#include <cassert>
#include <chrono>
#include <iostream>
#include <nav2_behaviors/plugins/back_up.hpp>
#include <nav2_behaviors/plugins/spin.hpp>
#include <nav2_core/behavior.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_core/goal_checker.hpp>
#include <pluginlib/class_loader.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

#include "equipo_jar_navegacion/shared_state.hpp"
using namespace std::chrono_literals;
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("passage_test_controller");
  pluginlib::ClassLoader<nav2_core::Controller> controllers("nav2_core", "nav2_core::Controller");
  auto controller = controllers.createSharedInstance("equipo_jar_navegacion/PassageController");
  assert(controller);
  pluginlib::ClassLoader<nav2_core::GoalChecker> checkers("nav2_core", "nav2_core::GoalChecker");
  auto checker = checkers.createSharedInstance("equipo_jar_navegacion/PassageGoalChecker");
  checker->initialize(node, "probe", nullptr);
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.;
  geometry_msgs::msg::Twist v;
  auto state = equipo_jar_navegacion::shared_state(std::string(node->get_namespace()) + "/" +
                                                   node->get_name());
  assert(checker->isGoalReached(pose, pose, v));
  state->locked = true;
  assert(!checker->isGoalReached(pose, pose, v));
  checker->reset();
  assert(!checker->isGoalReached(pose, pose, v));
  state->locked = false;
  assert(checker->isGoalReached(pose, pose, v));
  auto behavior = std::make_shared<rclcpp_lifecycle::LifecycleNode>("passage_test_behavior");
  auto server = std::make_shared<rclcpp::Node>("passage_permission_fixture");
  auto service = server->create_service<std_srvs::srv::Trigger>(
      "/jar/pasaje_permitir_recuperacion",
      [](std::shared_ptr<std_srvs::srv::Trigger::Request>,
         std::shared_ptr<std_srvs::srv::Trigger::Response> r) { r->success = false; });
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node->get_node_base_interface());
  executor.add_node(behavior->get_node_base_interface());
  executor.add_node(server);
  std::thread spinning([&]() { executor.spin(); });
  auto tf = std::make_shared<tf2_ros::Buffer>(behavior->get_clock());
  behavior->declare_parameter("robot_base_frame", "base_footprint");
  behavior->declare_parameter("global_frame", "odom");
  behavior->declare_parameter("transform_tolerance", .1);
  behavior->declare_parameter("cycle_frequency", 10.);
  tf->setUsingDedicatedThread(true);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_footprint";
  transform.header.stamp = behavior->now();
  transform.transform.rotation.w = 1.;
  assert(tf->setTransform(transform, "fixture", true));
  pluginlib::ClassLoader<nav2_core::Behavior> behaviors("nav2_core", "nav2_core::Behavior");
  auto spin = behaviors.createSharedInstance("equipo_jar_navegacion/GuardedSpin");
  spin->configure(behavior, "spin", tf, nullptr);
  spin->activate();
  auto backup = behaviors.createSharedInstance("equipo_jar_navegacion/GuardedBackUp");
  backup->configure(behavior, "backup", tf, nullptr);
  backup->activate();
  std::this_thread::sleep_for(200ms);
  auto sg = std::make_shared<nav2_msgs::action::Spin::Goal>();
  sg->target_yaw = 1.;
  auto bg = std::make_shared<nav2_msgs::action::BackUp::Goal>();
  bg->target.x = -.2;
  bg->speed = .15;
  // Positive control: the original behavior can initialize at this valid TF.
  // Therefore an abort below cannot pass merely because TF was missing.
  nav2_behaviors::Spin baseline;
  baseline.configure(behavior, "baseline_spin", tf, nullptr);
  baseline.activate();
  assert(baseline.onRun(sg) == nav2_behaviors::Status::SUCCEEDED);
  baseline.deactivate();
  baseline.cleanup();
  auto real_spin = std::dynamic_pointer_cast<nav2_behaviors::Spin>(spin);
  auto real_backup = std::dynamic_pointer_cast<nav2_behaviors::BackUp>(backup);
  assert(real_spin->onRun(sg) == nav2_behaviors::Status::FAILED);
  assert(real_backup->onRun(bg) == nav2_behaviors::Status::FAILED);
  assert(real_spin->onCycleUpdate() == nav2_behaviors::Status::FAILED);
  // Missing service is also denied, never interpreted as unlocked.
  service.reset();
  std::this_thread::sleep_for(100ms);
  assert(real_spin->onRun(sg) == nav2_behaviors::Status::FAILED);
  spin->deactivate();
  backup->deactivate();
  spin->cleanup();
  backup->cleanup();
  executor.cancel();
  spinning.join();
  rclcpp::shutdown();
  std::cout << "Real plugin loading, goal lock and recovery denial checks passed\n";
}
