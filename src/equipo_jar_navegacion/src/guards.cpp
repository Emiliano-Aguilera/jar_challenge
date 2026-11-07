#include <chrono>
#include <nav2_behaviors/plugins/back_up.hpp>
#include <nav2_behaviors/plugins/spin.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_srvs/srv/trigger.hpp>
namespace equipo_jar_navegacion {
// Every recovery asks for current permission. A retained old "unlocked" message
// cannot authorize a turn after the controller entered the passage.
class RecoveryPermit {
 public:
  void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
    node_ = node;
    if (!node_->has_parameter("passage_guard_timeout_s"))
      node_->declare_parameter("passage_guard_timeout_s", .1);
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    client_ = node_->create_client<std_srvs::srv::Trigger>(
        "/jar/pasaje_permitir_recuperacion", rmw_qos_profile_services_default, group_);
    callback_ =
        node_->add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter>& params) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const auto& p : params)
            if (p.get_name() == "passage_guard_timeout_s") {
              try {
                double v = p.as_double();
                if (!std::isfinite(v) || v <= 0 || v > .5) {
                  result.successful = false;
                  result.reason = "Recovery permission timeout must be in (0, 0.5]";
                }
              } catch (const std::exception& e) {
                result.successful = false;
                result.reason = e.what();
              }
            }
          return result;
        });
  }
  bool allowed() {
    if (!client_ || !client_->service_is_ready()) return false;
    double timeout = node_->get_parameter("passage_guard_timeout_s").as_double();
    if (!std::isfinite(timeout) || timeout <= 0 || timeout > .5) return false;
    auto future = client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(std::chrono::duration<double>(timeout)) != std::future_status::ready) {
      client_->remove_pending_request(future);
      return false;
    }
    return future.get()->success;
  }

 private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_;
};
class GuardedSpin : public nav2_behaviors::Spin {
 public:
  void onConfigure() override {
    nav2_behaviors::Spin::onConfigure();
    permit_.configure(node_.lock());
  }
  nav2_behaviors::Status onRun(
      const std::shared_ptr<const nav2_msgs::action::Spin::Goal> command) override {
    if (!permit_.allowed()) {
      stopRobot();
      return nav2_behaviors::Status::FAILED;
    }
    return nav2_behaviors::Spin::onRun(command);
  }
  nav2_behaviors::Status onCycleUpdate() override {
    if (!permit_.allowed()) {
      stopRobot();
      return nav2_behaviors::Status::FAILED;
    }
    return nav2_behaviors::Spin::onCycleUpdate();
  }

 private:
  RecoveryPermit permit_;
};
class GuardedBackUp : public nav2_behaviors::BackUp {
 public:
  void onConfigure() override {
    nav2_behaviors::BackUp::onConfigure();
    permit_.configure(node_.lock());
  }
  nav2_behaviors::Status onRun(
      const std::shared_ptr<const nav2_msgs::action::BackUp::Goal> command) override {
    if (!permit_.allowed()) {
      stopRobot();
      return nav2_behaviors::Status::FAILED;
    }
    return nav2_behaviors::BackUp::onRun(command);
  }
  nav2_behaviors::Status onCycleUpdate() override {
    if (!permit_.allowed()) {
      stopRobot();
      return nav2_behaviors::Status::FAILED;
    }
    return nav2_behaviors::BackUp::onCycleUpdate();
  }

 private:
  RecoveryPermit permit_;
};
}  // namespace equipo_jar_navegacion
PLUGINLIB_EXPORT_CLASS(equipo_jar_navegacion::GuardedSpin, nav2_core::Behavior)
PLUGINLIB_EXPORT_CLASS(equipo_jar_navegacion::GuardedBackUp, nav2_core::Behavior)
