#include <tf2/utils.h>

#include <chrono>
#include <functional>
#include <mutex>
#include <nav2_core/controller.hpp>
#include <nav2_core/exceptions.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav2_util/robot_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "equipo_jar_navegacion/passage.hpp"
#include "equipo_jar_navegacion/shared_state.hpp"
namespace equipo_jar_navegacion {
std::shared_ptr<SharedState> shared_state(const std::string& node) {
  static std::mutex mutex;
  static std::map<std::string, std::shared_ptr<SharedState>> entries;
  std::lock_guard<std::mutex> lock(mutex);
  auto& entry = entries[node];
  if (!entry) entry = std::make_shared<SharedState>();
  return entry;
}
using Steady = std::chrono::steady_clock;
inline double monotonic() {
  return std::chrono::duration<double>(Steady::now().time_since_epoch()).count();
}
inline Pose unpack(const geometry_msgs::msg::Pose& p) {
  return {p.position.x, p.position.y, tf2::getYaw(p.orientation)};
}
inline Velocity unpack(const geometry_msgs::msg::Twist& v) {
  return {v.linear.x, v.linear.y, v.angular.z};
}
// Keep parameter names and limits in one table; callbacks reject unsafe/in-flight edits.
#define PASSAGE_DOUBLES(X)                                                                       \
  X(extra_margin)                                                                                \
  X(lookahead)                                                                                   \
  X(center_range)                                                                                \
  X(detect_width)                                                                                \
  X(max_v)                                                                                       \
  X(max_lateral)                                                                                 \
  X(max_correction_w)                                                                            \
  X(align_w) X(heading_tolerance) X(heading_abort) X(position_tolerance) X(min_v) X(min_align_w) \
      X(linear_gain) X(yaw_gain) X(track_lookahead) X(still_speed) X(still_w) X(still_dwell)     \
          X(reaction) X(deceleration) X(max_duration) X(progress_timeout) X(progress_distance)   \
              X(grid_max_age) X(scan_max_age) X(tf_max_age)
class PassageController : public nav2_core::Controller {
 public:
  PassageController() : loader_("nav2_core", "nav2_core::Controller") {}
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override {
    node_ = parent.lock();
    if (!node_) throw std::runtime_error("Controller parent expired");
    prefix_ = name + ".passage.";
    tf_ = tf;
    local_ = map;
    shared_ = shared_state(std::string(node_->get_namespace()) + "/" + node_->get_name());
    auto declare = [&](const std::string& key, auto value) {
      if (!node_->has_parameter(prefix_ + key)) node_->declare_parameter(prefix_ + key, value);
    };
    declare("enabled", config_.enabled);
    config_.enabled = node_->get_parameter(prefix_ + "enabled").as_bool();
#define LOAD(field)               \
  declare(#field, config_.field); \
  config_.field = node_->get_parameter(prefix_ + #field).as_double();
    PASSAGE_DOUBLES(LOAD)
#undef LOAD
    if (!config_.valid()) throw std::runtime_error("Invalid passage parameters");
    inner_ = loader_.createSharedInstance("nav2_mppi_controller::MPPIController");
    inner_->configure(parent, name, tf, map);
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    grid_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
        "/global_costmap/costmap_raw", qos, [this](nav2_msgs::msg::Costmap::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          global_ = msg;
          grid_received_ = monotonic();
        });
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          scan_stamp_ = msg->header.stamp;
          scan_received_ = monotonic();
        });
    status_ = node_->create_publisher<std_msgs::msg::String>("/jar/pasaje_estado", qos);
    permit_ = node_->create_service<std_srvs::srv::Trigger>(
        "/jar/pasaje_permitir_recuperacion",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          std::lock_guard<std::mutex> lock(mutex_);
          response->success = false;
          try {
            if (active_ && !passage_.locked()) {
              if (!config_.enabled)
                response->success = true;
              else {
                auto snapshot = world();
                auto p = current();
                auto b = body();
                response->success = turn_safe(snapshot, b, position(p), config_.extra_margin) &&
                                    local_turn_safe(p, b);
              }
            }
          } catch (const std::exception& e) {
            response->message = e.what();
          }
          if (response->message.empty())
            response->message = response->success
                                    ? "recovery allowed"
                                    : "passage locked, inactive or turning area unavailable";
        });
    callback_ =
        node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& values) {
          // MPPI declares parameters during activate(): unrelated declarations must
          // not reenter our already-held lifecycle/control mutex.
          bool relevant = std::any_of(values.begin(), values.end(), [this](const auto& p) {
            return p.get_name().rfind(prefix_, 0) == 0;
          });
          if (!relevant) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
          }
          // Do not wait while rclcpp owns its parameter mutex: lifecycle cleanup
          // can hold our mutex while unregistering MPPI's parameter callback.
          std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
          if (!lock.owns_lock()) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = false;
            result.reason = "Controller busy; retry parameter update while idle";
            return result;
          }
          Config proposed = config_;
          bool change = false;
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = false;
          try {
            for (const auto& p : values) {
              if (p.get_name().rfind(prefix_, 0) != 0) continue;
              change = true;
              std::string key = p.get_name().substr(prefix_.size());
              if (key == "enabled") proposed.enabled = p.as_bool();
#define UPDATE(field) else if (key == #field) proposed.field = p.as_double();
              PASSAGE_DOUBLES(UPDATE)
#undef UPDATE
              else {
                result.reason = "Unknown passage parameter";
                return result;
              }
            }
            if (change && (passage_.locked() || !proposed.valid())) {
              result.reason =
                  "Passage parameters require idle unlocked control and valid safety limits";
              return result;
            }
            config_ = proposed;
            result.successful = true;
            return result;
          } catch (const std::exception& e) {
            result.reason = e.what();
            return result;
          }
        });
  }
  void activate() override {
    std::lock_guard<std::mutex> lock(mutex_);
    inner_->activate();
    status_->on_activate();
    active_ = true;
    publish();
  }
  void deactivate() override {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    inner_->deactivate();
    status_->on_deactivate();
  }
  void cleanup() override {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    inner_->cleanup();
    inner_.reset();
    callback_.reset();
    grid_sub_.reset();
    scan_sub_.reset();
    permit_.reset();
    status_.reset();
  }
  void setPlan(const nav_msgs::msg::Path& plan) override {
    std::lock_guard<std::mutex> lock(mutex_);
    plan_ = plan;
    inner_->setPlan(plan);
    // Replanning/cancellation is not evidence that the rear has left the passage.
  }
  void setSpeedLimit(const double& limit, const bool& percentage) override {
    std::lock_guard<std::mutex> lock(mutex_);
    inner_->setSpeedLimit(limit, percentage);
    speed_cap_ = limit == 0 ? INFINITY : (percentage ? config_.max_v * limit / 100. : limit);
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped& input, const geometry_msgs::msg::Twist& velocity,
      nav2_core::GoalChecker* checker) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) throw nav2_core::PlannerException("Passage controller inactive");
    if (!config_.enabled) return inner_->computeVelocityCommands(input, velocity, checker);
    try {
      auto snapshot = world();
      Pose pose = current();
      Body b = body();
      if (passage_.locked() && locked_body_ &&
          (std::abs(b.min_x - locked_body_->min_x) > 1e-4 ||
           std::abs(b.max_x - locked_body_->max_x) > 1e-4 ||
           std::abs(b.min_y - locked_body_->min_y) > 1e-4 ||
           std::abs(b.max_y - locked_body_->max_y) > 1e-4))
        throw std::runtime_error("Footprint changed during passage");
      auto measured = unpack(velocity);
      if (!finite(measured)) throw std::runtime_error("Invalid odometry velocity");
      auto before = passage_.phase();
      if (!passage_.locked()) {
        std::vector<Vec> points;
        points.reserve(plan_.poses.size());
        for (const auto& p : plan_.poses) {
          geometry_msgs::msg::PoseStamped q;
          if (!nav2_util::transformPoseInTargetFrame(p, q, *tf_, "map", config_.tf_max_age))
            throw std::runtime_error("Plan transform unavailable");
          points.push_back(position(unpack(q.pose)));
        }
        auto detection = detect(snapshot, b, points, pose, config_);
        if (detection.crossing) {
          if (!safe_stop(snapshot, b, pose, measured, config_.extra_margin, config_.reaction,
                         config_.deceleration) ||
              !passage_.begin(*detection.crossing, snapshot, b, pose, config_, monotonic())) {
            if (!turn_safe(snapshot, b, position(pose), config_.extra_margin))
              passage_.hold("already inside unchecked narrow passage");
            throw std::runtime_error("cannot stop and align safely before narrow passage");
          }
        } else if (detection.seen) {
          if (!turn_safe(snapshot, b, position(pose), config_.extra_margin))
            passage_.hold("already inside unchecked narrow passage");
          throw std::runtime_error("narrow passage has no checked fixed-heading exit");
        }
      }
      shared_->locked = passage_.locked();
      if (!passage_.locked()) {
        locked_body_.reset();
        return inner_->computeVelocityCommands(input, velocity, checker);
      }
      if (!locked_body_) locked_body_ = b;
      checker->reset();
      Config limits = config_;
      limits.max_v = std::min(limits.max_v, speed_cap_);
      limits.max_lateral = std::min(limits.max_lateral, limits.max_v);
      if (limits.max_v <= 0 || !limits.valid()) passage_.hold("speed limit stops passage");
      auto command = passage_.tick(snapshot, pose, measured, limits, monotonic());
      if (passage_.locked() && passage_.phase() != Phase::HOLD &&
          !local_motion_safe(pose, b, measured, command))
        passage_.hold("local sensor footprint or braking envelope obstructed");
      shared_->locked = passage_.locked();
      if (before != passage_.phase()) publish();
      if (passage_.phase() == Phase::HOLD) throw std::runtime_error(passage_.reason());
      geometry_msgs::msg::TwistStamped out;
      out.header.stamp = node_->now();
      out.header.frame_id = local_->getBaseFrameID();
      out.twist.linear.x = command.x;
      out.twist.linear.y = command.y;
      out.twist.angular.z = command.w;
      return out;
    } catch (const std::exception& e) {
      // A data failure during passage must not let a later plan reset the lock.
      if (passage_.locked()) {
        passage_.hold(e.what());
        shared_->locked = true;
        publish();
      }
      throw nav2_core::PlannerException(std::string("Narrow passage: ") + e.what());
    }
  }

 private:
  Body body() const {
    auto polygon = local_->getRobotFootprint();
    if (polygon.size() != 4)
      throw std::runtime_error("Passage mode requires a rectangular padded footprint");
    Body b{INFINITY, -INFINITY, INFINITY, -INFINITY};
    for (const auto& p : polygon) {
      b.min_x = std::min(b.min_x, p.x);
      b.max_x = std::max(b.max_x, p.x);
      b.min_y = std::min(b.min_y, p.y);
      b.max_y = std::max(b.max_y, p.y);
    }
    if (!valid(b)) throw std::runtime_error("Invalid published robot footprint");
    for (const auto& p : polygon)
      if (std::min(std::abs(p.x - b.min_x), std::abs(p.x - b.max_x)) > 1e-4 ||
          std::min(std::abs(p.y - b.min_y), std::abs(p.y - b.max_y)) > 1e-4)
        throw std::runtime_error("Footprint is not axis aligned in base frame");
    return b;
  }
  Grid world() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    double now = monotonic();
    if (!global_) throw std::runtime_error("Global costmap missing");
    if (now - grid_received_ > config_.grid_max_age)
      throw std::runtime_error("Global costmap wall age " + std::to_string(now - grid_received_));
    if (now - scan_received_ > config_.scan_max_age)
      throw std::runtime_error("LiDAR wall age " + std::to_string(now - scan_received_));
    if (!local_->isCurrent()) throw std::runtime_error("Local costmap is not current");
    double grid_age = (node_->now() - rclcpp::Time(global_->header.stamp)).seconds(),
           scan_age = (node_->now() - rclcpp::Time(scan_stamp_)).seconds();
    if (grid_age < 0 || grid_age > config_.grid_max_age || scan_age < 0 ||
        scan_age > config_.scan_max_age)
      throw std::runtime_error("Stale/future source stamp");
    const auto& m = global_->metadata;
    if (global_->header.frame_id != "map" || std::abs(tf2::getYaw(m.origin.orientation)) > 1e-6)
      throw std::runtime_error("Unexpected global costmap frame/origin");
    return Grid(m.size_x, m.size_y, m.resolution, m.origin.position.x, m.origin.position.y,
                global_->data);
  }
  Pose current() const {
    auto t = tf_->lookupTransform("map", local_->getBaseFrameID(), tf2::TimePointZero);
    double age = (node_->now() - rclcpp::Time(t.header.stamp)).seconds();
    if (age < 0 || age > config_.tf_max_age) throw std::runtime_error("Stale/future robot pose");
    Pose p{t.transform.translation.x, t.transform.translation.y, tf2::getYaw(t.transform.rotation)};
    if (!finite(p)) throw std::runtime_error("Invalid robot pose");
    return p;
  }
  std::pair<Grid, Pose> local_snapshot(Pose map_pose) const {
    geometry_msgs::msg::PoseStamped a, q;
    a.header.frame_id = "map";
    a.header.stamp = node_->now();
    a.pose.position.x = map_pose.x;
    a.pose.position.y = map_pose.y;
    a.pose.orientation.z = std::sin(map_pose.yaw / 2);
    a.pose.orientation.w = std::cos(map_pose.yaw / 2);
    if (!nav2_util::transformPoseInTargetFrame(a, q, *tf_, local_->getGlobalFrameID(),
                                               config_.tf_max_age))
      throw std::runtime_error("Local transform unavailable");
    auto raw = local_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*raw->getMutex());
    size_t count = size_t(raw->getSizeInCellsX()) * raw->getSizeInCellsY();
    return {Grid(raw->getSizeInCellsX(), raw->getSizeInCellsY(), raw->getResolution(),
                 raw->getOriginX(), raw->getOriginY(),
                 std::vector<unsigned char>(raw->getCharMap(), raw->getCharMap() + count)),
            unpack(q.pose)};
  }
  bool local_motion_safe(Pose p, Body b, Velocity actual, Velocity proposed) const {
    auto local = local_snapshot(p);
    return safe_stop(local.first, b, local.second, actual, config_.extra_margin, config_.reaction,
                     config_.deceleration) &&
           safe_stop(local.first, b, local.second, proposed, config_.extra_margin, config_.reaction,
                     config_.deceleration);
  }
  bool local_turn_safe(Pose p, Body b) const {
    auto l = local_snapshot(p);
    return turn_safe(l.first, b, position(l.second), config_.extra_margin);
  }
  void publish() {
    if (!status_ || !status_->is_activated()) return;
    std_msgs::msg::String m;
    m.data = std::string(phase_name(passage_.phase())) + ": " + passage_.reason();
    status_->publish(m);
    RCLCPP_INFO(node_->get_logger(), "Narrow passage %s", m.data.c_str());
  }
  pluginlib::ClassLoader<nav2_core::Controller> loader_;
  nav2_core::Controller::Ptr inner_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> local_;
  std::mutex mutex_, data_mutex_;
  Config config_;
  Passage passage_;
  std::optional<Body> locked_body_;
  std::shared_ptr<SharedState> shared_;
  std::string prefix_;
  nav_msgs::msg::Path plan_;
  bool active_ = false;
  double speed_cap_ = INFINITY;
  nav2_msgs::msg::Costmap::SharedPtr global_;
  builtin_interfaces::msg::Time scan_stamp_;
  double grid_received_ = 0, scan_received_ = 0;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr grid_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr permit_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_;
};
class PassageGoalChecker : public nav2_core::GoalChecker {
 public:
  PassageGoalChecker() : loader_("nav2_core", "nav2_core::GoalChecker") {}
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& name,
                  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override {
    auto node = parent.lock();
    if (!node) throw std::runtime_error("Goal checker parent expired");
    shared_ = shared_state(std::string(node->get_namespace()) + "/" + node->get_name());
    inner_ = loader_.createSharedInstance("nav2_controller::SimpleGoalChecker");
    inner_->initialize(parent, name, map);
  }
  void reset() override { inner_->reset(); }
  bool isGoalReached(const geometry_msgs::msg::Pose& q, const geometry_msgs::msg::Pose& g,
                     const geometry_msgs::msg::Twist& v) override {
    if (shared_->locked) {
      inner_->reset();
      return false;
    }
    return inner_->isGoalReached(q, g, v);
  }
  bool getTolerances(geometry_msgs::msg::Pose& p, geometry_msgs::msg::Twist& v) override {
    return inner_->getTolerances(p, v);
  }

 private:
  pluginlib::ClassLoader<nav2_core::GoalChecker> loader_;
  nav2_core::GoalChecker::Ptr inner_;
  std::shared_ptr<SharedState> shared_;
};
}  // namespace equipo_jar_navegacion
PLUGINLIB_EXPORT_CLASS(equipo_jar_navegacion::PassageController, nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(equipo_jar_navegacion::PassageGoalChecker, nav2_core::GoalChecker)
