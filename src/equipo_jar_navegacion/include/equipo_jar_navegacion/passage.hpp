#pragma once
#include <optional>
#include <string>

#include "geometry.hpp"
namespace equipo_jar_navegacion {
struct Config {
  bool enabled = false;
  double extra_margin = .02, lookahead = 1.4, center_range = .12, detect_width = .46;
  double heading_step = M_PI / 16;
  double max_v = .15, max_lateral = .05, max_correction_w = .08, align_w = .5;
  double heading_tolerance = .035, heading_abort = .12, position_tolerance = .02,
         track_lookahead = .08;
  double linear_gain = 2.0, yaw_gain = 2.0, min_v = .15, min_align_w = .5;
  double still_speed = .01, still_w = .02, still_dwell = .3;
  double command_reserve = .1;
  double reaction = .3, deceleration = .3, max_duration = 60, progress_timeout = 8,
         progress_distance = .03;
  double grid_max_age = 2.0, scan_max_age = .5, tf_max_age = .3;
  bool valid() const {
    for (double v : {command_reserve,
                     heading_step,
                     min_v,
                     min_align_w,
                     extra_margin,
                     lookahead,
                     center_range,
                     detect_width,
                     max_v,
                     max_lateral,
                     max_correction_w,
                     align_w,
                     heading_tolerance,
                     heading_abort,
                     linear_gain,
                     yaw_gain,
                     position_tolerance,
                     track_lookahead,
                     still_speed,
                     still_w,
                     still_dwell,
                     reaction,
                     deceleration,
                     max_duration,
                     progress_timeout,
                     progress_distance,
                     grid_max_age,
                     scan_max_age,
                     tf_max_age})
      if (!std::isfinite(v) || v <= 0) return false;
    return command_reserve >= .05 && command_reserve <= .5 && heading_step >= .05 &&
           heading_step <= M_PI / 4 && min_v <= max_v && min_align_w <= align_w &&
           extra_margin >= .01 && max_v <= .3 && max_lateral <= max_v && max_correction_w <= .2 &&
           align_w <= 1 && heading_tolerance < heading_abort && heading_abort <= .25 &&
           position_tolerance <= extra_margin && lookahead <= 2 && reaction >= .3 &&
           deceleration <= .3 && grid_max_age <= 2 && scan_max_age <= 1 && tf_max_age <= .5;
  }
};
struct Crossing {
  std::vector<Vec> points;
  double yaw = 0;
};
struct Detection {
  bool seen = false;
  std::optional<Crossing> crossing;
};
inline std::vector<Vec> resample(const std::vector<Vec>& path, Vec current, double length,
                                 double step) {
  if (path.size() < 2) return {};
  size_t first = 0;
  double nearest = INFINITY;
  for (size_t i = 0; i < path.size(); i++) {
    double d = norm(path[i] - current);
    if (d < nearest) {
      nearest = d;
      first = i;
    }
  }
  std::vector<Vec> out{path[first]};
  double total = 0;
  for (size_t i = first + 1; i < path.size(); i++) {
    Vec delta = path[i] - out.back();
    double distance = norm(delta);
    if (distance < 1e-6) continue;
    int n = std::max(1, int(std::ceil(distance / step)));
    Vec start = out.back();
    for (int j = 1; j <= n; j++) {
      Vec next = start + delta * (double(j) / n);
      total += norm(next - out.back());
      if (total > length) return out;
      out.push_back(next);
    }
  }
  return out;
}
inline bool paired_sides(const Grid& g, Body b, Vec p, double yaw, const Config& c) {
  Vec normal{-std::sin(yaw), std::cos(yaw)};
  double distances[2] = {0, 0};
  for (int side = 0; side < 2; side++)
    for (double d = g.resolution * .25; d <= c.detect_width; d += g.resolution * .25) {
      Vec q = p + normal * (side ? -d : d);
      int x = g.ix(q.x), y = g.iy(q.y);
      // A boundary or unknown is not evidence of a sensed two-sided corridor.
      if (!g.inside(x, y) || g.data[y * g.width + x] == 255) return false;
      if (g.data[y * g.width + x] == 254) {
        distances[side] = d;
        break;
      }
    }
  return distances[0] > 0 && distances[1] > 0 && distances[0] + distances[1] < c.detect_width &&
         !turn_safe(g, b, p, c.extra_margin);
}
inline Detection detect(const Grid& g, Body b, const std::vector<Vec>& path, Pose pose,
                        const Config& c) {
  Detection result;
  if (!c.valid() || !g.valid() || !finite(pose)) return result;
  auto points = resample(path, position(pose), c.lookahead, g.resolution * .5);
  if (points.size() < 3) return result;
  size_t narrow = 0;
  double yaw = 0;
  for (size_t i = 1; i + 1 < points.size(); i++) {
    Vec dir = points[std::min(i + 3, points.size() - 1)] - points[i > 3 ? i - 3 : 0];
    if (norm(dir) < 1e-6) continue;
    double heading = std::atan2(dir.y, dir.x);
    if (paired_sides(g, b, points[i], heading, c)) {
      narrow = i;
      yaw = heading;
      result.seen = true;
      break;
    }
  }
  if (!result.seen) return result;
  size_t first = narrow, last = narrow;
  while (first > 0 && !turn_safe(g, b, points[first], c.extra_margin)) first--;
  while (last + 1 < points.size() && !turn_safe(g, b, points[last], c.extra_margin)) last++;
  if (!turn_safe(g, b, points[first], c.extra_margin) ||
      !turn_safe(g, b, points[last], c.extra_margin) || last <= first)
    return result;
  // The position path bends around an obstacle: its incoming tangent is not
  // necessarily the corridor axis. Search body headings across a full turn,
  // preferring the current orientation to avoid an unnecessary alignment turn.
  std::vector<double> headings{pose.yaw, yaw};
  int directions = int(std::ceil(M_PI / c.heading_step));
  for (int k = 1; k <= directions; k++) {
    headings.push_back(angle(pose.yaw + k * M_PI / directions));
    headings.push_back(angle(pose.yaw - k * M_PI / directions));
  }
  for (double heading : headings) {
    struct State {
      Vec point;
      double cost = INFINITY;
      int parent = -1;
    };
    Vec normal{-std::sin(heading), std::cos(heading)};
    int span = std::max(1, int(std::ceil(c.center_range / (g.resolution * .5))));
    std::vector<std::vector<State>> layers;
    bool reachable = true;
    for (size_t i = first; i <= last; i++) {
      std::vector<State> layer;
      auto candidates = lateral_centers(g, b, points[i], heading, c.extra_margin, c.center_range);
      // Keep zero/coarse alternatives for turn-safe endpoints and connections.
      for (int k = -span; k <= span; k++)
        candidates.push_back(points[i] + normal * (k * c.center_range / span));
      for (Vec p : candidates) {
        double shift = norm(p - points[i]);
        if (!safe_pose(g, b, {p.x, p.y, heading}, c.extra_margin)) continue;
        if ((i == first || i == last) && !turn_safe(g, b, p, c.extra_margin)) continue;
        State state{p};
        double offset_cost = std::abs(shift) * .05;
        if (layers.empty()) {
          if (safe_segment(g, b, {pose.x, pose.y, heading}, {p.x, p.y, heading}, c.extra_margin))
            state.cost = norm(p - position(pose)) + offset_cost;
        } else {
          // Keep connected alternatives. A greedy best point can be safe by
          // itself while its swept connection clips the corner of a box.
          const auto& previous = layers.back();
          for (size_t j = 0; j < previous.size(); j++) {
            double cost = previous[j].cost + norm(p - previous[j].point) + offset_cost;
            if (cost >= state.cost) continue;
            auto a = previous[j].point;
            if (!safe_segment(g, b, {a.x, a.y, heading}, {p.x, p.y, heading}, c.extra_margin))
              continue;
            state.cost = cost;
            state.parent = int(j);
          }
        }
        if (std::isfinite(state.cost)) layer.push_back(state);
      }
      if (layer.empty()) {
        reachable = false;
        break;
      }
      layers.push_back(std::move(layer));
    }
    if (!reachable || layers.size() < 2) continue;
    const auto& exits = layers.back();
    int index = int(std::min_element(exits.begin(), exits.end(),
                                     [](const auto& a, const auto& b) { return a.cost < b.cost; }) -
                    exits.begin());
    Crossing candidate;
    candidate.yaw = heading;
    for (size_t i = layers.size(); i-- > 0;) {
      const auto& state = layers[i][index];
      candidate.points.push_back(state.point);
      index = state.parent;
    }
    std::reverse(candidate.points.begin(), candidate.points.end());
    result.crossing = std::move(candidate);
    return result;
  }
  return result;
}

enum class Phase { NORMAL, BRAKE, ALIGN, TRACK, EXIT, HOLD };
inline const char* phase_name(Phase p) {
  switch (p) {
    case Phase::NORMAL:
      return "normal";
    case Phase::BRAKE:
      return "brake";
    case Phase::ALIGN:
      return "align";
    case Phase::TRACK:
      return "cross";
    case Phase::EXIT:
      return "exit";
    default:
      return "hold";
  }
}
class Passage {
 public:
  Phase phase() const { return phase_; }
  bool locked() const { return phase_ != Phase::NORMAL; }
  const std::string& reason() const { return reason_; }
  void hold(std::string reason) {
    phase_ = Phase::HOLD;
    reason_ = std::move(reason);
  }
  bool begin(Crossing crossing, const Grid& g, Body b, Pose p, const Config& c, double now) {
    if (locked() || crossing.points.size() < 2 || !turn_safe(g, b, position(p), c.extra_margin) ||
        !c.valid() || !std::isfinite(now))
      return false;
    Vec start = crossing.points.front();
    if (!safe_segment(g, b, {p.x, p.y, crossing.yaw}, {start.x, start.y, crossing.yaw},
                      c.extra_margin))
      return false;
    crossing_ = std::move(crossing);
    frozen_ = g;
    body_ = b;
    next_ = 0;
    started_ = progress_at_ = now;
    progress_pose_ = p;
    still_at_.reset();
    phase_ = Phase::BRAKE;
    reason_ = "checked fixed-heading passage";
    return true;
  }
  Velocity tick(const Grid& live, Pose p, Velocity actual, const Config& c, double now) {
    if (!locked() || phase_ == Phase::HOLD) return {};
    if (!c.valid() || !finite(p) || !finite(actual) || !std::isfinite(now) || now < started_) {
      hold("invalid input or clock");
      return {};
    }
    if (now - started_ > c.max_duration) {
      hold("passage time limit");
      return {};
    }
    if (norm(position(p) - position(progress_pose_)) >= c.progress_distance ||
        std::abs(angle(p.yaw - progress_pose_.yaw)) >= c.heading_tolerance) {
      progress_at_ = now;
      progress_pose_ = p;
    }
    if (now - progress_at_ > c.progress_timeout) {
      hold("no progress in passage");
      return {};
    }
    auto pose_safe = [&](Pose q) {
      return safe_pose(live, body_, q, c.extra_margin) &&
             safe_pose(frozen_, body_, q, c.extra_margin);
    };
    auto turning = [&]() {
      return turn_safe(live, body_, position(p), c.extra_margin) &&
             turn_safe(frozen_, body_, position(p), c.extra_margin);
    };
    auto stopping = [&](Velocity v, double reserve = 0.) {
      return safe_stop(live, body_, p, v, c.extra_margin, c.reaction + reserve, c.deceleration) &&
             safe_stop(frozen_, body_, p, v, c.extra_margin, c.reaction + reserve, c.deceleration);
    };
    if (!pose_safe(p) || !stopping(actual)) {
      hold("current footprint or braking envelope obstructed");
      return {};
    }
    bool still = std::hypot(actual.x, actual.y) <= c.still_speed && std::abs(actual.w) <= c.still_w;
    if (!still)
      still_at_.reset();
    else if (!still_at_)
      still_at_ = now;
    bool settled = still_at_ && now - *still_at_ >= c.still_dwell;
    Velocity command;
    double yaw_error = angle(crossing_.yaw - p.yaw);
    if (phase_ == Phase::BRAKE) {
      if (settled) {
        if (!turning())
          hold("not enough room to align before entry");
        else
          phase_ = Phase::ALIGN;
      }
      return {};
    }
    if (phase_ == Phase::ALIGN) {
      if (!turning()) {
        hold("alignment swept area obstructed");
        return {};
      }
      if (std::abs(yaw_error) <= c.heading_tolerance) {
        if (still) phase_ = Phase::TRACK;
        return {};
      }
      command.w = std::copysign(
          std::clamp(std::abs(c.yaw_gain * yaw_error), c.min_align_w, c.align_w), yaw_error);
    } else if (phase_ == Phase::TRACK) {
      if (std::abs(yaw_error) > c.heading_abort) {
        hold("heading escaped crossing tolerance");
        return {};
      }
      while (next_ + 1 < crossing_.points.size() &&
             norm(crossing_.points[next_] - position(p)) <= c.track_lookahead &&
             safe_segment(live, body_, p,
                          {crossing_.points[next_ + 1].x, crossing_.points[next_ + 1].y, p.yaw},
                          c.extra_margin) &&
             safe_segment(frozen_, body_, p,
                          {crossing_.points[next_ + 1].x, crossing_.points[next_ + 1].y, p.yaw},
                          c.extra_margin))
        next_++;
      Vec target = crossing_.points[next_], delta = target - position(p);
      if (next_ + 1 == crossing_.points.size() && norm(delta) <= c.position_tolerance &&
          turning()) {
        phase_ = Phase::EXIT;
        still_at_.reset();
        return {};
      }
      // The same-heading path may include small lateral translations around sensed
      // obstacles. Keep the body orientation; never rotate to face each point.
      Vec world = delta * c.linear_gain;
      double speed = norm(world);
      if (speed > 1e-9) world = world * (std::clamp(speed, c.min_v, c.max_v) / speed);
      command.x = std::cos(p.yaw) * world.x + std::sin(p.yaw) * world.y;
      command.y = -std::sin(p.yaw) * world.x + std::cos(p.yaw) * world.y;
      if (std::abs(command.y) > c.max_lateral) {
        double scale = c.max_lateral / std::abs(command.y);
        command.x *= scale;
        command.y *= scale;
      }
      command.w = std::clamp(c.yaw_gain * yaw_error, -c.max_correction_w, c.max_correction_w);
      // Slow for a bend/exit before its braking envelope reaches a wall. Keep
      // x/y in proportion: clipping only y changes the checked travel direction.
      for (unsigned attempt = 0; attempt < 20 && !stopping(command, c.command_reserve); attempt++) {
        command.x *= .8;
        command.y *= .8;
        command.w *= .8;
      }
    } else if (phase_ == Phase::EXIT) {
      if (!turning()) {
        hold("whole-body exit has no turning room");
        return {};
      }
      if (settled) {
        phase_ = Phase::NORMAL;
        reason_ = "whole body clear and stationary";
      }
      return {};
    }
    if (!stopping(command, phase_ == Phase::TRACK ? c.command_reserve : 0.)) {
      hold("proposed motion or braking envelope obstructed");
      return {};
    }
    return command;
  }

 private:
  Phase phase_ = Phase::NORMAL;
  std::string reason_;
  Crossing crossing_;
  Grid frozen_;
  Body body_{};
  size_t next_ = 0;
  double started_ = 0, progress_at_ = 0;
  Pose progress_pose_;
  std::optional<double> still_at_;
};
}  // namespace equipo_jar_navegacion
