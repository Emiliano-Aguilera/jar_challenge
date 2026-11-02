#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
namespace equipo_jar_navegacion {
struct Vec {
  double x = 0, y = 0;
};
struct Pose {
  double x = 0, y = 0, yaw = 0;
};
struct Velocity {
  double x = 0, y = 0, w = 0;
};  // Body-frame velocity.
struct Body {
  double min_x, max_x, min_y, max_y;
};
inline double norm(Vec p) { return std::hypot(p.x, p.y); }
inline Vec operator+(Vec a, Vec b) { return {a.x + b.x, a.y + b.y}; }
inline Vec operator-(Vec a, Vec b) { return {a.x - b.x, a.y - b.y}; }
inline Vec operator*(Vec a, double s) { return {a.x * s, a.y * s}; }
inline double dot(Vec a, Vec b) { return a.x * b.x + a.y * b.y; }
inline double angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline Vec position(Pose p) { return {p.x, p.y}; }
inline double radius(Body b) {
  return std::hypot(std::max(std::abs(b.min_x), std::abs(b.max_x)),
                    std::max(std::abs(b.min_y), std::abs(b.max_y)));
}
inline bool finite(Pose p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);
}
inline bool finite(Velocity p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.w);
}
struct Grid {
  unsigned width = 0, height = 0;
  double resolution = 0, origin_x = 0, origin_y = 0;
  std::vector<unsigned char> data;
  Grid() = default;
  Grid(unsigned w, unsigned h, double r, double x, double y, std::vector<unsigned char> d)
      : width(w), height(h), resolution(r), origin_x(x), origin_y(y), data(std::move(d)) {
    if (!valid()) throw std::invalid_argument("Invalid costmap");
  }
  bool valid() const {
    return width > 0 && height > 0 && data.size() == size_t(width) * height &&
           std::isfinite(resolution) && resolution > 0 && std::isfinite(origin_x) &&
           std::isfinite(origin_y);
  }
  int ix(double x) const { return int(std::floor((x - origin_x) / resolution)); }
  int iy(double y) const { return int(std::floor((y - origin_y) / resolution)); }
  bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < int(width) && y < int(height); }
  bool obstacle(int x, int y) const { return !inside(x, y) || data[y * width + x] >= 254; }
};
inline bool valid(Body b) {
  return std::isfinite(b.min_x) && std::isfinite(b.max_x) && std::isfinite(b.min_y) &&
         std::isfinite(b.max_y) && b.min_x < 0 && b.max_x > 0 && b.min_y < 0 && b.max_y > 0;
}
inline std::array<Vec, 4> corners(Body b, Pose p, double extra) {
  std::array<Vec, 4> a{{{b.min_x - extra, b.min_y - extra},
                        {b.max_x + extra, b.min_y - extra},
                        {b.max_x + extra, b.max_y + extra},
                        {b.min_x - extra, b.max_y + extra}}};
  double c = std::cos(p.yaw), s = std::sin(p.yaw);
  for (auto& v : a) v = {p.x + c * v.x - s * v.y, p.y + s * v.x + c * v.y};
  return a;
}
inline bool overlap(const std::array<Vec, 4>& a, const std::array<Vec, 4>& b) {
  // SAT includes touching and occupied cell interiors, not just footprint edges.
  for (const auto* poly : {&a, &b})
    for (unsigned i = 0; i < 4; i++) {
      Vec e = (*poly)[(i + 1) % 4] - (*poly)[i], axis{-e.y, e.x};
      double amin = INFINITY, amax = -INFINITY, bmin = INFINITY, bmax = -INFINITY;
      for (auto v : a) {
        double d = dot(v, axis);
        amin = std::min(amin, d);
        amax = std::max(amax, d);
      }
      for (auto v : b) {
        double d = dot(v, axis);
        bmin = std::min(bmin, d);
        bmax = std::max(bmax, d);
      }
      if (amax < bmin - 1e-12 || bmax < amin - 1e-12) return false;
    }
  return true;
}
inline bool safe_pose(const Grid& g, Body body, Pose p, double extra) {
  if (!g.valid() || !valid(body) || !finite(p) || !std::isfinite(extra) || extra < 0) return false;
  auto a = corners(body, p, extra);
  double xmin = INFINITY, xmax = -INFINITY, ymin = INFINITY, ymax = -INFINITY;
  for (auto v : a) {
    xmin = std::min(xmin, v.x);
    xmax = std::max(xmax, v.x);
    ymin = std::min(ymin, v.y);
    ymax = std::max(ymax, v.y);
  }
  int x0 = g.ix(xmin - 1e-10), x1 = g.ix(xmax + 1e-10), y0 = g.iy(ymin - 1e-10),
      y1 = g.iy(ymax + 1e-10);
  if (!g.inside(x0, y0) || !g.inside(x1, y1)) return false;
  for (int y = y0; y <= y1; y++)
    for (int x = x0; x <= x1; x++)
      if (g.obstacle(x, y)) {
        double lo = g.origin_x + x * g.resolution, bo = g.origin_y + y * g.resolution,
               hi = lo + g.resolution, to = bo + g.resolution;
        if (overlap(a, {{{lo, bo}, {hi, bo}, {hi, to}, {lo, to}}})) return false;
      }
  return true;
}
inline bool turn_safe(const Grid& g, Body b, Vec p, double extra) {
  if (!g.valid() || !valid(b) || !std::isfinite(p.x) || !std::isfinite(p.y) ||
      !std::isfinite(extra) || extra < 0)
    return false;
  double r = std::hypot(std::max(std::abs(b.min_x), std::abs(b.max_x)) + extra,
                        std::max(std::abs(b.min_y), std::abs(b.max_y)) + extra);
  int x0 = g.ix(p.x - r - 1e-10), x1 = g.ix(p.x + r + 1e-10), y0 = g.iy(p.y - r - 1e-10),
      y1 = g.iy(p.y + r + 1e-10);
  if (!g.inside(x0, y0) || !g.inside(x1, y1)) return false;
  for (int y = y0; y <= y1; y++)
    for (int x = x0; x <= x1; x++)
      if (g.obstacle(x, y)) {
        double lo = g.origin_x + x * g.resolution, bo = g.origin_y + y * g.resolution;
        double dx = std::max({lo - p.x, 0., p.x - lo - g.resolution}),
               dy = std::max({bo - p.y, 0., p.y - bo - g.resolution});
        if (std::hypot(dx, dy) <= r) return false;
      }
  return true;
}
inline bool safe_segment(const Grid& g, Body b, Pose a, Pose z, double extra) {
  if (!g.valid() || !finite(a) || !finite(z)) return false;
  double travel = norm(position(z) - position(a)), rotation = angle(z.yaw - a.yaw);
  double bound = travel + radius(b) * std::abs(rotation);
  int n = std::max(1, int(std::ceil(bound / (g.resolution * .25))));
  if (n > 10000) return false;
  for (int i = 0; i <= n; i++) {
    double t = double(i) / n;
    if (!safe_pose(g, b, {a.x + (z.x - a.x) * t, a.y + (z.y - a.y) * t, a.yaw + rotation * t},
                   extra + bound / (2 * n)))
      return false;
  }
  return true;
}
inline Pose advance(Pose p, Velocity v, double dt) {
  // Exact integration of a constant body-frame planar twist.
  double dx, dy;
  if (std::abs(v.w) < 1e-9) {
    dx = v.x * dt;
    dy = v.y * dt;
  } else {
    double s = std::sin(v.w * dt), c = std::cos(v.w * dt);
    dx = (v.x * s + v.y * (c - 1)) / v.w;
    dy = (v.x * (1 - c) + v.y * s) / v.w;
  }
  return {p.x + std::cos(p.yaw) * dx - std::sin(p.yaw) * dy,
          p.y + std::sin(p.yaw) * dx + std::cos(p.yaw) * dy, angle(p.yaw + v.w * dt)};
}
inline bool safe_stop(const Grid& g, Body b, Pose p, Velocity v, double extra, double reaction,
                      double decel) {
  if (!finite(v) || !std::isfinite(reaction) || reaction < 0 || !std::isfinite(decel) || decel <= 0)
    return false;
  // Conservative constant-twist envelope: retain full velocity throughout reaction
  // and stopping time rather than assuming instant response to the new command.
  double duration = reaction + std::max(std::hypot(v.x, v.y), std::abs(v.w) * radius(b)) / decel;
  if (duration > 10) return false;
  int n = std::max(1, int(std::ceil(duration / .025)));
  for (int i = 0; i < n; i++) {
    Pose next = advance(p, v, duration / n);
    if (!safe_segment(g, b, p, next, extra)) return false;
    p = next;
  }
  return true;
}
}  // namespace equipo_jar_navegacion
