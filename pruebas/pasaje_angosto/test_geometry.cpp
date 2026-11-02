#include <cassert>
#include <cmath>
#include <iostream>

#include "equipo_jar_navegacion/geometry.hpp"
using namespace equipo_jar_navegacion;
Grid room() { return Grid(160, 120, .025, -1., -1., std::vector<unsigned char>(160 * 120, 0)); }
void mark(Grid& g, double x0, double y0, double x1, double y1, unsigned char value = 254) {
  for (unsigned y = 0; y < g.height; y++)
    for (unsigned x = 0; x < g.width; x++) {
      double px = g.origin_x + (x + .5) * g.resolution, py = g.origin_y + (y + .5) * g.resolution;
      if (px >= x0 && px < x1 && py >= y0 && py < y1) g.data[y * g.width + x] = value;
    }
}
int main() {
  Body b{-.201, .139, -.12, .12};
  auto g = room();
  mark(g, -1., -.55, 3., -.50);
  mark(g, .70, -.125, .95, .125);
  assert(safe_pose(g, b, Pose{.8, -.31, 0}, .01));
  assert(!safe_pose(g, b, Pose{.8, -.31, .733}, .01));  // Turning consumes the available gap.
  assert(!safe_pose(g, b, Pose{.8, -.40, 0}, .01));     // Rear/side cell overlap, not center-only.
  assert(!safe_pose(g, b, Pose{-1., 0, 0}, 0));         // Outside is never assumed clear.
  assert(!safe_pose(g, b, Pose{NAN, 0, 0}, 0));
  auto unknown = room();
  mark(unknown, .1, -.1, .2, .1, 255);
  assert(!safe_pose(unknown, b, Pose{0, 0, 0}, 0));
  auto inflated = room();
  mark(inflated, .1, -.1, .2, .1, 253);
  assert(safe_pose(inflated, b, Pose{0, 0, 0}, 0));  // Do not inflate a full footprint twice.
  auto touching = room();
  mark(touching, -.225, -.1, -.2, .1);
  assert(!safe_pose(touching, b, Pose{.001, 0, 0}, 0));  // Exact contact at a cell's right edge.
  auto internal = room();
  mark(internal, 0., 0., .025, .025);
  assert(!safe_pose(internal, b, Pose{0, 0, 0}, 0));  // Filled interior, not edge-only checks.
  auto obstacle = room();
  mark(obstacle, .45, -.20, .475, .20);
  assert(!safe_segment(obstacle, b, Pose{0, 0, 0}, Pose{1, 0, 0}, .01));
  assert(!safe_stop(obstacle, b, Pose{.2, 0, 0}, Velocity{.6, 0, 0}, .01, .3, .3));
  assert(safe_stop(obstacle, b, Pose{0, 0, 0}, Velocity{.05, 0, 0}, .01, .3, .3));
  assert(!turn_safe(g, b, Vec{.8, -.31}, .01));
  assert(turn_safe(g, b, Vec{0, 0}, .01));
  Grid fine(400, 400, .005, -1., -1., std::vector<unsigned char>(400 * 400, 0));
  mark(fine, .255, -.005, .260, .005);
  assert(!turn_safe(fine, b, Vec{0, 0}, .02));  // Extra margin grows both corner axes.
  std::cout << "geometry safety checks passed\n";
}
