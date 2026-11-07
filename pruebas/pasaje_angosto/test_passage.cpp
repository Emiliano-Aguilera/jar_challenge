#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "equipo_jar_navegacion/passage.hpp"
using namespace equipo_jar_navegacion;
Grid corridor(double half = .2) {
  Grid g(180, 100, .025, -1., -1.25, std::vector<unsigned char>(180 * 100, 0));
  for (unsigned y = 0; y < g.height; y++)
    for (unsigned x = 0; x < g.width; x++) {
      double px = g.origin_x + (x + .5) * g.resolution, py = g.origin_y + (y + .5) * g.resolution;
      if (px >= .35 && px <= 1.1 && std::abs(py) >= half) g.data[y * g.width + x] = 254;
    }
  return g;
}
int main() {
  Body b{-.201, .139, -.12, .12};
  Config cfg;
  // Recorded first-goal approach: the old tangent/greedy detector rejects every
  // candidate and the controller never leaves the start (run 41437278210).
  std::ifstream fixture(std::filesystem::path(__FILE__).parent_path() / "recorded_approach.txt");
  assert(fixture.good());
  unsigned w, h;
  double resolution, ox, oy;
  fixture >> w >> h >> resolution >> ox >> oy;
  Pose start;
  fixture >> start.x >> start.y >> start.yaw;
  size_t count;
  fixture >> count;
  std::vector<Vec> recorded_path(count);
  for (auto& point : recorded_path) fixture >> point.x >> point.y;
  fixture >> count;
  std::vector<unsigned char> cells;
  for (size_t i = 0; i < count; i++) {
    unsigned cost, length;
    fixture >> cost >> length;
    cells.insert(cells.end(), length, cost);
  }
  assert(!fixture.fail());
  Grid recorded(w, h, resolution, ox, oy, std::move(cells));
  auto recorded_route = detect(recorded, b, recorded_path, start, cfg);
  assert(recorded_route.seen && recorded_route.crossing);
  assert(std::abs(angle(recorded_route.crossing->yaw)) < .1);
  assert(recorded_route.crossing->points.back().x > 1.0);
  for (size_t i = 1; i < recorded_route.crossing->points.size(); i++) {
    auto a = recorded_route.crossing->points[i - 1], z = recorded_route.crossing->points[i];
    assert(safe_segment(recorded, b, {a.x, a.y, recorded_route.crossing->yaw},
                        {z.x, z.y, recorded_route.crossing->yaw}, cfg.extra_margin));
  }
  Pose reversed = start;
  reversed.yaw = M_PI;
  auto reversed_route = detect(recorded, b, recorded_path, reversed, cfg);
  assert(reversed_route.crossing);
  Passage replay;
  Pose replay_pose = start;
  Velocity replay_velocity;
  assert(replay.begin(*recorded_route.crossing, recorded, b, start, cfg, 0));
  for (unsigned i = 0; i < 1200 && replay.locked(); i++) {
    auto command = replay.tick(recorded, replay_pose, replay_velocity, cfg, i * .025);
    if (replay.phase() == Phase::HOLD)
      std::cerr << "recorded hold: " << replay.reason() << " pose=" << replay_pose.x << ","
                << replay_pose.y << "\n";
    assert(replay.phase() != Phase::HOLD);
    replay_pose = advance(replay_pose, command, .025);
    replay_velocity = command;
  }
  assert(!replay.locked() && replay_pose.x > 1.0);
  auto g = corridor();
  std::vector<Vec> line{{0, 0}, {1.6, 0}};
  auto d = detect(g, b, line, Pose{0, 0, 0}, cfg);
  assert(d.seen && d.crossing);
  assert(d.crossing->points.back().x > 1.2);
  assert(!detect(corridor(.4), b, line, Pose{0, 0, 0}, cfg).seen);
  assert(!detect(corridor(.1), b, line, Pose{0, 0, 0}, cfg).crossing);
  Passage p;
  Pose pose{0, 0, .2};
  Velocity v;
  double t = 0;
  assert(p.begin(*d.crossing, g, b, pose, cfg, t));
  assert(p.locked());
  bool crossed = false;
  for (unsigned i = 0; i < 900 && p.locked(); i++, t += .05) {
    auto cmd = p.tick(g, pose, v, cfg, t);
    if (p.phase() == Phase::HOLD)
      std::cerr << "unexpected hold: " << p.reason() << " x=" << pose.x << " yaw=" << pose.yaw
                << " t=" << t << "\n";
    assert(p.phase() != Phase::HOLD);
    if (p.phase() == Phase::ALIGN && std::abs(cmd.w) > 1e-9) assert(std::abs(cmd.w) >= .5 - 1e-9);
    if (p.phase() == Phase::TRACK) {
      crossed = true;
      assert(std::abs(cmd.w) <= cfg.max_correction_w + 1e-9);
    }
    pose = advance(pose, cmd, .05);
    v = cmd;
  }
  assert(crossed && !p.locked());
  assert(pose.x > 1.2);
  assert(turn_safe(g, b, position(pose), cfg.extra_margin));
  // New lethal obstacles must stop the passage, not trigger a recovery rotation.
  Passage blocked;
  pose = {0, 0, 0};
  v = {};
  t = 0;
  assert(blocked.begin(*d.crossing, g, b, pose, cfg, t));
  for (unsigned i = 0; i < 500 && blocked.locked(); i++, t += .05) {
    if (pose.x > .45)
      for (unsigned y = 0; y < g.height; y++)
        for (unsigned x = 0; x < g.width; x++) {
          double px = g.origin_x + (x + .5) * g.resolution;
          if (px > .75 && px < .85) g.data[y * g.width + x] = 254;
        }
    auto cmd = blocked.tick(g, pose, v, cfg, t);
    pose = advance(pose, cmd, .05);
    v = cmd;
    if (blocked.phase() == Phase::HOLD) {
      assert(norm({cmd.x, cmd.y}) == 0 && cmd.w == 0);
      break;
    }
  }
  assert(blocked.phase() == Phase::HOLD && blocked.locked());
  assert(pose.x < .75);
  // Clearing the live costmap cannot erase walls retained at passage entry.
  auto original = corridor();
  auto route = detect(original, b, line, Pose{0, 0, 0}, cfg);
  Passage retained;
  assert(retained.begin(*route.crossing, original, b, Pose{0, 0, 0}, cfg, 0));
  retained.tick(original, Pose{0, 0, 0}, Velocity{}, cfg, 0);
  retained.tick(original, Pose{0, 0, 0}, Velocity{}, cfg, .4);
  retained.tick(original, Pose{0, 0, 0}, Velocity{}, cfg, .5);
  Grid cleared = original;
  std::fill(cleared.data.begin(), cleared.data.end(), 0);
  auto stopped = retained.tick(cleared, Pose{.5, .10, 0}, Velocity{}, cfg, .6);
  assert(retained.phase() == Phase::HOLD && stopped.x == 0 && stopped.w == 0);
  // Cancellation/new plan cannot clear the latch; only verified exit can.
  blocked.hold("test cancellation");
  assert(blocked.locked());
  Config bad;
  bad.extra_margin = -.1;
  assert(!bad.valid());
  std::cout << "passage state and obstacle checks passed\n";
}
