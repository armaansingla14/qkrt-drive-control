#include "qkrt/planners.hpp"
#include "test_harness.hpp"

using namespace qkrt;

namespace {
OccupancyGrid wallMap() {
  // 40x40 m at 0.5 m; vertical wall at x = 10 m with a gap at the top.
  OccupancyGrid g(80, 80, 0.5);
  g.fillRect(20, 0, 21, 65);
  return g;
}

bool pathCollisionFree(const OccupancyGrid& g, const std::vector<Vec2>& p) {
  for (std::size_t i = 1; i < p.size(); ++i)
    if (!g.segmentFree(p[i - 1], p[i])) return false;
  return true;
}
}  // namespace

TEST_CASE("grid coordinate conversion") {
  OccupancyGrid g(10, 10, 0.5, {-2.5, -2.5});
  const Cell c = g.worldToCell({0.1, -0.1});
  CHECK(c.x == 5 && c.y == 4);
  CHECK_NEAR(g.cellCenter(c).x, 0.25, 1e-12);
  CHECK(g.occupied(-1, 0));
}

TEST_CASE("inflation grows obstacles") {
  OccupancyGrid g(21, 21, 0.1);
  g.set(10, 10, 100);
  const OccupancyGrid inf = g.inflated(0.3);
  CHECK(inf.occupied(13, 10));
  CHECK(!inf.occupied(14, 10));
  CHECK(!inf.occupied(13, 13));
}

TEST_CASE("astar straight line in empty map is optimal") {
  OccupancyGrid g(50, 50, 1.0);
  const PlanResult r = planAStar(g, {0.5, 0.5}, {40.5, 0.5});
  CHECK(r.success);
  CHECK_NEAR(r.length, 40.0, 1e-9);
}

TEST_CASE("astar diagonal uses octile cost") {
  OccupancyGrid g(50, 50, 1.0);
  const PlanResult r = planAStar(g, {0.5, 0.5}, {30.5, 30.5});
  CHECK(r.success);
  CHECK_NEAR(r.length, 30.0 * std::sqrt(2.0), 1e-9);
}

TEST_CASE("astar routes around a wall and fails when blocked") {
  OccupancyGrid g = wallMap();
  const PlanResult r = planAStar(g, {2.0, 2.0}, {38.0, 2.0});
  CHECK(r.success);
  CHECK(r.length > 60.0);
  CHECK(pathCollisionFree(g, r.path));
  g.fillRect(20, 0, 21, 79);
  CHECK(!planAStar(g, {2.0, 2.0}, {38.0, 2.0}).success);
}

TEST_CASE("rrt finds collision-free path") {
  const OccupancyGrid g = wallMap();
  RrtConfig cfg;
  cfg.max_iterations = 8000;
  cfg.seed = 7;
  const PlanResult r = planRrt(g, {2.0, 2.0}, {38.0, 2.0}, cfg);
  CHECK(r.success);
  CHECK(pathCollisionFree(g, r.path));
  CHECK_NEAR(r.path.front().x, 2.0, 1e-12);
  CHECK_NEAR(r.path.back().x, 38.0, 1e-12);
}

TEST_CASE("rrt* is not longer than rrt on average") {
  OccupancyGrid g(80, 80, 0.5);
  g.fillRect(30, 20, 50, 60);
  double rrt = 0, star = 0;
  for (unsigned s = 1; s <= 5; ++s) {
    RrtConfig cfg;
    cfg.seed = s;
    cfg.max_iterations = 3000;
    const PlanResult a = planRrt(g, {2, 20}, {38, 20}, cfg);
    cfg.star = true;
    const PlanResult b = planRrt(g, {2, 20}, {38, 20}, cfg);
    CHECK(a.success && b.success);
    CHECK(pathCollisionFree(g, b.path));
    rrt += a.length;
    star += b.length;
  }
  CHECK(star < rrt);
}

TEST_CASE("shortcut keeps path valid and never longer") {
  const OccupancyGrid g = wallMap();
  const PlanResult r = planAStar(g, {2.0, 2.0}, {38.0, 2.0});
  const auto s = shortcutPath(g, r.path);
  CHECK(pathCollisionFree(g, s));
  CHECK(pathLength(s) <= r.length + 1e-9);
  CHECK(s.size() < r.path.size());
}

TEST_CASE("pure pursuit drives a kinematic unicycle to the goal") {
  PurePursuit pp(1.0, 1.0, 0.2);
  pp.setPath({{0, 0}, {10, 0}, {10, 10}});
  Pose2 pose{0, 0, 0};
  double max_cte = 0.0;
  for (int i = 0; i < 5000 && !pp.done(); ++i) {
    const Twist2 cmd = pp.compute(pose);
    pose = integrateArc(pose, cmd.v, cmd.w, 0.01);
    max_cte = std::max(max_cte, pp.crossTrackError(pose));
  }
  CHECK(pp.done());
  CHECK(distance(pose.position(), {10, 10}) < 0.25);
  CHECK(max_cte < 1.0);
}

TEST_MAIN()
