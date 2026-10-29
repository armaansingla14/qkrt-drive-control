// Planner success rate, path quality and run time across random occupancy maps.
#include <cstdio>
#include <random>

#include "bench_util.hpp"
#include "qkrt/planners.hpp"

using namespace qkrt;
using bench::Clock;

namespace {
struct Stats {
  int attempts = 0, success = 0;
  std::vector<double> ms, ratio, ratio_short;
};

bool pathValid(const OccupancyGrid& g, const std::vector<Vec2>& p) {
  for (std::size_t i = 1; i < p.size(); ++i)
    if (!g.segmentFree(p[i - 1], p[i])) return false;
  return !p.empty();
}
}  // namespace

int main(int argc, char** argv) {
  const std::string out = bench::outDir(argc, argv);
  const bool quick = bench::hasFlag(argc, argv, "--quick");
  const int maps_per_density = quick ? 10 : 100;
  const double densities[] = {0.15, 0.25, 0.35};
  const int W = 100, H = 100;
  const double res = 0.5, robot_radius = 0.3;

  bench::File csv(out + "/planners.csv"), md(out + "/planners.md");
  if (!csv.f || !md.f) return 1;
  std::fprintf(csv.f, "density,maps,solvable,planner,successes,success_rate_solvable,mean_ms,p95_ms,"
                      "mean_len_ratio_vs_astar,mean_len_ratio_after_shortcut\n");
  std::fprintf(md.f, "%d random %dx%d maps (%.1f m cells, %.0fx%.0f m) per density, obstacles inflated by %.1f m, "
                     "start/goal >= 20 m apart.\nLength ratio is relative to the 8-connected A* path on the same query "
                     "(A* row: shortcut A* vs raw A*). RRT and RRT* get 3000 iterations, 1 m step, 5 %% goal bias; RRT stops at the "
                     "first solution, RRT* spends the full budget.\n\n"
                     "| Obstacle density | Solvable queries | Planner | Success (of solvable) | Mean time [ms] | p95 time [ms] | Length / A* | Length / A* after shortcut |\n"
                     "|---|---|---|---|---|---|---|---|\n",
               maps_per_density, W, H, res, W * res, H * res, robot_radius);

  for (double density : densities) {
    Stats st[3];
    const char* names[3] = {"astar", "rrt", "rrt_star"};
    int solvable = 0;
    for (int m = 0; m < maps_per_density; ++m) {
      const unsigned seed = static_cast<unsigned>(density * 1000) * 1000 + m;
      const OccupancyGrid raw = OccupancyGrid::random(W, H, res, density, seed);
      const OccupancyGrid g = raw.inflated(robot_radius);
      std::mt19937 rng(seed);
      std::uniform_real_distribution<double> ux(0.0, W * res), uy(0.0, H * res);
      Vec2 s, goal;
      int guard = 0;
      do {
        s = {ux(rng), uy(rng)};
        goal = {ux(rng), uy(rng)};
      } while ((g.occupiedWorld(s) || g.occupiedWorld(goal) || distance(s, goal) < 20.0) && ++guard < 10000);

      auto t0 = Clock::now();
      const PlanResult a = planAStar(g, s, goal);
      const double astar_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
      if (!a.success) continue;  // unsolvable query: A* is complete on the grid
      st[0].ms.push_back(astar_ms);
      ++solvable;
      ++st[0].attempts;
      ++st[0].success;
      st[0].ratio.push_back(1.0);
      st[0].ratio_short.push_back(pathLength(shortcutPath(g, a.path)) / a.length);

      for (int k = 1; k <= 2; ++k) {
        RrtConfig cfg;
        cfg.seed = seed;
        cfg.star = (k == 2);
        cfg.max_iterations = 3000;
        cfg.step = 1.0;
        t0 = Clock::now();
        const PlanResult r = planRrt(g, s, goal, cfg);
        st[k].ms.push_back(std::chrono::duration<double, std::milli>(Clock::now() - t0).count());
        ++st[k].attempts;
        if (r.success && pathValid(g, r.path)) {
          ++st[k].success;
          st[k].ratio.push_back(r.length / a.length);
          st[k].ratio_short.push_back(pathLength(shortcutPath(g, r.path)) / a.length);
        }
      }
    }
    for (int k = 0; k < 3; ++k) {
      const double rate = st[k].attempts ? double(st[k].success) / st[k].attempts : 0.0;
      std::fprintf(csv.f, "%.2f,%d,%d,%s,%d,%.4f,%.3f,%.3f,%.4f,%.4f\n", density, maps_per_density, solvable, names[k],
                   st[k].success, rate, bench::mean(st[k].ms), bench::percentile(st[k].ms, 95), bench::mean(st[k].ratio),
                   bench::mean(st[k].ratio_short));
      std::fprintf(md.f, "| %.2f | %d / %d | %s | %.1f %% (%d) | %.2f | %.2f | %.3f | %.3f |\n", density, solvable,
                   maps_per_density, names[k], 100.0 * rate, st[k].success, bench::mean(st[k].ms),
                   bench::percentile(st[k].ms, 95), bench::mean(st[k].ratio), bench::mean(st[k].ratio_short));
      std::printf("density %.2f solvable %d/%d %-8s success %.1f%% time %.2f ms len ratio %.3f (shortcut %.3f)\n", density,
                  solvable, maps_per_density, names[k], 100.0 * rate, bench::mean(st[k].ms), bench::mean(st[k].ratio),
                  bench::mean(st[k].ratio_short));
    }
  }
  return 0;
}
