#pragma once

#include <cstddef>
#include <vector>

#include "qkrt/occupancy_grid.hpp"
#include "qkrt/types.hpp"

namespace qkrt {

struct PlanResult {
  bool success = false;
  std::vector<Vec2> path;   ///< world-frame waypoints, start first
  double length = 0.0;      ///< [m]
  std::size_t expansions = 0;  ///< A*: nodes expanded, RRT: tree size
};

double pathLength(const std::vector<Vec2>& path);

/// 8-connected A* with octile heuristic. Diagonal moves may not cut obstacle corners.
PlanResult planAStar(const OccupancyGrid& grid, const Vec2& start, const Vec2& goal);

struct RrtConfig {
  int max_iterations = 4000;
  double step = 1.0;            ///< steering distance [m]
  double goal_bias = 0.05;
  double goal_tolerance = 1.0;  ///< connect directly to the goal from within this radius [m]
  bool star = false;            ///< RRT* (choose-parent + rewire)
  bool stop_at_first = true;    ///< RRT: stop at the first solution. RRT* ignores this and uses the full budget.
  double rewire_gamma = 0.0;    ///< RRT* radius constant; <= 0 derives it from the map area
  unsigned seed = 1;
};

PlanResult planRrt(const OccupancyGrid& grid, const Vec2& start, const Vec2& goal, const RrtConfig& cfg);

/// Greedy line-of-sight shortcutting of a waypoint path.
std::vector<Vec2> shortcutPath(const OccupancyGrid& grid, const std::vector<Vec2>& path);

/// Pure-pursuit path tracker producing (v, w) for a differential-drive base.
class PurePursuit {
 public:
  PurePursuit(double lookahead, double cruise_speed, double goal_tolerance, double max_yaw_rate = 2.0);
  void setPath(const std::vector<Vec2>& path);
  Twist2 compute(const Pose2& pose);
  bool done() const { return done_; }
  /// Cross-track distance from the pose to the active path segment.
  double crossTrackError(const Pose2& pose) const;

 private:
  double lookahead_, cruise_, goal_tol_, max_w_;
  std::vector<Vec2> path_;
  std::size_t seg_ = 0;
  bool done_ = true;
};

}  // namespace qkrt
