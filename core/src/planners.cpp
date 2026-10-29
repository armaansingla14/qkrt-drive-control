#include "qkrt/planners.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <random>

namespace qkrt {

double pathLength(const std::vector<Vec2>& path) {
  double len = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) len += distance(path[i - 1], path[i]);
  return len;
}

// ----------------------------------------------------------------------------- A*

PlanResult planAStar(const OccupancyGrid& grid, const Vec2& start, const Vec2& goal) {
  PlanResult res;
  const Cell s = grid.worldToCell(start), g = grid.worldToCell(goal);
  if (grid.occupied(s.x, s.y) || grid.occupied(g.x, g.y)) return res;

  const int W = grid.width(), H = grid.height();
  const std::size_t N = static_cast<std::size_t>(W) * H;
  constexpr double kInf = std::numeric_limits<double>::infinity();
  std::vector<double> gcost(N, kInf);
  std::vector<int> parent(N, -1);
  std::vector<uint8_t> closed(N, 0);

  auto h = [&](int x, int y) {
    const double dx = std::abs(x - g.x), dy = std::abs(y - g.y);
    return (dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy);
  };
  using Item = std::pair<double, int>;  // (f, index)
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
  const int si = grid.index(s.x, s.y), gi = grid.index(g.x, g.y);
  gcost[si] = 0.0;
  open.push({h(s.x, s.y), si});

  static const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  bool found = false;
  while (!open.empty()) {
    const int cur = open.top().second;
    open.pop();
    if (closed[cur]) continue;
    closed[cur] = 1;
    ++res.expansions;
    if (cur == gi) {
      found = true;
      break;
    }
    const int cx = cur % W, cy = cur / W;
    for (int k = 0; k < 8; ++k) {
      const int nx = cx + dx8[k], ny = cy + dy8[k];
      if (grid.occupied(nx, ny)) continue;
      const bool diag = k >= 4;
      if (diag && (grid.occupied(cx + dx8[k], cy) || grid.occupied(cx, cy + dy8[k]))) continue;
      const int ni = grid.index(nx, ny);
      if (closed[ni]) continue;
      const double ng = gcost[cur] + (diag ? std::sqrt(2.0) : 1.0);
      if (ng < gcost[ni]) {
        gcost[ni] = ng;
        parent[ni] = cur;
        open.push({ng + h(nx, ny), ni});
      }
    }
  }
  if (!found) return res;

  std::vector<Vec2> rev;
  for (int i = gi; i != -1; i = parent[i]) rev.push_back(grid.cellCenter({i % W, i / W}));
  res.path.assign(rev.rbegin(), rev.rend());
  res.path.front() = start;
  res.path.back() = goal;
  res.length = pathLength(res.path);
  res.success = true;
  return res;
}

// ----------------------------------------------------------------------------- RRT / RRT*

namespace {
struct Node {
  Vec2 p;
  int parent = -1;
  double cost = 0.0;
  std::vector<int> children;
};

void propagateCost(std::vector<Node>& tree, int root, double delta) {
  std::vector<int> stack{root};
  while (!stack.empty()) {
    const int n = stack.back();
    stack.pop_back();
    tree[n].cost += delta;
    for (int c : tree[n].children) stack.push_back(c);
  }
}

void reparent(std::vector<Node>& tree, int node, int new_parent, double new_cost) {
  auto& old_children = tree[tree[node].parent].children;
  old_children.erase(std::find(old_children.begin(), old_children.end(), node));
  tree[node].parent = new_parent;
  tree[new_parent].children.push_back(node);
  propagateCost(tree, node, new_cost - tree[node].cost);
}
}  // namespace

PlanResult planRrt(const OccupancyGrid& grid, const Vec2& start, const Vec2& goal, const RrtConfig& cfg) {
  PlanResult res;
  if (grid.occupiedWorld(start) || grid.occupiedWorld(goal)) return res;

  std::mt19937 rng(cfg.seed);
  const Vec2 lo = grid.origin(), ext = grid.extent();
  std::uniform_real_distribution<double> ux(lo.x, lo.x + ext.x), uy(lo.y, lo.y + ext.y), u01(0.0, 1.0);
  double gamma = cfg.rewire_gamma;
  if (gamma <= 0.0) {
    const double free_area = ext.x * ext.y * (1.0 - grid.occupiedFraction());
    gamma = 2.0 * std::sqrt(1.5) * std::sqrt(free_area / kPi);
  }

  std::vector<Node> tree;
  tree.reserve(cfg.max_iterations + 1);
  tree.push_back({start, -1, 0.0, {}});
  std::vector<int> goal_candidates;
  std::vector<int> near;

  for (int it = 0; it < cfg.max_iterations; ++it) {
    const Vec2 sample = u01(rng) < cfg.goal_bias ? goal : Vec2{ux(rng), uy(rng)};
    int nearest = 0;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < tree.size(); ++i) {
      const double d2 = (tree[i].p - sample).squaredNorm();
      if (d2 < best_d2) {
        best_d2 = d2;
        nearest = static_cast<int>(i);
      }
    }
    const double d = std::sqrt(best_d2);
    if (d < 1e-9) continue;
    const Vec2 from = tree[nearest].p;
    const Vec2 to = d <= cfg.step ? sample : from + (sample - from) * (cfg.step / d);
    if (!grid.segmentFree(from, to)) continue;

    int parent = nearest;
    double cost = tree[nearest].cost + distance(from, to);
    near.clear();
    if (cfg.star) {
      const double n = static_cast<double>(tree.size() + 1);
      const double radius = std::min(gamma * std::sqrt(std::log(n) / n), 3.0 * cfg.step);
      const double r2 = radius * radius;
      for (std::size_t i = 0; i < tree.size(); ++i)
        if ((tree[i].p - to).squaredNorm() <= r2) near.push_back(static_cast<int>(i));
      for (int j : near) {  // choose best parent
        const double c = tree[j].cost + distance(tree[j].p, to);
        if (c < cost && grid.segmentFree(tree[j].p, to)) {
          cost = c;
          parent = j;
        }
      }
    }
    const int id = static_cast<int>(tree.size());
    tree.push_back({to, parent, cost, {}});
    tree[parent].children.push_back(id);

    if (cfg.star) {  // rewire neighbours through the new node
      for (int j : near) {
        if (j == parent) continue;
        const double c = cost + distance(to, tree[j].p);
        if (c + 1e-12 < tree[j].cost && grid.segmentFree(to, tree[j].p)) reparent(tree, j, id, c);
      }
    }

    if (distance(to, goal) <= cfg.goal_tolerance && grid.segmentFree(to, goal)) {
      goal_candidates.push_back(id);
      if (!cfg.star && cfg.stop_at_first) break;
    }
  }
  res.expansions = tree.size();
  if (goal_candidates.empty()) return res;

  int best = -1;
  double best_cost = std::numeric_limits<double>::infinity();
  for (int c : goal_candidates) {
    const double total = tree[c].cost + distance(tree[c].p, goal);
    if (total < best_cost) {
      best_cost = total;
      best = c;
    }
  }
  std::vector<Vec2> rev{goal};
  if (distance(tree[best].p, goal) < 1e-9) rev.clear();
  for (int i = best; i != -1; i = tree[i].parent) rev.push_back(tree[i].p);
  res.path.assign(rev.rbegin(), rev.rend());
  res.length = pathLength(res.path);
  res.success = true;
  return res;
}

std::vector<Vec2> shortcutPath(const OccupancyGrid& grid, const std::vector<Vec2>& path) {
  if (path.size() < 3) return path;
  std::vector<Vec2> out{path.front()};
  std::size_t i = 0;
  while (i + 1 < path.size()) {
    std::size_t j = path.size() - 1;
    while (j > i + 1 && !grid.segmentFree(path[i], path[j])) --j;
    out.push_back(path[j]);
    i = j;
  }
  return out;
}

// ----------------------------------------------------------------------------- Pure pursuit

PurePursuit::PurePursuit(double lookahead, double cruise_speed, double goal_tolerance, double max_yaw_rate)
    : lookahead_(lookahead), cruise_(cruise_speed), goal_tol_(goal_tolerance), max_w_(max_yaw_rate) {}

void PurePursuit::setPath(const std::vector<Vec2>& path) {
  path_ = path;
  seg_ = 0;
  done_ = path_.size() < 2;
}

namespace {
// Closest point parameter of p on segment ab, clamped to [0,1].
double segmentParam(const Vec2& a, const Vec2& b, const Vec2& p) {
  const Vec2 ab = b - a;
  const double l2 = ab.squaredNorm();
  if (l2 < 1e-12) return 0.0;
  return std::clamp((p - a).dot(ab) / l2, 0.0, 1.0);
}
}  // namespace

double PurePursuit::crossTrackError(const Pose2& pose) const {
  if (path_.size() < 2) return 0.0;
  const std::size_t s = std::min(seg_, path_.size() - 2);
  const Vec2 a = path_[s], b = path_[s + 1];
  return distance(a + (b - a) * segmentParam(a, b, pose.position()), pose.position());
}

Twist2 PurePursuit::compute(const Pose2& pose) {
  if (done_) return {};
  const Vec2 p = pose.position();
  if (distance(p, path_.back()) <= goal_tol_) {
    done_ = true;
    return {};
  }
  // Advance the active segment once the robot has passed its end.
  while (seg_ + 2 < path_.size() && segmentParam(path_[seg_], path_[seg_ + 1], p) >= 1.0) ++seg_;

  // Lookahead point: walk forward along the path from the projection.
  const Vec2 a = path_[seg_], b = path_[seg_ + 1];
  Vec2 cur = a + (b - a) * segmentParam(a, b, p);
  double remaining = lookahead_;
  std::size_t k = seg_ + 1;
  Vec2 target = path_.back();
  while (k < path_.size()) {
    const double d = distance(cur, path_[k]);
    if (d >= remaining) {
      target = cur + (path_[k] - cur) * (remaining / d);
      break;
    }
    remaining -= d;
    cur = path_[k++];
  }

  const Vec2 rel = target - p;
  const double alpha = wrapAngle(std::atan2(rel.y, rel.x) - pose.theta);
  const double ld = std::max(rel.norm(), 1e-6);
  const double dist_goal = distance(p, path_.back());
  double v = cruise_ * std::min(1.0, dist_goal / (2.0 * lookahead_) + 0.2);
  if (std::abs(alpha) > kPi / 2.0) v = 0.0;  // turn in place when the target is behind
  double w = (v > 0.0) ? 2.0 * v * std::sin(alpha) / ld : max_w_ * (alpha > 0 ? 1.0 : -1.0);
  w = std::clamp(w, -max_w_, max_w_);
  if (std::abs(w) >= max_w_ && v > 0.0) v *= 0.5;
  return {v, w};
}

}  // namespace qkrt
