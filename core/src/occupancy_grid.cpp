#include "qkrt/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace qkrt {

OccupancyGrid::OccupancyGrid(int width, int height, double resolution, Vec2 origin)
    : w_(width), h_(height), res_(resolution), origin_(origin), data_(static_cast<size_t>(width) * height, 0) {}

Cell OccupancyGrid::worldToCell(const Vec2& p) const {
  return {static_cast<int>(std::floor((p.x - origin_.x) / res_)),
          static_cast<int>(std::floor((p.y - origin_.y) / res_))};
}

Vec2 OccupancyGrid::cellCenter(const Cell& c) const {
  return {origin_.x + (c.x + 0.5) * res_, origin_.y + (c.y + 0.5) * res_};
}

bool OccupancyGrid::occupiedWorld(const Vec2& p) const {
  const Cell c = worldToCell(p);
  return occupied(c.x, c.y);
}

bool OccupancyGrid::segmentFree(const Vec2& a, const Vec2& b) const {
  const double len = distance(a, b);
  const int n = std::max(1, static_cast<int>(std::ceil(len / (0.25 * res_))));
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / n;
    if (occupiedWorld(a + (b - a) * t)) return false;
  }
  return true;
}

OccupancyGrid OccupancyGrid::inflated(double radius) const {
  OccupancyGrid out = *this;
  const int r = static_cast<int>(std::ceil(radius / res_));
  std::vector<Cell> disk;
  for (int dy = -r; dy <= r; ++dy)
    for (int dx = -r; dx <= r; ++dx)
      if (std::hypot(dx, dy) * res_ <= radius + 1e-9) disk.push_back({dx, dy});
  for (int y = 0; y < h_; ++y)
    for (int x = 0; x < w_; ++x) {
      if (data_[index(x, y)] == 0) continue;
      for (const Cell& d : disk) out.set(x + d.x, y + d.y, 100);
    }
  return out;
}

double OccupancyGrid::occupiedFraction() const {
  if (data_.empty()) return 0.0;
  const auto n = std::count_if(data_.begin(), data_.end(), [](uint8_t v) { return v != 0; });
  return static_cast<double>(n) / data_.size();
}

void OccupancyGrid::fillRect(int x0, int y0, int x1, int y1, uint8_t v) {
  for (int y = std::max(0, y0); y <= std::min(h_ - 1, y1); ++y)
    for (int x = std::max(0, x0); x <= std::min(w_ - 1, x1); ++x) data_[index(x, y)] = v;
}

OccupancyGrid OccupancyGrid::random(int width, int height, double resolution, double density, unsigned seed) {
  OccupancyGrid g(width, height, resolution);
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> ux(0, width - 1), uy(0, height - 1);
  const int max_side = std::max(2, std::min(width, height) / 8);
  std::uniform_int_distribution<int> us(1, max_side);
  const size_t target = static_cast<size_t>(density * width * height);
  size_t occ = 0;
  int guard = 0;
  while (occ < target && guard++ < 100000) {
    const int x0 = ux(rng), y0 = uy(rng);
    const int x1 = x0 + us(rng), y1 = y0 + us(rng);
    for (int y = y0; y <= std::min(height - 1, y1); ++y)
      for (int x = x0; x <= std::min(width - 1, x1); ++x) {
        uint8_t& c = g.data_[g.index(x, y)];
        if (c == 0) {
          c = 100;
          ++occ;
        }
      }
  }
  return g;
}

}  // namespace qkrt
