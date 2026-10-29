#pragma once

#include <cstdint>
#include <vector>

#include "qkrt/types.hpp"

namespace qkrt {

struct Cell {
  int x = 0;
  int y = 0;
  bool operator==(const Cell& o) const { return x == o.x && y == o.y; }
};

/// 2D occupancy grid. Cell values: 0 = free, anything else = occupied.
class OccupancyGrid {
 public:
  OccupancyGrid() = default;
  OccupancyGrid(int width, int height, double resolution, Vec2 origin = {});

  int width() const { return w_; }
  int height() const { return h_; }
  double resolution() const { return res_; }
  Vec2 origin() const { return origin_; }
  Vec2 extent() const { return {w_ * res_, h_ * res_}; }

  bool inBounds(int x, int y) const { return x >= 0 && y >= 0 && x < w_ && y < h_; }
  bool occupied(int x, int y) const { return !inBounds(x, y) || data_[index(x, y)] != 0; }
  bool occupiedWorld(const Vec2& p) const;
  void set(int x, int y, uint8_t v) { if (inBounds(x, y)) data_[index(x, y)] = v; }
  int index(int x, int y) const { return y * w_ + x; }

  Cell worldToCell(const Vec2& p) const;
  Vec2 cellCenter(const Cell& c) const;

  /// True if the straight segment a->b only crosses free cells (sampled at resolution/4).
  bool segmentFree(const Vec2& a, const Vec2& b) const;

  /// Copy with obstacles grown by `radius` metres (configuration space for a disc robot).
  OccupancyGrid inflated(double radius) const;
  double occupiedFraction() const;
  void fillRect(int x0, int y0, int x1, int y1, uint8_t v = 100);

  /// Random clutter map: axis-aligned blocks added until `density` of cells are occupied.
  static OccupancyGrid random(int width, int height, double resolution, double density, unsigned seed);

  const std::vector<uint8_t>& data() const { return data_; }

 private:
  int w_ = 0, h_ = 0;
  double res_ = 1.0;
  Vec2 origin_;
  std::vector<uint8_t> data_;
};

}  // namespace qkrt
