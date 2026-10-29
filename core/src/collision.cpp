#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "qkrt/collision.hpp"

namespace qkrt {

Aabb Shape::bounds() const {
  if (kind == Kind::Circle) return {{center.x - radius, center.y - radius}, {center.x + radius, center.y + radius}};
  const double c = std::abs(std::cos(angle)), s = std::abs(std::sin(angle));
  const double ex = c * half.x + s * half.y;
  const double ey = s * half.x + c * half.y;
  return {{center.x - ex, center.y - ey}, {center.x + ex, center.y + ey}};
}

namespace {

bool circleCircle(const Shape& a, const Shape& b) {
  const double r = a.radius + b.radius;
  return (a.center - b.center).squaredNorm() <= r * r;
}

bool circleBox(const Shape& c, const Shape& b) {
  // Transform the circle centre into the box frame and clamp.
  const Vec2 d = c.center - b.center;
  const double cs = std::cos(b.angle), sn = std::sin(b.angle);
  const Vec2 local{cs * d.x + sn * d.y, -sn * d.x + cs * d.y};
  const Vec2 closest{std::clamp(local.x, -b.half.x, b.half.x), std::clamp(local.y, -b.half.y, b.half.y)};
  return (local - closest).squaredNorm() <= c.radius * c.radius;
}

bool boxBox(const Shape& a, const Shape& b) {
  // Separating axis theorem on the four face normals.
  const std::array<Vec2, 2> axa{Vec2{std::cos(a.angle), std::sin(a.angle)}, Vec2{-std::sin(a.angle), std::cos(a.angle)}};
  const std::array<Vec2, 2> axb{Vec2{std::cos(b.angle), std::sin(b.angle)}, Vec2{-std::sin(b.angle), std::cos(b.angle)}};
  const Vec2 t = b.center - a.center;
  auto separated = [&](const Vec2& axis) {
    const double ra = a.half.x * std::abs(axa[0].dot(axis)) + a.half.y * std::abs(axa[1].dot(axis));
    const double rb = b.half.x * std::abs(axb[0].dot(axis)) + b.half.y * std::abs(axb[1].dot(axis));
    return std::abs(t.dot(axis)) > ra + rb;
  };
  return !(separated(axa[0]) || separated(axa[1]) || separated(axb[0]) || separated(axb[1]));
}

}  // namespace

bool intersects(const Shape& a, const Shape& b) {
  using K = Shape::Kind;
  if (a.kind == K::Circle && b.kind == K::Circle) return circleCircle(a, b);
  if (a.kind == K::Circle && b.kind == K::Box) return circleBox(a, b);
  if (a.kind == K::Box && b.kind == K::Circle) return circleBox(b, a);
  return boxBox(a, b);
}

// ----------------------------------------------------------------------------- brute force

void BruteForceIndex::build(const std::vector<Shape>& shapes) {
  shapes_ = shapes;
  boxes_.resize(shapes.size());
  for (std::size_t i = 0; i < shapes.size(); ++i) boxes_[i] = shapes[i].bounds();
}

std::size_t BruteForceIndex::query(const Shape& q, std::vector<int>& out) const {
  const Aabb qb = q.bounds();
  std::size_t n = 0;
  for (std::size_t i = 0; i < shapes_.size(); ++i) {
    if (!boxes_[i].overlaps(qb)) continue;
    ++narrow_tests_;
    if (intersects(q, shapes_[i])) {
      out.push_back(static_cast<int>(i));
      ++n;
    }
  }
  return n;
}

bool BruteForceIndex::anyHit(const Shape& q) const {
  const Aabb qb = q.bounds();
  for (std::size_t i = 0; i < shapes_.size(); ++i) {
    if (!boxes_[i].overlaps(qb)) continue;
    ++narrow_tests_;
    if (intersects(q, shapes_[i])) return true;
  }
  return false;
}

// ----------------------------------------------------------------------------- static BVH

void Bvh::build(const std::vector<Shape>& shapes) {
  shapes_ = shapes;
  boxes_.resize(shapes.size());
  std::vector<Vec2> centroids(shapes.size());
  order_.resize(shapes.size());
  for (std::size_t i = 0; i < shapes.size(); ++i) {
    boxes_[i] = shapes[i].bounds();
    centroids[i] = boxes_[i].center();
    order_[i] = static_cast<int>(i);
  }
  nodes_.clear();
  nodes_.reserve(2 * shapes.size() / std::max(1, leaf_size_) + 2);
  if (!shapes.empty()) buildRecursive(0, static_cast<int>(shapes.size()), centroids);
}

int Bvh::buildRecursive(int first, int count, std::vector<Vec2>& centroids) {
  const int id = static_cast<int>(nodes_.size());
  nodes_.emplace_back();
  Aabb box = boxes_[order_[first]];
  Aabb cbox{centroids[order_[first]], centroids[order_[first]]};
  for (int i = first + 1; i < first + count; ++i) {
    box = box.merged(boxes_[order_[i]]);
    const Vec2 c = centroids[order_[i]];
    cbox = cbox.merged({c, c});
  }
  nodes_[id].box = box;
  if (count <= leaf_size_) {
    nodes_[id].first = first;
    nodes_[id].count = count;
    return id;
  }

  const bool split_x = (cbox.max.x - cbox.min.x) >= (cbox.max.y - cbox.min.y);
  auto key = [&](int prim) { return split_x ? centroids[prim].x : centroids[prim].y; };
  const double lo = split_x ? cbox.min.x : cbox.min.y;
  const double hi = split_x ? cbox.max.x : cbox.max.y;

  int mid = first + count / 2;
  if (hi - lo > 1e-12) {
    // Binned SAH (perimeter metric in 2D) over 12 buckets.
    constexpr int kBins = 12;
    struct Bin { Aabb box; int n = 0; };
    std::array<Bin, kBins> bins{};
    auto binOf = [&](int prim) {
      return std::min(kBins - 1, static_cast<int>(kBins * (key(prim) - lo) / (hi - lo)));
    };
    for (int i = first; i < first + count; ++i) {
      const int b = binOf(order_[i]);
      bins[b].box = bins[b].n == 0 ? boxes_[order_[i]] : bins[b].box.merged(boxes_[order_[i]]);
      ++bins[b].n;
    }
    double best_cost = std::numeric_limits<double>::infinity();
    int best_split = -1;
    for (int s = 1; s < kBins; ++s) {
      Aabb lb, rb;
      int ln = 0, rn = 0;
      for (int b = 0; b < s; ++b)
        if (bins[b].n) { lb = ln ? lb.merged(bins[b].box) : bins[b].box; ln += bins[b].n; }
      for (int b = s; b < kBins; ++b)
        if (bins[b].n) { rb = rn ? rb.merged(bins[b].box) : bins[b].box; rn += bins[b].n; }
      if (ln == 0 || rn == 0) continue;
      const double cost = ln * lb.perimeter() + rn * rb.perimeter();
      if (cost < best_cost) { best_cost = cost; best_split = s; }
    }
    if (best_split > 0) {
      auto it = std::partition(order_.begin() + first, order_.begin() + first + count,
                               [&](int prim) { return binOf(prim) < best_split; });
      mid = static_cast<int>(it - order_.begin());
    }
  }
  if (mid == first || mid == first + count) {  // degenerate: fall back to median split
    mid = first + count / 2;
    std::nth_element(order_.begin() + first, order_.begin() + mid, order_.begin() + first + count,
                     [&](int a, int b) { return key(a) < key(b); });
  }
  const int l = buildRecursive(first, mid - first, centroids);
  const int r = buildRecursive(mid, first + count - mid, centroids);
  nodes_[id].left = l;
  nodes_[id].right = r;
  return id;
}

template <typename F>
void Bvh::traverse(const Aabb& qb, F&& visit) const {
  if (nodes_.empty()) return;
  int stack[256];
  int sp = 0;
  stack[sp++] = 0;
  while (sp > 0) {
    const Node& n = nodes_[stack[--sp]];
    if (!n.box.overlaps(qb)) continue;
    if (n.count > 0) {
      for (int i = n.first; i < n.first + n.count; ++i)
        if (!visit(order_[i])) return;
    } else {
      stack[sp++] = n.right;
      stack[sp++] = n.left;
    }
  }
}

std::size_t Bvh::query(const Shape& q, std::vector<int>& out) const {
  const Aabb qb = q.bounds();
  std::size_t n = 0;
  traverse(qb, [&](int prim) {
    if (boxes_[prim].overlaps(qb)) {
      ++narrow_tests_;
      if (intersects(q, shapes_[prim])) {
        out.push_back(prim);
        ++n;
      }
    }
    return true;
  });
  return n;
}

bool Bvh::anyHit(const Shape& q) const {
  const Aabb qb = q.bounds();
  bool hit = false;
  traverse(qb, [&](int prim) {
    if (boxes_[prim].overlaps(qb)) {
      ++narrow_tests_;
      hit = intersects(q, shapes_[prim]);
    }
    return !hit;
  });
  return hit;
}

int Bvh::depth() const {
  if (nodes_.empty()) return 0;
  int best = 0;
  std::vector<std::pair<int, int>> stack{{0, 1}};
  while (!stack.empty()) {
    auto [n, d] = stack.back();
    stack.pop_back();
    best = std::max(best, d);
    if (nodes_[n].count == 0) {
      stack.push_back({nodes_[n].left, d + 1});
      stack.push_back({nodes_[n].right, d + 1});
    }
  }
  return best;
}

}  // namespace qkrt
