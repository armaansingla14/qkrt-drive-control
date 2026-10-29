#pragma once

#include <cstdint>
#include <vector>

#include "qkrt/types.hpp"

namespace qkrt {

struct Aabb {
  Vec2 min;
  Vec2 max;

  bool overlaps(const Aabb& o) const {
    return min.x <= o.max.x && max.x >= o.min.x && min.y <= o.max.y && max.y >= o.min.y;
  }
  bool contains(const Aabb& o) const {
    return min.x <= o.min.x && min.y <= o.min.y && max.x >= o.max.x && max.y >= o.max.y;
  }
  Aabb merged(const Aabb& o) const {
    return {{std::min(min.x, o.min.x), std::min(min.y, o.min.y)}, {std::max(max.x, o.max.x), std::max(max.y, o.max.y)}};
  }
  Aabb expanded(double m) const { return {{min.x - m, min.y - m}, {max.x + m, max.y + m}}; }
  double perimeter() const { return 2.0 * ((max.x - min.x) + (max.y - min.y)); }
  Vec2 center() const { return (min + max) * 0.5; }
};

/// Collision primitive: a disc or an oriented box.
struct Shape {
  enum class Kind : uint8_t { Circle, Box };
  Kind kind = Kind::Circle;
  Vec2 center;
  double radius = 0.0;  ///< Circle
  Vec2 half;            ///< Box half extents in its local frame
  double angle = 0.0;   ///< Box orientation [rad]

  static Shape circle(Vec2 c, double r) { Shape s; s.kind = Kind::Circle; s.center = c; s.radius = r; return s; }
  static Shape box(Vec2 c, Vec2 half_extents, double angle) {
    Shape s; s.kind = Kind::Box; s.center = c; s.half = half_extents; s.angle = angle; return s;
  }
  Aabb bounds() const;
};

/// Exact narrow-phase test (circle/circle, circle/box, box/box via SAT).
bool intersects(const Shape& a, const Shape& b);

/// Common interface for spatial indices answering "which shapes overlap this query shape".
class CollisionIndex {
 public:
  virtual ~CollisionIndex() = default;
  virtual void build(const std::vector<Shape>& shapes) = 0;
  /// Appends ids of overlapping shapes to `out` (order unspecified). Returns the count added.
  virtual std::size_t query(const Shape& q, std::vector<int>& out) const = 0;
  /// True if anything overlaps (early exit).
  virtual bool anyHit(const Shape& q) const = 0;
  /// Number of narrow-phase tests performed since the last reset (diagnostic).
  std::size_t narrowTests() const { return narrow_tests_; }
  void resetStats() { narrow_tests_ = 0; }

 protected:
  mutable std::size_t narrow_tests_ = 0;
};

/// Baseline: AABB reject + narrow phase against every shape.
class BruteForceIndex final : public CollisionIndex {
 public:
  void build(const std::vector<Shape>& shapes) override;
  std::size_t query(const Shape& q, std::vector<int>& out) const override;
  bool anyHit(const Shape& q) const override;

 private:
  std::vector<Shape> shapes_;
  std::vector<Aabb> boxes_;
};

/// Static bounding volume hierarchy built top-down (binned SAH on the longest centroid axis),
/// stored as a flat array with the right child index for depth-first traversal.
class Bvh final : public CollisionIndex {
 public:
  explicit Bvh(int leaf_size = 4) : leaf_size_(leaf_size) {}
  void build(const std::vector<Shape>& shapes) override;
  std::size_t query(const Shape& q, std::vector<int>& out) const override;
  bool anyHit(const Shape& q) const override;
  std::size_t nodeCount() const { return nodes_.size(); }
  int depth() const;

 private:
  struct Node {
    Aabb box;
    int left = -1;     ///< interior: index of left child (right = right_)
    int right = -1;
    int first = 0;     ///< leaf: first primitive in `order_`
    int count = 0;     ///< leaf: number of primitives (0 for interior nodes)
  };
  int buildRecursive(int first, int count, std::vector<Vec2>& centroids);
  template <typename F>
  void traverse(const Aabb& qb, F&& visit) const;

  int leaf_size_;
  std::vector<Node> nodes_;
  std::vector<int> order_;
  std::vector<Shape> shapes_;
  std::vector<Aabb> boxes_;
};

/// Dynamic AABB tree (incremental insert/remove/move with fattened boxes and AVL-style
/// rotations), suitable for moving obstacles such as other robots.
class DynamicAabbTree {
 public:
  explicit DynamicAabbTree(double margin = 0.1) : margin_(margin) {}
  int insert(const Aabb& box, int user_id);
  void remove(int proxy);
  /// Returns true if the proxy had to be re-inserted (box left its fat AABB).
  bool move(int proxy, const Aabb& box);
  template <typename F>
  void query(const Aabb& box, F&& visit) const;  // visit(user_id) -> bool keep_going
  int height() const { return root_ < 0 ? 0 : nodes_[root_].height; }
  int userId(int proxy) const { return nodes_[proxy].user; }
  const Aabb& fatBox(int proxy) const { return nodes_[proxy].box; }
  std::size_t size() const { return leaf_count_; }
  /// Structural invariant check (parents, heights, containment). For tests.
  bool validate() const;

 private:
  struct Node {
    Aabb box;
    int parent = -1;
    int left = -1;
    int right = -1;
    int height = 0;  ///< leaf = 0, free = -1
    int user = -1;
    bool isLeaf() const { return left < 0; }
  };
  int allocate();
  void release(int n);
  void insertLeaf(int leaf);
  void removeLeaf(int leaf);
  int balance(int a);
  bool validateNode(int n) const;

  double margin_;
  std::vector<Node> nodes_;
  std::vector<int> free_list_;
  int root_ = -1;
  std::size_t leaf_count_ = 0;
};

/// CollisionIndex adapter around DynamicAabbTree.
class AabbTreeIndex final : public CollisionIndex {
 public:
  explicit AabbTreeIndex(double margin = 0.0) : margin_(margin), tree_(margin) {}
  void build(const std::vector<Shape>& shapes) override;
  std::size_t query(const Shape& q, std::vector<int>& out) const override;
  bool anyHit(const Shape& q) const override;
  void update(int id, const Shape& s);
  const DynamicAabbTree& tree() const { return tree_; }

 private:
  double margin_;
  DynamicAabbTree tree_;
  std::vector<Shape> shapes_;
  std::vector<int> proxies_;
};

// ----------------------------------------------------------------------------- template impl

template <typename F>
void DynamicAabbTree::query(const Aabb& box, F&& visit) const {
  if (root_ < 0) return;
  int stack[128];
  std::vector<int> overflow;
  int sp = 0;
  stack[sp++] = root_;
  while (sp > 0 || !overflow.empty()) {
    int n;
    if (!overflow.empty()) {
      n = overflow.back();
      overflow.pop_back();
    } else {
      n = stack[--sp];
    }
    const Node& node = nodes_[n];
    if (!node.box.overlaps(box)) continue;
    if (node.isLeaf()) {
      if (!visit(node.user)) return;
    } else {
      for (int c : {node.left, node.right}) {
        if (sp < 128) stack[sp++] = c;
        else overflow.push_back(c);
      }
    }
  }
}

}  // namespace qkrt
