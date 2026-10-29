#include <algorithm>
#include <cmath>
#include <functional>

#include "qkrt/collision.hpp"

namespace qkrt {

int DynamicAabbTree::allocate() {
  if (!free_list_.empty()) {
    const int n = free_list_.back();
    free_list_.pop_back();
    nodes_[n] = Node{};
    return n;
  }
  nodes_.emplace_back();
  return static_cast<int>(nodes_.size()) - 1;
}

void DynamicAabbTree::release(int n) {
  nodes_[n].height = -1;
  nodes_[n].parent = -1;
  free_list_.push_back(n);
}

int DynamicAabbTree::insert(const Aabb& box, int user_id) {
  const int leaf = allocate();
  nodes_[leaf].box = box.expanded(margin_);
  nodes_[leaf].user = user_id;
  nodes_[leaf].height = 0;
  insertLeaf(leaf);
  ++leaf_count_;
  return leaf;
}

void DynamicAabbTree::remove(int proxy) {
  removeLeaf(proxy);
  release(proxy);
  --leaf_count_;
}

bool DynamicAabbTree::move(int proxy, const Aabb& box) {
  if (nodes_[proxy].box.contains(box)) return false;
  removeLeaf(proxy);
  nodes_[proxy].box = box.expanded(margin_);
  insertLeaf(proxy);
  return true;
}

void DynamicAabbTree::insertLeaf(int leaf) {
  if (root_ < 0) {
    root_ = leaf;
    nodes_[leaf].parent = -1;
    return;
  }
  // Descend choosing the child that minimises the perimeter-cost increase (Box2D heuristic).
  const Aabb leaf_box = nodes_[leaf].box;
  int index = root_;
  while (!nodes_[index].isLeaf()) {
    const Node& n = nodes_[index];
    const double area = n.box.perimeter();
    const double combined = n.box.merged(leaf_box).perimeter();
    const double cost = 2.0 * combined;
    const double inherit = 2.0 * (combined - area);
    auto childCost = [&](int c) {
      const Aabb m = leaf_box.merged(nodes_[c].box);
      return nodes_[c].isLeaf() ? m.perimeter() + inherit : (m.perimeter() - nodes_[c].box.perimeter()) + inherit;
    };
    const double c1 = childCost(n.left), c2 = childCost(n.right);
    if (cost < c1 && cost < c2) break;
    index = c1 < c2 ? n.left : n.right;
  }
  const int sibling = index;
  const int old_parent = nodes_[sibling].parent;
  const int new_parent = allocate();
  nodes_[new_parent].parent = old_parent;
  nodes_[new_parent].box = leaf_box.merged(nodes_[sibling].box);
  nodes_[new_parent].height = nodes_[sibling].height + 1;
  nodes_[new_parent].left = sibling;
  nodes_[new_parent].right = leaf;
  nodes_[sibling].parent = new_parent;
  nodes_[leaf].parent = new_parent;
  if (old_parent >= 0) {
    if (nodes_[old_parent].left == sibling) nodes_[old_parent].left = new_parent;
    else nodes_[old_parent].right = new_parent;
  } else {
    root_ = new_parent;
  }
  // Refit and rebalance ancestors.
  for (int i = nodes_[leaf].parent; i >= 0; i = nodes_[i].parent) {
    i = balance(i);
    const int l = nodes_[i].left, r = nodes_[i].right;
    nodes_[i].height = 1 + std::max(nodes_[l].height, nodes_[r].height);
    nodes_[i].box = nodes_[l].box.merged(nodes_[r].box);
  }
}

void DynamicAabbTree::removeLeaf(int leaf) {
  if (leaf == root_) {
    root_ = -1;
    return;
  }
  const int parent = nodes_[leaf].parent;
  const int grand = nodes_[parent].parent;
  const int sibling = nodes_[parent].left == leaf ? nodes_[parent].right : nodes_[parent].left;
  if (grand >= 0) {
    if (nodes_[grand].left == parent) nodes_[grand].left = sibling;
    else nodes_[grand].right = sibling;
    nodes_[sibling].parent = grand;
    release(parent);
    for (int i = grand; i >= 0; i = nodes_[i].parent) {
      i = balance(i);
      const int l = nodes_[i].left, r = nodes_[i].right;
      nodes_[i].box = nodes_[l].box.merged(nodes_[r].box);
      nodes_[i].height = 1 + std::max(nodes_[l].height, nodes_[r].height);
    }
  } else {
    root_ = sibling;
    nodes_[sibling].parent = -1;
    release(parent);
  }
  nodes_[leaf].parent = -1;
}

// Left or right rotation if the subtree rooted at `a` is imbalanced. Returns the new subtree root.
int DynamicAabbTree::balance(int ia) {
  Node& A = nodes_[ia];
  if (A.isLeaf() || A.height < 2) return ia;
  const int ib = A.left, ic = A.right;
  const int bal = nodes_[ic].height - nodes_[ib].height;

  auto rotate = [&](int up, int other) -> int {
    // Promote `up` (a child of ia) and push ia down; `other` is ia's remaining child.
    Node& U = nodes_[up];
    const int f = U.left, g = U.right;
    U.left = ia;
    U.parent = nodes_[ia].parent;
    nodes_[ia].parent = up;
    if (U.parent >= 0) {
      if (nodes_[U.parent].left == ia) nodes_[U.parent].left = up;
      else nodes_[U.parent].right = up;
    } else {
      root_ = up;
    }
    const bool other_is_left = nodes_[ia].left == other;
    const int keep = nodes_[f].height > nodes_[g].height ? f : g;
    const int give = keep == f ? g : f;
    U.right = keep;
    if (other_is_left) nodes_[ia].right = give;
    else nodes_[ia].left = give;
    nodes_[give].parent = ia;
    nodes_[ia].box = nodes_[other].box.merged(nodes_[give].box);
    U.box = nodes_[ia].box.merged(nodes_[keep].box);
    nodes_[ia].height = 1 + std::max(nodes_[other].height, nodes_[give].height);
    U.height = 1 + std::max(nodes_[ia].height, nodes_[keep].height);
    return up;
  };

  if (bal > 1) return rotate(ic, ib);
  if (bal < -1) return rotate(ib, ic);
  return ia;
}

bool DynamicAabbTree::validateNode(int n) const {
  const Node& node = nodes_[n];
  if (node.isLeaf()) return node.height == 0;
  const Node& l = nodes_[node.left];
  const Node& r = nodes_[node.right];
  if (l.parent != n || r.parent != n) return false;
  if (node.height != 1 + std::max(l.height, r.height)) return false;
  if (!node.box.contains(l.box) || !node.box.contains(r.box)) return false;
  return validateNode(node.left) && validateNode(node.right);
}

bool DynamicAabbTree::validate() const {
  if (root_ < 0) return leaf_count_ == 0;
  if (nodes_[root_].parent != -1) return false;
  return validateNode(root_);
}

// ----------------------------------------------------------------------------- adapter

void AabbTreeIndex::build(const std::vector<Shape>& shapes) {
  tree_ = DynamicAabbTree(margin_);
  shapes_ = shapes;
  proxies_.resize(shapes.size());
  for (std::size_t i = 0; i < shapes.size(); ++i) proxies_[i] = tree_.insert(shapes[i].bounds(), static_cast<int>(i));
}

void AabbTreeIndex::update(int id, const Shape& s) {
  shapes_[id] = s;
  tree_.move(proxies_[id], s.bounds());
}

std::size_t AabbTreeIndex::query(const Shape& q, std::vector<int>& out) const {
  const Aabb qb = q.bounds();
  std::size_t n = 0;
  tree_.query(qb, [&](int id) {
    ++narrow_tests_;
    if (intersects(q, shapes_[id])) {
      out.push_back(id);
      ++n;
    }
    return true;
  });
  return n;
}

bool AabbTreeIndex::anyHit(const Shape& q) const {
  bool hit = false;
  tree_.query(q.bounds(), [&](int id) {
    ++narrow_tests_;
    hit = intersects(q, shapes_[id]);
    return !hit;
  });
  return hit;
}

}  // namespace qkrt
