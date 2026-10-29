#include <algorithm>
#include <random>

#include "qkrt/collision.hpp"
#include "test_harness.hpp"

using namespace qkrt;

namespace {
std::vector<Shape> randomShapes(int n, double world, unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> up(0.0, world), us(0.1, 1.5), ua(-kPi, kPi), u01(0, 1);
  std::vector<Shape> out;
  for (int i = 0; i < n; ++i) {
    if (u01(rng) < 0.5) out.push_back(Shape::circle({up(rng), up(rng)}, us(rng)));
    else out.push_back(Shape::box({up(rng), up(rng)}, {us(rng), us(rng)}, ua(rng)));
  }
  return out;
}

std::vector<int> sorted(std::vector<int> v) {
  std::sort(v.begin(), v.end());
  return v;
}
}  // namespace

TEST_CASE("narrow phase basic cases") {
  CHECK(intersects(Shape::circle({0, 0}, 1), Shape::circle({1.9, 0}, 1)));
  CHECK(!intersects(Shape::circle({0, 0}, 1), Shape::circle({2.1, 0}, 1)));
  // Rotated box: the AABBs overlap but the shapes do not.
  const Shape diamond = Shape::box({0, 0}, {1, 1}, kPi / 4);
  CHECK(diamond.bounds().overlaps(Shape::circle({1.3, 1.3}, 0.2).bounds()));
  CHECK(!intersects(diamond, Shape::circle({1.3, 1.3}, 0.2)));
  CHECK(intersects(diamond, Shape::circle({1.3, 0.0}, 0.2)));
  CHECK(intersects(Shape::box({0, 0}, {1, 0.2}, 0.0), Shape::box({0, 0}, {1, 0.2}, kPi / 2)));
  CHECK(!intersects(Shape::box({0, 0}, {1, 0.2}, 0.0), Shape::box({0, 1.0}, {1, 0.2}, 0.3)));
  CHECK(!intersects(diamond, Shape::box({1.6, 1.6}, {0.5, 0.5}, kPi / 4)));
}

TEST_CASE("rotated box bounds are tight") {
  const Aabb b = Shape::box({0, 0}, {1, 1}, kPi / 4).bounds();
  CHECK_NEAR(b.max.x, std::sqrt(2.0), 1e-12);
  CHECK_NEAR(b.min.y, -std::sqrt(2.0), 1e-12);
}

TEST_CASE("bvh and aabb tree agree with brute force") {
  const auto shapes = randomShapes(3000, 200.0, 1);
  const auto queries = randomShapes(500, 200.0, 2);
  BruteForceIndex bf;
  Bvh bvh;
  AabbTreeIndex tree;
  bf.build(shapes);
  bvh.build(shapes);
  tree.build(shapes);
  int total = 0;
  for (const Shape& q : queries) {
    std::vector<int> a, b, c;
    bf.query(q, a);
    bvh.query(q, b);
    tree.query(q, c);
    CHECK(sorted(a) == sorted(b));
    CHECK(sorted(a) == sorted(c));
    CHECK(bf.anyHit(q) == !a.empty());
    CHECK(bvh.anyHit(q) == !a.empty());
    CHECK(tree.anyHit(q) == !a.empty());
    total += static_cast<int>(a.size());
  }
  CHECK(total > 0);
}

TEST_CASE("bvh depth is logarithmic") {
  Bvh bvh;
  bvh.build(randomShapes(20000, 1000.0, 3));
  CHECK(bvh.depth() < 40);
  CHECK(bvh.nodeCount() > 0);
}

TEST_CASE("dynamic tree insert/remove/move keeps invariants") {
  DynamicAabbTree tree(0.2);
  const auto shapes = randomShapes(2000, 100.0, 4);
  std::vector<int> proxies;
  for (std::size_t i = 0; i < shapes.size(); ++i) proxies.push_back(tree.insert(shapes[i].bounds(), (int)i));
  CHECK(tree.validate());
  CHECK(tree.size() == 2000);
  CHECK(tree.height() < 30);
  for (std::size_t i = 0; i < proxies.size(); i += 2) tree.remove(proxies[i]);
  CHECK(tree.validate());
  CHECK(tree.size() == 1000);
  std::mt19937 rng(5);
  std::normal_distribution<double> n(0.0, 0.5);
  int reinserted = 0;
  for (std::size_t i = 1; i < proxies.size(); i += 2) {
    Aabb b = shapes[i].bounds();
    const Vec2 d{n(rng), n(rng)};
    b.min += d;
    b.max += d;
    reinserted += tree.move(proxies[i], b) ? 1 : 0;
    CHECK(tree.fatBox(proxies[i]).contains(b));
  }
  CHECK(tree.validate());
  CHECK(reinserted > 0 && reinserted < 1000);
}

TEST_CASE("aabb tree index tracks moving obstacles") {
  std::vector<Shape> shapes{Shape::circle({0, 0}, 0.5), Shape::circle({10, 0}, 0.5)};
  AabbTreeIndex idx(0.1);
  idx.build(shapes);
  const Shape probe = Shape::circle({5, 0}, 0.5);
  CHECK(!idx.anyHit(probe));
  idx.update(1, Shape::circle({5.5, 0}, 0.5));
  CHECK(idx.anyHit(probe));
}

TEST_MAIN()
