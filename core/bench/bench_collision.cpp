// Collision query latency: static BVH and dynamic AABB tree vs brute force.
#include <cstdio>
#include <memory>
#include <random>

#include "bench_util.hpp"
#include "qkrt/collision.hpp"

using namespace qkrt;
using bench::Clock;

namespace {

std::vector<Shape> makeObstacles(int n, double side, std::mt19937& rng) {
  std::uniform_real_distribution<double> up(0.0, side), us(0.2, 1.2), ua(-kPi, kPi), u01(0.0, 1.0);
  std::vector<Shape> out;
  out.reserve(n);
  for (int i = 0; i < n; ++i) {
    if (u01(rng) < 0.5) out.push_back(Shape::circle({up(rng), up(rng)}, us(rng)));
    else out.push_back(Shape::box({up(rng), up(rng)}, {us(rng), us(rng)}, ua(rng)));
  }
  return out;
}

std::vector<Shape> makeFootprints(int n, double side, std::mt19937& rng) {
  std::uniform_real_distribution<double> up(0.0, side), ua(-kPi, kPi);
  std::vector<Shape> out;
  out.reserve(n);
  for (int i = 0; i < n; ++i) out.push_back(Shape::box({up(rng), up(rng)}, {0.4, 0.25}, ua(rng)));  // 0.8 x 0.5 m rover
  return out;
}

struct Timing {
  double build_ms = 0, query_ns = 0, any_ns = 0;
  std::size_t hits = 0, any = 0, narrow = 0;
};

Timing run(CollisionIndex& idx, const std::vector<Shape>& obs, const std::vector<Shape>& queries) {
  Timing t;
  auto t0 = Clock::now();
  idx.build(obs);
  t.build_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
  std::vector<int> out;
  out.reserve(64);
  idx.resetStats();
  t0 = Clock::now();
  for (const Shape& q : queries) {
    out.clear();
    t.hits += idx.query(q, out);
  }
  t.query_ns = std::chrono::duration<double, std::nano>(Clock::now() - t0).count() / queries.size();
  t.narrow = idx.narrowTests();
  t0 = Clock::now();
  for (const Shape& q : queries) t.any += idx.anyHit(q);
  t.any_ns = std::chrono::duration<double, std::nano>(Clock::now() - t0).count() / queries.size();
  return t;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string out = bench::outDir(argc, argv);
  const bool quick = bench::hasFlag(argc, argv, "--quick");
  const int num_queries = quick ? 10000 : 20000;
  std::vector<int> sizes{100, 1000, 10000, 100000};
  if (quick) sizes.pop_back();

  bench::File csv(out + "/collision.csv"), md(out + "/collision.md");
  if (!csv.f || !md.f) return 1;
  std::fprintf(csv.f, "obstacles,queries,method,build_ms,query_ns,any_hit_ns,total_hits,queries_with_hit,narrow_tests,speedup_vs_brute\n");
  std::fprintf(md.f, "Random circles and oriented boxes (~1 obstacle per 25 m^2), %d oriented-box rover footprints per size.\n\n"
                     "| Obstacles | Method | Build [ms] | Query all hits [ns] | Any-hit [ns] | Speed-up (all hits) | Hits match brute force |\n"
                     "|---|---|---|---|---|---|---|\n", num_queries);

  for (int n : sizes) {
    std::mt19937 rng(42 + n);
    const double side = std::sqrt(25.0 * n);
    const auto obs = makeObstacles(n, side, rng);
    const auto queries = makeFootprints(num_queries, side, rng);
    BruteForceIndex bf;
    Bvh bvh;
    AabbTreeIndex tree;
    const Timing tb = run(bf, obs, queries);
    const std::pair<const char*, Timing> rows[] = {{"brute_force", tb}, {"bvh", run(bvh, obs, queries)}, {"aabb_tree", run(tree, obs, queries)}};
    for (const auto& [name, t] : rows) {
      const double speedup = tb.query_ns / t.query_ns;
      const bool match = t.hits == tb.hits && t.any == tb.any;
      std::fprintf(csv.f, "%d,%d,%s,%.3f,%.1f,%.1f,%zu,%zu,%zu,%.2f\n", n, num_queries, name, t.build_ms, t.query_ns, t.any_ns,
                   t.hits, t.any, t.narrow, speedup);
      std::fprintf(md.f, "| %d | %s | %.2f | %.0f | %.0f | %.1fx | %s |\n", n, name, t.build_ms, t.query_ns, t.any_ns, speedup,
                   match ? "yes" : "NO");
      std::printf("N=%6d %-12s build %8.2f ms  query %10.0f ns  any %10.0f ns  speedup %7.1fx  hits %zu %s\n", n, name,
                  t.build_ms, t.query_ns, t.any_ns, speedup, t.hits, match ? "" : "MISMATCH");
    }
  }

  // Moving obstacles: 10 % of obstacles move every frame, then a batch of footprint queries.
  const int n = 10000, frames = quick ? 20 : 100, movers = n / 10, per_frame_queries = 1000;
  std::mt19937 rng(7);
  const double side = std::sqrt(25.0 * n);
  auto obs = makeObstacles(n, side, rng);
  const auto queries = makeFootprints(per_frame_queries, side, rng);
  std::normal_distribution<double> step(0.0, 0.05);
  std::uniform_int_distribution<int> pick(0, n - 1);
  std::vector<std::vector<std::pair<int, Vec2>>> moves(frames);
  for (auto& f : moves)
    for (int k = 0; k < movers; ++k) f.push_back({pick(rng), {step(rng), step(rng)}});

  std::fprintf(md.f, "\nDynamic scene: %d obstacles, %d move per frame (~5 cm), %d queries per frame, %d frames.\n\n"
                     "| Strategy | ms per frame | Hits match |\n|---|---|---|\n", n, movers, per_frame_queries, frames);
  std::fprintf(csv.f, "\n");
  bench::File dyn(out + "/collision_dynamic.csv");
  std::fprintf(dyn.f, "strategy,obstacles,movers_per_frame,queries_per_frame,frames,ms_per_frame,total_hits\n");
  auto simulate = [&](int strategy) {
    std::vector<Shape> world = obs;
    BruteForceIndex bf;
    Bvh bvh;
    AabbTreeIndex tree(0.1);
    if (strategy == 1) bvh.build(world);
    if (strategy == 2) tree.build(world);
    std::size_t hits = 0;
    std::vector<int> out;
    const auto t0 = Clock::now();
    for (const auto& f : moves) {
      for (const auto& [id, d] : f) {
        world[id].center += d;
        if (strategy == 2) tree.update(id, world[id]);
      }
      if (strategy == 0) bf.build(world);
      if (strategy == 1) bvh.build(world);  // static BVH must be rebuilt
      for (const Shape& q : queries) {
        out.clear();
        hits += strategy == 0 ? bf.query(q, out) : strategy == 1 ? bvh.query(q, out) : tree.query(q, out);
      }
    }
    const double ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count() / frames;
    return std::make_pair(ms, hits);
  };
  const auto b0 = simulate(0), b1 = simulate(1), b2 = simulate(2);
  const std::tuple<const char*, std::pair<double, std::size_t>> drows[] = {
      {"brute_force", b0}, {"bvh_rebuild_each_frame", b1}, {"aabb_tree_incremental", b2}};
  for (const auto& [name, r] : drows) {
    std::fprintf(dyn.f, "%s,%d,%d,%d,%d,%.3f,%zu\n", name, n, movers, per_frame_queries, frames, r.first, r.second);
    std::fprintf(md.f, "| %s | %.3f | %s |\n", name, r.first, r.second == b0.second ? "yes" : "NO");
    std::printf("%-24s %.3f ms/frame hits %zu\n", name, r.first, r.second);
  }
  return 0;
}
