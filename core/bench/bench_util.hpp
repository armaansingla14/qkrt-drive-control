#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace bench {

using Clock = std::chrono::steady_clock;

inline double percentile(std::vector<double> v, double p) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const double idx = p / 100.0 * (v.size() - 1);
  const std::size_t lo = static_cast<std::size_t>(std::floor(idx));
  const std::size_t hi = std::min(lo + 1, v.size() - 1);
  return v[lo] + (v[hi] - v[lo]) * (idx - lo);
}

inline double mean(const std::vector<double>& v) {
  double s = 0.0;
  for (double x : v) s += x;
  return v.empty() ? 0.0 : s / v.size();
}

inline double stddev(const std::vector<double>& v) {
  if (v.size() < 2) return 0.0;
  const double m = mean(v);
  double s = 0.0;
  for (double x : v) s += (x - m) * (x - m);
  return std::sqrt(s / (v.size() - 1));
}

inline std::string outDir(int argc, char** argv) {
  for (int i = 1; i + 1 < argc; ++i)
    if (std::string(argv[i]) == "--out") return argv[i + 1];
  return "results";
}

inline bool hasFlag(int argc, char** argv, const char* flag) {
  for (int i = 1; i < argc; ++i)
    if (std::string(argv[i]) == flag) return true;
  return false;
}

struct File {
  FILE* f = nullptr;
  explicit File(const std::string& path) : f(std::fopen(path.c_str(), "w")) {
    if (!f) std::fprintf(stderr, "cannot open %s\n", path.c_str());
  }
  ~File() {
    if (f) std::fclose(f);
  }
};

}  // namespace bench
