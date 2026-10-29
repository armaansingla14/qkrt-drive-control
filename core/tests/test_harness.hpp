#pragma once
// Minimal self-contained test harness (no external dependencies).

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace qtest {

struct Case {
  std::string name;
  std::function<void()> fn;
};

inline std::vector<Case>& registry() {
  static std::vector<Case> r;
  return r;
}
inline int& failures() {
  static int f = 0;
  return f;
}

struct Registrar {
  Registrar(const char* name, std::function<void()> fn) { registry().push_back({name, std::move(fn)}); }
};

inline int runAll() {
  int failed_cases = 0;
  for (auto& c : registry()) {
    const int before = failures();
    c.fn();
    const bool ok = failures() == before;
    if (!ok) ++failed_cases;
    std::printf("[%s] %s\n", ok ? " OK " : "FAIL", c.name.c_str());
  }
  std::printf("%zu cases, %d failed\n", registry().size(), failed_cases);
  return failed_cases == 0 ? 0 : 1;
}

}  // namespace qtest

#define QT_CONCAT_(a, b) a##b
#define QT_CONCAT(a, b) QT_CONCAT_(a, b)
#define TEST_CASE(name)                                                  \
  static void QT_CONCAT(qt_fn_, __LINE__)();                            \
  static qtest::Registrar QT_CONCAT(qt_reg_, __LINE__)(name, QT_CONCAT(qt_fn_, __LINE__)); \
  static void QT_CONCAT(qt_fn_, __LINE__)()

#define CHECK(cond)                                                              \
  do {                                                                           \
    if (!(cond)) {                                                               \
      std::printf("  %s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond);     \
      ++qtest::failures();                                                       \
    }                                                                            \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                          \
  do {                                                                                 \
    const double qt_a = (a), qt_b = (b);                                               \
    if (!(std::abs(qt_a - qt_b) <= (tol))) {                                           \
      std::printf("  %s:%d: CHECK_NEAR(%s, %s) failed: %.9g vs %.9g (tol %.3g)\n",     \
                  __FILE__, __LINE__, #a, #b, qt_a, qt_b, (double)(tol));              \
      ++qtest::failures();                                                             \
    }                                                                                  \
  } while (0)

#define TEST_MAIN() \
  int main() { return qtest::runAll(); }
