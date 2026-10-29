#pragma once

#include <cmath>

namespace qkrt {

constexpr double kPi = 3.14159265358979323846;

/// Wrap an angle to [-pi, pi).
inline double wrapAngle(double a) {
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a < 0.0) a += 2.0 * kPi;
  return a - kPi;
}

struct Vec2 {
  double x = 0.0;
  double y = 0.0;

  constexpr Vec2() = default;
  constexpr Vec2(double x_, double y_) : x(x_), y(y_) {}

  Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
  Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
  Vec2 operator*(double s) const { return {x * s, y * s}; }
  Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
  double dot(const Vec2& o) const { return x * o.x + y * o.y; }
  double cross(const Vec2& o) const { return x * o.y - y * o.x; }
  double norm() const { return std::hypot(x, y); }
  double squaredNorm() const { return x * x + y * y; }
};

inline double distance(const Vec2& a, const Vec2& b) { return (a - b).norm(); }

struct Pose2 {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  Vec2 position() const { return {x, y}; }
};

/// Body-frame planar twist: forward speed v [m/s], yaw rate w [rad/s].
struct Twist2 {
  double v = 0.0;
  double w = 0.0;
};

/// Exact constant-twist (arc) integration of a planar pose.
inline Pose2 integrateArc(const Pose2& p, double v, double w, double dt) {
  Pose2 out = p;
  const double dth = w * dt;
  if (std::abs(dth) < 1e-9) {
    out.x += v * dt * std::cos(p.theta + 0.5 * dth);
    out.y += v * dt * std::sin(p.theta + 0.5 * dth);
  } else {
    const double r = v / w;
    out.x += r * (std::sin(p.theta + dth) - std::sin(p.theta));
    out.y += -r * (std::cos(p.theta + dth) - std::cos(p.theta));
  }
  out.theta = wrapAngle(p.theta + dth);
  return out;
}

}  // namespace qkrt
