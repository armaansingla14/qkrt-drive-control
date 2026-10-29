#include "qkrt/kinematics.hpp"

#include <algorithm>
#include <stdexcept>

namespace qkrt {

DiffDriveKinematics::DiffDriveKinematics(double wheel_radius, double track_width, double icr_factor)
    : r_(wheel_radius), b_(track_width), chi_(icr_factor) {
  if (r_ <= 0.0 || b_ <= 0.0 || chi_ < 1.0) {
    throw std::invalid_argument("DiffDriveKinematics: radius/track must be > 0 and icr_factor >= 1");
  }
}

Twist2 DiffDriveKinematics::forward(const WheelSpeeds& ws) const {
  const double vl = r_ * ws.left;
  const double vr = r_ * ws.right;
  return {0.5 * (vl + vr), (vr - vl) / effectiveTrack()};
}

WheelSpeeds DiffDriveKinematics::inverse(const Twist2& tw) const {
  const double half = 0.5 * effectiveTrack() * tw.w;
  return {(tw.v - half) / r_, (tw.v + half) / r_};
}

WheelSpeeds DiffDriveKinematics::desaturate(const WheelSpeeds& ws, double max_wheel_speed) {
  const double peak = std::max(std::abs(ws.left), std::abs(ws.right));
  if (peak <= max_wheel_speed || peak <= 0.0) return ws;
  const double s = max_wheel_speed / peak;
  return {ws.left * s, ws.right * s};
}

const Pose2& WheelOdometry::update(double dl, double dr) {
  // Treat the increments as a constant twist over a unit interval.
  const Twist2 tw = kin_.forward({dl, dr});
  pose_ = integrateArc(pose_, tw.v, tw.w, 1.0);
  return pose_;
}

}  // namespace qkrt
