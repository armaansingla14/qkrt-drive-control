#pragma once

#include "qkrt/types.hpp"

namespace qkrt {

/// Wheel angular speeds [rad/s].
struct WheelSpeeds {
  double left = 0.0;
  double right = 0.0;
};

/// Differential / skid-steer kinematics.
///
/// For a skid-steer chassis the effective track width is larger than the geometric
/// one because the wheels scrub sideways while turning. This is modelled with the
/// instantaneous-centre-of-rotation (ICR) expansion factor `icr_factor` (chi >= 1,
/// chi == 1 for an ideal differential drive).
class DiffDriveKinematics {
 public:
  DiffDriveKinematics(double wheel_radius, double track_width, double icr_factor = 1.0);

  Twist2 forward(const WheelSpeeds& ws) const;
  WheelSpeeds inverse(const Twist2& tw) const;

  /// Scale both wheels so that neither exceeds `max_wheel_speed`, preserving the path curvature.
  static WheelSpeeds desaturate(const WheelSpeeds& ws, double max_wheel_speed);

  double wheelRadius() const { return r_; }
  double trackWidth() const { return b_; }
  double effectiveTrack() const { return b_ * chi_; }
  double icrFactor() const { return chi_; }

 private:
  double r_;
  double b_;
  double chi_;
};

inline DiffDriveKinematics makeSkidSteer(double wheel_radius, double track_width, double icr_factor) {
  return DiffDriveKinematics(wheel_radius, track_width, icr_factor);
}

/// Dead-reckoning from wheel angle increments.
class WheelOdometry {
 public:
  explicit WheelOdometry(const DiffDriveKinematics& kin) : kin_(kin) {}
  void reset(const Pose2& p) { pose_ = p; }
  /// dl, dr: wheel rotation increments [rad] over dt.
  const Pose2& update(double dl, double dr);
  const Pose2& pose() const { return pose_; }

 private:
  DiffDriveKinematics kin_;
  Pose2 pose_;
};

}  // namespace qkrt
