#pragma once

#include "qkrt/kinematics.hpp"
#include "qkrt/pid.hpp"

namespace qkrt {

struct DriveControllerConfig {
  double wheel_radius = 0.1;
  double track_width = 0.5;
  double icr_factor = 1.0;
  double max_wheel_speed = 20.0;  ///< [rad/s] at full duty
  PidConfig wheel_pid{0.15, 2.0, 0.0, -1.0, 1.0, AntiWindup::BackCalculation, 0.0, 0.0, true};
  bool use_feedforward = true;    ///< duty feed-forward = setpoint / max_wheel_speed
};

struct DriveOutput {
  WheelSpeeds setpoint;  ///< [rad/s]
  double duty_left = 0.0;
  double duty_right = 0.0;
};

/// Body twist -> wheel speed setpoints (with curvature-preserving desaturation) -> per-wheel
/// velocity PID -> duty commands for the motor controllers.
class DriveController {
 public:
  explicit DriveController(const DriveControllerConfig& cfg = {});
  DriveOutput update(const Twist2& cmd, const WheelSpeeds& measured, double dt);
  void reset();
  const DiffDriveKinematics& kinematics() const { return kin_; }
  const DriveControllerConfig& config() const { return cfg_; }

 private:
  DriveControllerConfig cfg_;
  DiffDriveKinematics kin_;
  Pid left_, right_;
};

}  // namespace qkrt
