#include "qkrt/drive_controller.hpp"

namespace qkrt {

DriveController::DriveController(const DriveControllerConfig& cfg)
    : cfg_(cfg), kin_(cfg.wheel_radius, cfg.track_width, cfg.icr_factor), left_(cfg.wheel_pid), right_(cfg.wheel_pid) {}

void DriveController::reset() {
  left_.reset();
  right_.reset();
}

DriveOutput DriveController::update(const Twist2& cmd, const WheelSpeeds& measured, double dt) {
  DriveOutput out;
  out.setpoint = DiffDriveKinematics::desaturate(kin_.inverse(cmd), cfg_.max_wheel_speed);
  const double ffl = cfg_.use_feedforward ? out.setpoint.left / cfg_.max_wheel_speed : 0.0;
  const double ffr = cfg_.use_feedforward ? out.setpoint.right / cfg_.max_wheel_speed : 0.0;
  out.duty_left = left_.update(out.setpoint.left, measured.left, dt, ffl);
  out.duty_right = right_.update(out.setpoint.right, measured.right, dt, ffr);
  return out;
}

}  // namespace qkrt
