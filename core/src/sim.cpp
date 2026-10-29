#include "qkrt/sim.hpp"

#include <algorithm>
#include <cmath>

namespace qkrt {

DiffDriveSim::DiffDriveSim(const SimConfig& cfg)
    : cfg_(cfg), kin_(cfg.wheel_radius, cfg.track_width, cfg.icr_factor), rng_(cfg.seed) {
  reset({});
}

void DiffDriveSim::reset(const Pose2& pose) {
  truth_ = GroundTruth{};
  truth_.pose = pose;
  truth_.gyro_bias = cfg_.gyro_bias_init_std * n01_(rng_);
  truth_.accel_bias = cfg_.accel_bias_std * n01_(rng_);
  t_ = 0.0;
  angle_left_ = angle_right_ = 0.0;
  ticks_left_ = ticks_right_ = 0;
  next_gps_t_ = 0.0;
  burst_until_ = -1.0;
}

void DiffDriveSim::updateSlip(double dt) {
  double sl = 0.0, sr = 0.0;
  const Vec2 p = truth_.pose.position();
  for (const auto& z : cfg_.slip_zones) {
    if (distance(p, z.center) <= z.radius) {
      sl = std::max(sl, z.slip_left);
      sr = std::max(sr, z.slip_right);
    }
  }
  if (cfg_.slip_event_rate > 0.0) {
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    if (t_ >= burst_until_ && u01(rng_) < cfg_.slip_event_rate * dt) {
      burst_until_ = t_ + 0.3 + 0.7 * u01(rng_);
      const int which = static_cast<int>(u01(rng_) * 3.0);  // 0 left, 1 right, 2 both
      const double mag = 0.2 + 0.5 * u01(rng_);
      burst_left_ = (which != 1) ? mag : 0.0;
      burst_right_ = (which != 0) ? mag : 0.0;
    }
    if (t_ < burst_until_) {
      sl = std::max(sl, burst_left_);
      sr = std::max(sr, burst_right_);
    }
  }
  truth_.slip_left = sl;
  truth_.slip_right = sr;
  truth_.slipping = std::max(sl, sr) > 0.05;
}

SensorFrame DiffDriveSim::step(double duty_left, double duty_right, double dt) {
  duty_left = std::clamp(duty_left, -1.0, 1.0);
  duty_right = std::clamp(duty_right, -1.0, 1.0);
  updateSlip(dt);

  // Motor dynamics. A slipping wheel sees less load torque and spins up faster; modelled as a
  // shorter time constant proportional to the remaining traction.
  const double tau_l = cfg_.motor_tau * (1.0 - 0.5 * truth_.slip_left);
  const double tau_r = cfg_.motor_tau * (1.0 - 0.5 * truth_.slip_right);
  truth_.wheel_left += dt * (cfg_.max_wheel_speed * duty_left - truth_.wheel_left) / tau_l;
  truth_.wheel_right += dt * (cfg_.max_wheel_speed * duty_right - truth_.wheel_right) / tau_r;

  // Ground contact speeds reduced by slip.
  const WheelSpeeds ground{(1.0 - truth_.slip_left) * truth_.wheel_left,
                           (1.0 - truth_.slip_right) * truth_.wheel_right};
  const Twist2 tw = kin_.forward(ground);
  const double v_prev = truth_.v;
  truth_.pose = integrateArc(truth_.pose, tw.v, tw.w, dt);
  truth_.v = tw.v;
  truth_.w = tw.w;
  t_ += dt;

  // Encoders measure wheel rotation (not ground speed), quantised to counts.
  angle_left_ += truth_.wheel_left * dt;
  angle_right_ += truth_.wheel_right * dt;
  const double cnt_per_rad = cfg_.encoder_cpr / (2.0 * kPi);
  const long tl = static_cast<long>(std::floor(angle_left_ * cnt_per_rad));
  const long tr = static_cast<long>(std::floor(angle_right_ * cnt_per_rad));
  SensorFrame s;
  s.t = t_;
  s.enc_left = (tl - ticks_left_) / cnt_per_rad / dt;
  s.enc_right = (tr - ticks_right_) / cnt_per_rad / dt;
  ticks_left_ = tl;
  ticks_right_ = tr;

  truth_.gyro_bias += cfg_.gyro_bias_walk * std::sqrt(dt) * n01_(rng_);
  s.gyro_z = truth_.w + truth_.gyro_bias + cfg_.gyro_noise_std * n01_(rng_);
  s.accel_x = (truth_.v - v_prev) / dt + truth_.accel_bias + cfg_.accel_noise_std * n01_(rng_);

  if (cfg_.gps_rate_hz > 0.0 && t_ + 1e-12 >= next_gps_t_) {
    next_gps_t_ += 1.0 / cfg_.gps_rate_hz;
    s.gps_valid = true;
    s.gps_x = truth_.pose.x + cfg_.gps_noise_std * n01_(rng_);
    s.gps_y = truth_.pose.y + cfg_.gps_noise_std * n01_(rng_);
  }
  return s;
}

}  // namespace qkrt
