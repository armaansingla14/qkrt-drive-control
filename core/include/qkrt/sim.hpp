#pragma once

#include <random>
#include <vector>

#include "qkrt/kinematics.hpp"
#include "qkrt/types.hpp"

namespace qkrt {

/// Circular patch of low-traction ground (mud, wet grass, loose gravel).
/// `slip_left/right` is the longitudinal slip ratio s: ground speed = (1 - s) * r * omega.
struct SlipZone {
  Vec2 center;
  double radius = 1.0;
  double slip_left = 0.4;
  double slip_right = 0.4;
};

struct SimConfig {
  double wheel_radius = 0.1;
  double track_width = 0.5;
  double icr_factor = 1.0;
  double motor_tau = 0.05;          ///< motor/wheel first-order time constant [s]
  double max_wheel_speed = 20.0;    ///< no-load wheel speed at full duty [rad/s]
  int encoder_cpr = 4096;           ///< encoder counts per wheel revolution
  double gyro_noise_std = 0.005;    ///< [rad/s]
  double gyro_bias_init_std = 0.01; ///< [rad/s]
  double gyro_bias_walk = 2e-4;     ///< [rad/s/sqrt(s)]
  double accel_noise_std = 0.05;    ///< [m/s^2]
  double accel_bias_std = 0.02;     ///< constant per-run accelerometer bias [m/s^2]
  double gps_noise_std = 0.5;       ///< [m] per axis
  double gps_rate_hz = 2.0;
  std::vector<SlipZone> slip_zones;
  double slip_event_rate = 0.0;     ///< random slip bursts per second (Poisson)
  unsigned seed = 1;
};

struct SensorFrame {
  double t = 0.0;
  double enc_left = 0.0;   ///< wheel speed from encoder count differences [rad/s]
  double enc_right = 0.0;
  double gyro_z = 0.0;     ///< [rad/s], includes bias + noise
  double accel_x = 0.0;    ///< body longitudinal acceleration [m/s^2]
  bool gps_valid = false;
  double gps_x = 0.0;
  double gps_y = 0.0;
};

struct GroundTruth {
  Pose2 pose;
  double v = 0.0;
  double w = 0.0;
  double wheel_left = 0.0;   ///< true wheel angular speed [rad/s]
  double wheel_right = 0.0;
  double slip_left = 0.0;
  double slip_right = 0.0;
  bool slipping = false;
  double gyro_bias = 0.0;
  double accel_bias = 0.0;
};

/// 2D differential-drive / skid-steer simulator with motor lag, wheel slip and noisy sensors.
class DiffDriveSim {
 public:
  explicit DiffDriveSim(const SimConfig& cfg);

  void reset(const Pose2& pose);
  /// Advance by dt with wheel duty commands in [-1, 1]; returns the sensor readings at the new time.
  SensorFrame step(double duty_left, double duty_right, double dt);

  const GroundTruth& truth() const { return truth_; }
  const SimConfig& config() const { return cfg_; }
  double time() const { return t_; }

 private:
  void updateSlip(double dt);

  SimConfig cfg_;
  DiffDriveKinematics kin_;
  std::mt19937 rng_;
  std::normal_distribution<double> n01_{0.0, 1.0};
  GroundTruth truth_;
  double t_ = 0.0;
  double angle_left_ = 0.0, angle_right_ = 0.0;
  long ticks_left_ = 0, ticks_right_ = 0;
  double next_gps_t_ = 0.0;
  // random slip burst state
  double burst_until_ = -1.0;
  double burst_left_ = 0.0, burst_right_ = 0.0;
};

}  // namespace qkrt
