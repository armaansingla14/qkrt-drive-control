#pragma once

#include <Eigen/Dense>

#include "qkrt/kinematics.hpp"
#include "qkrt/sim.hpp"
#include "qkrt/types.hpp"

namespace qkrt {

struct EkfConfig {
  // Process noise spectral densities (variance per second).
  double q_pos = 1e-4;
  double q_theta = 1e-5;
  double q_v = 0.05;      ///< velocity random walk on top of IMU-acceleration propagation
  double q_w = 2.0;       ///< yaw-rate random walk (yaw acceleration)
  double q_bias = 1e-7;   ///< gyro bias random walk
  // Measurement noise standard deviations.
  double odom_v_std = 0.03;   ///< [m/s]
  double odom_w_std = 0.06;   ///< [rad/s] base yaw-rate noise
  /// Additional yaw-rate std per m/s of speed: wheel-odometry yaw rate degrades with speed
  /// (radius/track errors, skid). Keeps the gyro-bias state from absorbing odometry errors.
  double odom_w_std_per_mps = 0.25;
  double stationary_speed = 0.02;    ///< [m/s] encoders below this => zero-rate update
  double stationary_w_std = 0.002;   ///< [rad/s] yaw-rate std when stationary
  double gyro_std = 0.006;    ///< [rad/s]
  double gps_std = 0.5;       ///< [m]
  // Slip handling.
  double slip_inflation = 1e4;  ///< odometry covariance multiplier while slip is detected
  double nis_gate = 9.21;       ///< chi-square 2 dof, 99 %
};

/// Planar EKF with state [x, y, theta, v, omega, gyro_bias].
///
/// Prediction integrates the IMU longitudinal acceleration into v. Wheel odometry
/// observes (v, omega), the gyro observes omega + bias, GPS-like fixes observe (x, y).
class LocalizationEkf {
 public:
  using Vec6 = Eigen::Matrix<double, 6, 1>;
  using Mat6 = Eigen::Matrix<double, 6, 6>;

  explicit LocalizationEkf(const EkfConfig& cfg = {});

  void reset(const Pose2& pose, double pos_std = 0.05, double theta_std = 0.02);
  void predict(double dt, double accel_x);
  /// Returns the normalised innovation squared of the update.
  double updateGyro(double gyro_z);
  double updateGps(double x, double y);
  /// NIS of a wheel-odometry measurement against the current prior (no state change).
  double odometryNis(double v_enc, double w_enc) const;
  double updateWheelOdometry(double v_enc, double w_enc, bool slip);

  Pose2 pose() const { return {x_(0), x_(1), x_(2)}; }
  double velocity() const { return x_(3); }
  double yawRate() const { return x_(4); }
  double gyroBias() const { return x_(5); }
  const Vec6& state() const { return x_; }
  const Mat6& covariance() const { return P_; }
  const EkfConfig& config() const { return cfg_; }

 private:
  Eigen::Matrix2d odomR(double v_enc, double w_enc, bool slip) const;
  template <int M>
  double update(const Eigen::Matrix<double, M, 1>& z, const Eigen::Matrix<double, M, 6>& H,
                const Eigen::Matrix<double, M, M>& R, int angle_row = -1);

  EkfConfig cfg_;
  Vec6 x_ = Vec6::Zero();
  Mat6 P_ = Mat6::Identity();
};

struct SlipDetectorConfig {
  double yaw_residual_threshold = 0.10;  ///< |omega_enc - omega_gyro| [rad/s]
  double speed_residual_abs = 0.08;      ///< |v_enc - v_imu| [m/s]
  double speed_residual_rel = 0.10;      ///< ... plus this fraction of |v_enc|
  double filter_tau = 0.05;              ///< residual low-pass time constant [s]
  double anchor_tau = 0.5;               ///< IMU-velocity re-anchoring time constant [s]
  double hold_time = 0.3;                ///< keep the flag this long after residuals clear [s]
};

/// Detects wheel slip from disagreement between encoder kinematics and the IMU.
///  - yaw channel: encoder yaw rate vs bias-corrected gyro
///  - longitudinal channel: encoder speed vs an IMU-integrated speed that is slowly
///    re-anchored to the encoders only while no slip is flagged.
class WheelSlipDetector {
 public:
  explicit WheelSlipDetector(const SlipDetectorConfig& cfg = {}) : cfg_(cfg) {}
  void reset(double v0 = 0.0);
  bool update(double v_enc, double w_enc, double gyro_corrected, double accel_x, double dt);
  void force(double dt_hold);
  bool slipping() const { return slipping_; }
  double yawResidual() const { return r_yaw_; }
  double speedResidual() const { return r_speed_; }

 private:
  SlipDetectorConfig cfg_;
  double v_imu_ = 0.0;
  double r_yaw_ = 0.0;
  double r_speed_ = 0.0;
  double hold_ = 0.0;
  bool slipping_ = false;
  bool initialised_ = false;
};

enum class LocalizerMode { DeadReckoning, EkfNaive, EkfSlipAware };

/// Full localization pipeline consuming raw sensor frames.
class Localizer {
 public:
  Localizer(LocalizerMode mode, const DiffDriveKinematics& kin, const EkfConfig& ekf = {},
            const SlipDetectorConfig& det = {});
  void reset(const Pose2& pose);
  void process(const SensorFrame& s, double dt);

  Pose2 pose() const;
  bool slipFlag() const { return slip_flag_; }
  LocalizerMode mode() const { return mode_; }
  const LocalizationEkf& ekf() const { return ekf_; }

 private:
  LocalizerMode mode_;
  DiffDriveKinematics kin_;
  LocalizationEkf ekf_;
  WheelSlipDetector detector_;
  Pose2 dr_pose_;
  bool slip_flag_ = false;
};

}  // namespace qkrt
