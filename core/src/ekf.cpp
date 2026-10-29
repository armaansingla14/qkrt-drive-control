#include "qkrt/ekf.hpp"

#include <algorithm>
#include <cmath>

namespace qkrt {

LocalizationEkf::LocalizationEkf(const EkfConfig& cfg) : cfg_(cfg) { reset({}); }

void LocalizationEkf::reset(const Pose2& pose, double pos_std, double theta_std) {
  x_.setZero();
  x_(0) = pose.x;
  x_(1) = pose.y;
  x_(2) = pose.theta;
  P_.setZero();
  P_.diagonal() << pos_std * pos_std, pos_std * pos_std, theta_std * theta_std, 0.01, 0.01, 1e-4;
}

void LocalizationEkf::predict(double dt, double accel_x) {
  const double th = x_(2), v = x_(3), w = x_(4);
  const double c = std::cos(th), s = std::sin(th);
  x_(0) += v * c * dt;
  x_(1) += v * s * dt;
  x_(2) = wrapAngle(th + w * dt);
  x_(3) += accel_x * dt;

  Mat6 F = Mat6::Identity();
  F(0, 2) = -v * s * dt;
  F(0, 3) = c * dt;
  F(1, 2) = v * c * dt;
  F(1, 3) = s * dt;
  F(2, 4) = dt;

  Vec6 q;
  q << cfg_.q_pos, cfg_.q_pos, cfg_.q_theta, cfg_.q_v, cfg_.q_w, cfg_.q_bias;
  P_ = F * P_ * F.transpose();
  P_.diagonal() += q * dt;
}

template <int M>
double LocalizationEkf::update(const Eigen::Matrix<double, M, 1>& z, const Eigen::Matrix<double, M, 6>& H,
                               const Eigen::Matrix<double, M, M>& R, int angle_row) {
  Eigen::Matrix<double, M, 1> y = z - H * x_;
  if (angle_row >= 0) y(angle_row) = wrapAngle(y(angle_row));
  const Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
  const Eigen::Matrix<double, M, M> S_inv = S.inverse();
  const Eigen::Matrix<double, 6, M> K = P_ * H.transpose() * S_inv;
  x_ += K * y;
  x_(2) = wrapAngle(x_(2));
  // Joseph form keeps P symmetric positive semi-definite.
  const Mat6 IKH = Mat6::Identity() - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
  return (y.transpose() * S_inv * y)(0, 0);
}

double LocalizationEkf::updateGyro(double gyro_z) {
  Eigen::Matrix<double, 1, 1> z;
  z << gyro_z;
  Eigen::Matrix<double, 1, 6> H = Eigen::Matrix<double, 1, 6>::Zero();
  H(0, 4) = 1.0;
  H(0, 5) = 1.0;
  Eigen::Matrix<double, 1, 1> R;
  R << cfg_.gyro_std * cfg_.gyro_std;
  return update<1>(z, H, R);
}

double LocalizationEkf::updateGps(double x, double y) {
  Eigen::Vector2d z(x, y);
  Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero();
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  const Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * cfg_.gps_std * cfg_.gps_std;
  return update<2>(z, H, R);
}

namespace {
Eigen::Matrix<double, 2, 6> odomH() {
  Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero();
  H(0, 3) = 1.0;
  H(1, 4) = 1.0;
  return H;
}
}  // namespace

double LocalizationEkf::odometryNis(double v_enc, double w_enc) const {
  const Eigen::Matrix<double, 2, 6> H = odomH();
  Eigen::Vector2d R(cfg_.odom_v_std * cfg_.odom_v_std, cfg_.odom_w_std * cfg_.odom_w_std);
  const Eigen::Vector2d y = Eigen::Vector2d(v_enc, w_enc) - H * x_;
  const Eigen::Matrix2d S = H * P_ * H.transpose() + Eigen::Matrix2d(R.asDiagonal());
  return y.dot(S.ldlt().solve(y));
}

double LocalizationEkf::updateWheelOdometry(double v_enc, double w_enc, bool slip) {
  const double k = slip ? cfg_.slip_inflation : 1.0;
  Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
  R(0, 0) = k * cfg_.odom_v_std * cfg_.odom_v_std;
  R(1, 1) = k * cfg_.odom_w_std * cfg_.odom_w_std;
  return update<2>(Eigen::Vector2d(v_enc, w_enc), odomH(), R);
}

// ---------------------------------------------------------------------------

void WheelSlipDetector::reset(double v0) {
  v_imu_ = v0;
  r_yaw_ = r_speed_ = 0.0;
  hold_ = 0.0;
  slipping_ = false;
  initialised_ = true;
}

void WheelSlipDetector::force(double dt_hold) {
  hold_ = std::max(hold_, dt_hold);
  slipping_ = true;
}

bool WheelSlipDetector::update(double v_enc, double w_enc, double gyro_corrected, double accel_x, double dt) {
  if (!initialised_) reset(v_enc);
  v_imu_ += accel_x * dt;
  const double alpha = dt / (cfg_.filter_tau + dt);
  r_yaw_ += alpha * ((w_enc - gyro_corrected) - r_yaw_);
  r_speed_ += alpha * ((v_enc - v_imu_) - r_speed_);

  const bool yaw_bad = std::abs(r_yaw_) > cfg_.yaw_residual_threshold;
  const bool speed_bad = std::abs(r_speed_) > cfg_.speed_residual_abs + cfg_.speed_residual_rel * std::abs(v_enc);
  if (yaw_bad || speed_bad) {
    hold_ = cfg_.hold_time;
  } else {
    hold_ = std::max(0.0, hold_ - dt);
  }
  slipping_ = yaw_bad || speed_bad || hold_ > 0.0;

  // Re-anchor the IMU speed to the encoders only while traction is trusted.
  if (!slipping_) v_imu_ += (v_enc - v_imu_) * std::min(1.0, dt / cfg_.anchor_tau);
  return slipping_;
}

// ---------------------------------------------------------------------------

Localizer::Localizer(LocalizerMode mode, const DiffDriveKinematics& kin, const EkfConfig& ekf,
                     const SlipDetectorConfig& det)
    : mode_(mode), kin_(kin), ekf_(ekf), detector_(det) {}

void Localizer::reset(const Pose2& pose) {
  ekf_.reset(pose);
  detector_.reset(0.0);
  dr_pose_ = pose;
  slip_flag_ = false;
}

void Localizer::process(const SensorFrame& s, double dt) {
  const Twist2 enc = kin_.forward({s.enc_left, s.enc_right});
  if (mode_ == LocalizerMode::DeadReckoning) {
    dr_pose_ = integrateArc(dr_pose_, enc.v, enc.w, dt);
    return;
  }
  ekf_.predict(dt, s.accel_x);
  ekf_.updateGyro(s.gyro_z);
  slip_flag_ = false;
  if (mode_ == LocalizerMode::EkfSlipAware) {
    slip_flag_ = detector_.update(enc.v, enc.w, s.gyro_z - ekf_.gyroBias(), s.accel_x, dt);
    if (!slip_flag_ && ekf_.odometryNis(enc.v, enc.w) > ekf_.config().nis_gate) {
      detector_.force(0.1);
      slip_flag_ = true;
    }
  }
  ekf_.updateWheelOdometry(enc.v, enc.w, slip_flag_);
  if (s.gps_valid) ekf_.updateGps(s.gps_x, s.gps_y);
}

Pose2 Localizer::pose() const {
  return mode_ == LocalizerMode::DeadReckoning ? dr_pose_ : ekf_.pose();
}

}  // namespace qkrt
