#include "qkrt/pid.hpp"
#include "qkrt/types.hpp"

#include <algorithm>
#include <cmath>

namespace qkrt {

Pid::Pid(const PidConfig& cfg) : cfg_(cfg) {}

void Pid::setConfig(const PidConfig& cfg) { cfg_ = cfg; }

void Pid::reset() {
  integral_ = 0.0;
  prev_error_ = 0.0;
  prev_measurement_ = 0.0;
  d_state_ = 0.0;
  last_unsat_ = 0.0;
  saturated_ = false;
  first_ = true;
}

double Pid::backCalcGain() const {
  if (cfg_.kb > 0.0) return cfg_.kb;
  if (cfg_.ki <= 0.0 || cfg_.kp <= 0.0) return cfg_.ki > 0.0 ? cfg_.ki : 0.0;
  const double ti = cfg_.kp / cfg_.ki;
  const double td = cfg_.kd / cfg_.kp;
  const double tt = td > 0.0 ? std::sqrt(ti * td) : ti;
  return 1.0 / tt;
}

double Pid::update(double setpoint, double measurement, double dt, double feedforward) {
  if (dt <= 0.0) return std::clamp(last_unsat_, cfg_.out_min, cfg_.out_max);
  const double error = setpoint - measurement;

  // Derivative term (optionally on measurement, optionally low-pass filtered).
  double d_raw = 0.0;
  if (!first_) {
    d_raw = cfg_.derivative_on_measurement ? -(measurement - prev_measurement_) / dt
                                           : (error - prev_error_) / dt;
  }
  if (cfg_.d_cutoff_hz > 0.0) {
    const double tau = 1.0 / (2.0 * kPi * cfg_.d_cutoff_hz);
    const double alpha = dt / (tau + dt);
    d_state_ += alpha * (d_raw - d_state_);
  } else {
    d_state_ = d_raw;
  }
  first_ = false;
  prev_error_ = error;
  prev_measurement_ = measurement;

  const double p_term = cfg_.kp * error;
  const double d_term = cfg_.kd * d_state_;
  const double u_unsat = feedforward + p_term + integral_ + d_term;
  const double u = std::clamp(u_unsat, cfg_.out_min, cfg_.out_max);
  saturated_ = (u != u_unsat);
  last_unsat_ = u_unsat;

  // Integrator update for the next step.
  switch (cfg_.anti_windup) {
    case AntiWindup::None:
      integral_ += cfg_.ki * error * dt;
      break;
    case AntiWindup::Clamping: {
      const bool pushing_high = (u_unsat > cfg_.out_max) && (error > 0.0);
      const bool pushing_low = (u_unsat < cfg_.out_min) && (error < 0.0);
      if (!pushing_high && !pushing_low) integral_ += cfg_.ki * error * dt;
      break;
    }
    case AntiWindup::BackCalculation:
      integral_ += (cfg_.ki * error + backCalcGain() * (u - u_unsat)) * dt;
      break;
  }
  return u;
}

}  // namespace qkrt
