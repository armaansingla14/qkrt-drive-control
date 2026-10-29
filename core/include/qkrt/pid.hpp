#pragma once

#include <limits>

namespace qkrt {

enum class AntiWindup {
  None,             ///< plain integrator (for comparison only)
  Clamping,         ///< conditional integration: freeze I when saturated and error pushes further
  BackCalculation,  ///< feed saturation error back into the integrator with gain kb
};

struct PidConfig {
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double out_min = -1.0;
  double out_max = 1.0;
  AntiWindup anti_windup = AntiWindup::BackCalculation;
  /// Back-calculation gain [1/s]. If <= 0 it is derived as 1/Tt with Tt = sqrt(Ti*Td) (or Ti if kd==0).
  double kb = 0.0;
  /// First-order low-pass on the derivative term [Hz]. <= 0 disables the filter.
  double d_cutoff_hz = 0.0;
  /// Take the derivative of the measurement instead of the error (no derivative kick on setpoint steps).
  bool derivative_on_measurement = true;
};

/// Discrete PID controller intended for wheel-velocity loops (duty-cycle output).
class Pid {
 public:
  explicit Pid(const PidConfig& cfg = {});

  /// One controller step. `feedforward` is added before saturation.
  double update(double setpoint, double measurement, double dt, double feedforward = 0.0);
  void reset();

  double integral() const { return integral_; }
  bool saturated() const { return saturated_; }
  double lastUnsaturated() const { return last_unsat_; }
  const PidConfig& config() const { return cfg_; }
  void setConfig(const PidConfig& cfg);

 private:
  double backCalcGain() const;

  PidConfig cfg_;
  double integral_ = 0.0;
  double prev_error_ = 0.0;
  double prev_measurement_ = 0.0;
  double d_state_ = 0.0;
  double last_unsat_ = 0.0;
  bool saturated_ = false;
  bool first_ = true;
};

}  // namespace qkrt
