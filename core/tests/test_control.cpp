#include <algorithm>

#include "qkrt/kinematics.hpp"
#include "qkrt/pid.hpp"
#include "test_harness.hpp"

using namespace qkrt;

namespace {
// First-order motor: dw/dt = (k*u - w)/tau, u in [-1, 1].
double simulateStep(AntiWindup aw, double setpoint, double* overshoot_out) {
  PidConfig cfg;
  cfg.kp = 0.15;
  cfg.ki = 2.0;
  cfg.anti_windup = aw;
  Pid pid(cfg);
  const double k = 20.0, tau = 0.08, dt = 0.001;
  double w = 0.0, peak = 0.0, err_int = 0.0;
  for (int i = 0; i < 3000; ++i) {
    const double u = pid.update(setpoint, w, dt);
    w += dt * (k * u - w) / tau;
    peak = std::max(peak, w);
    err_int += std::abs(setpoint - w) * dt;
  }
  *overshoot_out = (peak - setpoint) / setpoint;
  return err_int;
}
}  // namespace

TEST_CASE("pid proportional only") {
  PidConfig cfg;
  cfg.kp = 2.0;
  cfg.out_min = -10;
  cfg.out_max = 10;
  Pid pid(cfg);
  CHECK_NEAR(pid.update(1.0, 0.25, 0.01), 1.5, 1e-12);
}

TEST_CASE("pid output saturates") {
  PidConfig cfg;
  cfg.kp = 100.0;
  Pid pid(cfg);
  CHECK_NEAR(pid.update(1.0, 0.0, 0.01), 1.0, 1e-12);
  CHECK(pid.saturated());
  CHECK_NEAR(pid.update(-1.0, 0.0, 0.01), -1.0, 1e-12);
}

TEST_CASE("pid integral removes steady-state error") {
  PidConfig cfg;
  cfg.kp = 0.1;
  cfg.ki = 1.0;
  Pid pid(cfg);
  double w = 0.0;
  for (int i = 0; i < 20000; ++i) {
    const double u = pid.update(5.0, w, 0.001);
    w += 0.001 * (20.0 * u - w) / 0.08;
  }
  CHECK_NEAR(w, 5.0, 1e-3);
}

TEST_CASE("anti-windup reduces overshoot under saturation") {
  double os_none = 0, os_clamp = 0, os_bc = 0;
  simulateStep(AntiWindup::None, 18.0, &os_none);
  simulateStep(AntiWindup::Clamping, 18.0, &os_clamp);
  simulateStep(AntiWindup::BackCalculation, 18.0, &os_bc);
  CHECK(os_none > 0.05);
  CHECK(os_clamp < os_none);
  CHECK(os_bc < os_none);
}

TEST_CASE("derivative on measurement has no setpoint kick") {
  PidConfig cfg;
  cfg.kp = 0.0;
  cfg.kd = 1.0;
  cfg.out_min = -1e9;
  cfg.out_max = 1e9;
  Pid pid(cfg);
  pid.update(0.0, 0.0, 0.01);
  CHECK_NEAR(pid.update(10.0, 0.0, 0.01), 0.0, 1e-12);
  cfg.derivative_on_measurement = false;
  Pid pid2(cfg);
  pid2.update(0.0, 0.0, 0.01);
  CHECK_NEAR(pid2.update(10.0, 0.0, 0.01), 1000.0, 1e-9);
}

TEST_CASE("diff drive forward/inverse round trip") {
  DiffDriveKinematics kin(0.1, 0.5);
  const Twist2 tw{0.8, -0.6};
  const WheelSpeeds ws = kin.inverse(tw);
  const Twist2 back = kin.forward(ws);
  CHECK_NEAR(back.v, tw.v, 1e-12);
  CHECK_NEAR(back.w, tw.w, 1e-12);
  CHECK_NEAR(kin.forward({10.0, 10.0}).w, 0.0, 1e-12);
  CHECK_NEAR(kin.forward({-5.0, 5.0}).v, 0.0, 1e-12);
}

TEST_CASE("skid steer needs more wheel differential to turn") {
  DiffDriveKinematics diff(0.1, 0.5, 1.0);
  DiffDriveKinematics skid = makeSkidSteer(0.1, 0.5, 1.6);
  const WheelSpeeds a = diff.inverse({0.0, 1.0});
  const WheelSpeeds b = skid.inverse({0.0, 1.0});
  CHECK_NEAR(b.right / a.right, 1.6, 1e-12);
  CHECK_NEAR(skid.effectiveTrack(), 0.8, 1e-12);
}

TEST_CASE("desaturation preserves curvature") {
  DiffDriveKinematics kin(0.1, 0.5);
  const Twist2 tw{3.0, 2.0};
  const WheelSpeeds ws = DiffDriveKinematics::desaturate(kin.inverse(tw), 20.0);
  CHECK(std::max(std::abs(ws.left), std::abs(ws.right)) <= 20.0 + 1e-12);
  const Twist2 out = kin.forward(ws);
  CHECK_NEAR(out.w / out.v, tw.w / tw.v, 1e-12);
}

TEST_CASE("odometry drives a full circle") {
  DiffDriveKinematics kin(0.1, 0.5);
  WheelOdometry odo(kin);
  const WheelSpeeds ws = kin.inverse({1.0, 1.0});  // radius 1 m
  const int n = 1000;
  const double T = 2.0 * kPi;  // one revolution
  for (int i = 0; i < n; ++i) odo.update(ws.left * T / n, ws.right * T / n);
  CHECK_NEAR(odo.pose().x, 0.0, 1e-9);
  CHECK_NEAR(odo.pose().y, 0.0, 1e-9);
}

TEST_CASE("angle wrap") {
  CHECK_NEAR(wrapAngle(3.0 * kPi), -kPi, 1e-12);
  CHECK_NEAR(wrapAngle(-0.5), -0.5, 1e-12);
  CHECK_NEAR(wrapAngle(2.0 * kPi + 0.25), 0.25, 1e-12);
}

TEST_MAIN()
