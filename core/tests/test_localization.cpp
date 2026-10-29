#include <cmath>
#include <vector>

#include "qkrt/ekf.hpp"
#include "qkrt/sim.hpp"
#include "test_harness.hpp"

using namespace qkrt;

namespace {
SimConfig baseConfig(unsigned seed) {
  SimConfig c;
  c.seed = seed;
  return c;
}

// Open-loop duty commands that follow a gentle S-curve at ~1 m/s.
void dutyFor(const DiffDriveSim& sim, double t, double* dl, double* dr) {
  const auto& c = sim.config();
  DiffDriveKinematics kin(c.wheel_radius, c.track_width, c.icr_factor);
  const WheelSpeeds ws = kin.inverse({1.0, 0.5 * std::sin(0.3 * t)});
  *dl = ws.left / c.max_wheel_speed;
  *dr = ws.right / c.max_wheel_speed;
}

double runRmse(LocalizerMode mode, SimConfig cfg, double duration, int* slip_steps = nullptr,
               int* flagged_in_slip = nullptr, int* false_flags = nullptr) {
  DiffDriveSim sim(cfg);
  Localizer loc(mode, DiffDriveKinematics(cfg.wheel_radius, cfg.track_width, cfg.icr_factor));
  loc.reset({});
  const double dt = 0.01;
  double se = 0.0;
  int n = 0;
  for (double t = 0; t < duration; t += dt) {
    double dl, dr;
    dutyFor(sim, t, &dl, &dr);
    const SensorFrame s = sim.step(dl, dr, dt);
    loc.process(s, dt);
    const Pose2 e = loc.pose();
    const Pose2& g = sim.truth().pose;
    se += (e.x - g.x) * (e.x - g.x) + (e.y - g.y) * (e.y - g.y);
    ++n;
    if (slip_steps && sim.truth().slipping) ++*slip_steps;
    if (flagged_in_slip && sim.truth().slipping && loc.slipFlag()) ++*flagged_in_slip;
    if (false_flags && !sim.truth().slipping && loc.slipFlag()) ++*false_flags;
  }
  return std::sqrt(se / n);
}
}  // namespace

TEST_CASE("sim reaches commanded speed without slip") {
  DiffDriveSim sim(baseConfig(3));
  SensorFrame s;
  for (int i = 0; i < 200; ++i) s = sim.step(0.5, 0.5, 0.01);
  CHECK_NEAR(sim.truth().v, 1.0, 0.01);
  CHECK_NEAR(sim.truth().w, 0.0, 1e-9);
  CHECK_NEAR(0.5 * (s.enc_left + s.enc_right) * 0.1, 1.0, 0.03);
}

TEST_CASE("slip zone: encoders over-report ground speed") {
  SimConfig c = baseConfig(4);
  c.slip_zones.push_back({{0.0, 0.0}, 100.0, 0.5, 0.5});
  DiffDriveSim sim(c);
  SensorFrame s;
  for (int i = 0; i < 200; ++i) s = sim.step(0.5, 0.5, 0.01);
  CHECK(sim.truth().slipping);
  CHECK_NEAR(sim.truth().v, 0.5, 0.02);
  CHECK(0.5 * (s.enc_left + s.enc_right) * 0.1 > 0.9);
}

TEST_CASE("gps fixes arrive at configured rate") {
  SimConfig c = baseConfig(5);
  c.gps_rate_hz = 5.0;
  DiffDriveSim sim(c);
  int fixes = 0;
  for (int i = 0; i < 1000; ++i) fixes += sim.step(0.2, 0.2, 0.01).gps_valid ? 1 : 0;
  CHECK(fixes >= 49 && fixes <= 51);
}

TEST_CASE("ekf tracks without slip") {
  const double rmse = runRmse(LocalizerMode::EkfNaive, baseConfig(11), 60.0);
  CHECK(rmse < 0.5);
}

TEST_CASE("ekf gps update reduces position covariance") {
  LocalizationEkf ekf;
  ekf.reset({}, 5.0, 0.1);
  const double before = ekf.covariance()(0, 0);
  ekf.updateGps(1.0, -1.0);
  CHECK(ekf.covariance()(0, 0) < before);
  CHECK(ekf.pose().x > 0.5);
}

TEST_CASE("gyro bias is estimated when stationary") {
  LocalizationEkf ekf;
  ekf.reset({});
  for (int i = 0; i < 3000; ++i) {
    ekf.predict(0.01, 0.0);
    ekf.updateGyro(0.02);
    ekf.updateWheelOdometry(0.0, 0.0, false);
  }
  CHECK_NEAR(ekf.gyroBias(), 0.02, 2e-3);
}

TEST_CASE("slip detector flags asymmetric slip and rarely false-alarms") {
  SimConfig c = baseConfig(21);
  c.slip_event_rate = 0.2;
  int slip_steps = 0, flagged = 0, false_flags = 0;
  runRmse(LocalizerMode::EkfSlipAware, c, 120.0, &slip_steps, &flagged, &false_flags);
  CHECK(slip_steps > 200);
  const double recall = double(flagged) / slip_steps;
  const double false_rate = double(false_flags) / (12000 - slip_steps);
  CHECK(recall > 0.5);
  CHECK(false_rate < 0.1);
}

TEST_CASE("slip-aware ekf beats naive ekf under heavy slip (5-seed mean)") {
  double naive = 0, aware = 0;
  for (unsigned seed = 100; seed < 105; ++seed) {
    SimConfig c = baseConfig(seed);
    c.slip_event_rate = 0.3;
    c.gps_rate_hz = 1.0;
    c.gps_noise_std = 1.0;
    naive += runRmse(LocalizerMode::EkfNaive, c, 90.0);
    aware += runRmse(LocalizerMode::EkfSlipAware, c, 90.0);
  }
  CHECK(aware < naive);
}

TEST_MAIN()
