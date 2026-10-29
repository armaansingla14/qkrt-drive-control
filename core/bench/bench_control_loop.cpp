// Control-loop timing and PID anti-windup benchmark.
//
// 1. Runs the onboard stack (teleop gate, drive controller, simulator step, slip-aware EKF)
//    in a periodic loop paced with sleep_until and records the compute time of each cycle and
//    how late each cycle woke up relative to its deadline.
// 2. Measures per-component compute cost in a tight loop.
// 3. Compares anti-windup strategies on a saturating wheel-velocity step.
#include <cstdio>
#include <thread>

#include "bench_util.hpp"
#include "qkrt/drive_controller.hpp"
#include "qkrt/ekf.hpp"
#include "qkrt/sim.hpp"
#include "qkrt/teleop.hpp"

using namespace qkrt;
using bench::Clock;

namespace {

struct Stack {
  DiffDriveSim sim{SimConfig{}};
  DriveController ctrl;
  Localizer loc{LocalizerMode::EkfSlipAware, ctrl.kinematics()};
  teleop::TeleopGate gate;
  SensorFrame s;
  uint32_t seq = 0;
  Stack() { loc.reset({}); }

  void cycle(double t, double dt) {
    if (static_cast<int>(t * 1000) % 50 == 0) gate.handle({teleop::MsgType::Drive, ++seq, 1.0, 0.5 * std::sin(0.3 * t)}, t);
    const Twist2 cmd = gate.command(t, dt);
    const DriveOutput out = ctrl.update(cmd, {s.enc_left, s.enc_right}, dt);
    s = sim.step(out.duty_left, out.duty_right, dt);
    loc.process(s, dt);
  }
};

void periodicLoop(double rate_hz, double seconds, FILE* csv, FILE* md) {
  Stack stack;
  const auto period = std::chrono::nanoseconds(static_cast<long long>(1e9 / rate_hz));
  const double dt = 1.0 / rate_hz;
  const int n = static_cast<int>(seconds * rate_hz);
  std::vector<double> compute_us, wake_us;
  compute_us.reserve(n);
  wake_us.reserve(n);
  int overruns = 0;
  auto deadline = Clock::now() + period;
  for (int i = 0; i < n; ++i) {
    std::this_thread::sleep_until(deadline);
    const auto woke = Clock::now();
    stack.cycle(i * dt, dt);
    const auto done = Clock::now();
    wake_us.push_back(std::chrono::duration<double, std::micro>(woke - deadline).count());
    compute_us.push_back(std::chrono::duration<double, std::micro>(done - woke).count());
    deadline += period;
    if (done > deadline) {
      ++overruns;
      deadline = done + period;  // skip missed slots instead of bursting
    }
  }
  using bench::percentile;
  std::fprintf(csv, "%.0f,%d,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%d\n", rate_hz, n, percentile(compute_us, 50),
               percentile(compute_us, 99), percentile(compute_us, 99.9), percentile(compute_us, 100),
               percentile(wake_us, 50), percentile(wake_us, 99), percentile(wake_us, 99.9), percentile(wake_us, 100),
               overruns);
  std::fprintf(md, "| %.0f Hz | %d | %.1f | %.1f | %.1f | %.1f | %.1f | %.1f | %d |\n", rate_hz, n,
               percentile(compute_us, 50), percentile(compute_us, 99), percentile(compute_us, 100),
               percentile(wake_us, 50), percentile(wake_us, 99), percentile(wake_us, 100), overruns);
  std::printf("%5.0f Hz: compute p50 %.1f us p99 %.1f us max %.1f us | wake p50 %.1f us p99 %.1f us max %.1f us | overruns %d\n",
              rate_hz, percentile(compute_us, 50), percentile(compute_us, 99), percentile(compute_us, 100),
              percentile(wake_us, 50), percentile(wake_us, 99), percentile(wake_us, 100), overruns);
}

template <typename F>
double nsPerCall(int iters, F&& f) {
  const auto t0 = Clock::now();
  for (int i = 0; i < iters; ++i) f(i);
  return std::chrono::duration<double, std::nano>(Clock::now() - t0).count() / iters;
}

struct StepMetrics {
  double overshoot_pct, settle_s, iae;
};

StepMetrics windupStep(AntiWindup aw) {
  // Wheel velocity loop driving the simulator's motor model to 18 rad/s (90 % of the no-load
  // speed) from rest: the duty saturates during spin-up, which is where the integrator winds up.
  PidConfig cfg;
  cfg.kp = 0.15;
  cfg.ki = 2.0;
  cfg.anti_windup = aw;
  Pid pid(cfg);
  const double k = 20.0, tau = 0.08, dt = 0.001, sp = 18.0;
  double w = 0.0, peak = 0.0, iae = 0.0, last_outside = 0.0;
  for (int i = 0; i < 5000; ++i) {
    const double u = pid.update(sp, w, dt);
    w += dt * (k * u - w) / tau;
    peak = std::max(peak, w);
    iae += std::abs(sp - w) * dt;
    if (std::abs(sp - w) > 0.02 * sp) last_outside = (i + 1) * dt;
  }
  return {100.0 * (peak - sp) / sp, last_outside, iae};
}

}  // namespace

int main(int argc, char** argv) {
  const std::string out = bench::outDir(argc, argv);
  const bool quick = bench::hasFlag(argc, argv, "--quick");
  bench::File csv(out + "/control_loop_timing.csv"), md(out + "/control_loop_timing.md");
  if (!csv.f || !md.f) return 1;
  std::fprintf(csv.f, "rate_hz,cycles,compute_p50_us,compute_p99_us,compute_p999_us,compute_max_us,"
                      "wake_p50_us,wake_p99_us,wake_p999_us,wake_max_us,overruns\n");
  std::fprintf(md.f, "### Periodic loop (gate + wheel PIDs + simulator + slip-aware EKF)\n\n"
                     "| Rate | Cycles | Compute p50 [us] | Compute p99 [us] | Compute max [us] | Wake-up p50 [us] | Wake-up p99 [us] | Wake-up max [us] | Overruns |\n"
                     "|---|---|---|---|---|---|---|---|---|\n");
  const double secs = quick ? 2.0 : 10.0;
  periodicLoop(100.0, secs, csv.f, md.f);
  periodicLoop(500.0, secs, csv.f, md.f);
  periodicLoop(1000.0, secs, csv.f, md.f);

  // Component costs (tight loop, no sleeping).
  const int iters = quick ? 20000 : 200000;
  Pid pid(PidConfig{0.15, 2.0, 0.01, -1, 1, AntiWindup::BackCalculation, 0.0, 30.0, true});
  volatile double sink = 0.0;
  const double pid_ns = nsPerCall(iters, [&](int i) { sink = sink + pid.update(10.0, 0.001 * (i % 1000), 0.001); });
  LocalizationEkf ekf;
  ekf.reset({});
  const double ekf_ns = nsPerCall(iters, [&](int i) {
    ekf.predict(0.001, 0.01);
    ekf.updateGyro(0.01);
    ekf.updateWheelOdometry(1.0, 0.0, false);
    if (i % 500 == 0) ekf.updateGps(0.001 * i, 0.0);
  });
  DiffDriveSim sim{SimConfig{}};
  const double sim_ns = nsPerCall(iters, [&](int) { sink = sink + sim.step(0.5, 0.5, 0.001).enc_left; });
  const std::string msg = teleop::encodeOperatorMessage({teleop::MsgType::Drive, 77, 0.5, -0.25});
  const double json_ns = nsPerCall(iters, [&](int) { sink = sink + teleop::decodeOperatorMessage(msg)->v; });
  std::fprintf(md.f, "\n### Component cost (tight loop, %d iterations)\n\n| Component | ns/call |\n|---|---|\n", iters);
  std::fprintf(md.f, "| Wheel PID update | %.0f |\n| EKF predict + gyro + odometry (+GPS every 500) | %.0f |\n"
                     "| Simulator step | %.0f |\n| Operator JSON decode | %.0f |\n", pid_ns, ekf_ns, sim_ns, json_ns);
  std::printf("pid %.0f ns, ekf %.0f ns, sim %.0f ns, json %.0f ns\n", pid_ns, ekf_ns, sim_ns, json_ns);

  // Anti-windup comparison.
  bench::File aw(out + "/pid_antiwindup.csv");
  std::fprintf(aw.f, "anti_windup,overshoot_pct,settling_time_2pct_s,iae\n");
  std::fprintf(md.f, "\n### Anti-windup on a saturating wheel-speed step (0 -> 18 rad/s)\n\n"
                     "| Strategy | Overshoot [%%] | 2%% settling [s] | IAE [rad] |\n|---|---|---|---|\n");
  const std::pair<const char*, AntiWindup> modes[] = {
      {"none", AntiWindup::None}, {"clamping", AntiWindup::Clamping}, {"back_calculation", AntiWindup::BackCalculation}};
  for (const auto& [name, mode] : modes) {
    const StepMetrics m = windupStep(mode);
    std::fprintf(aw.f, "%s,%.3f,%.4f,%.4f\n", name, m.overshoot_pct, m.settle_s, m.iae);
    std::fprintf(md.f, "| %s | %.2f | %.3f | %.3f |\n", name, m.overshoot_pct, m.settle_s, m.iae);
    std::printf("%-17s overshoot %.2f %% settle %.3f s IAE %.3f\n", name, m.overshoot_pct, m.settle_s, m.iae);
  }
  return 0;
}
