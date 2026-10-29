// Localization accuracy with and without wheel-slip handling in the diff-drive simulator.
//
// For each scenario and seed the rover is driven closed-loop (wheel PIDs) along a smooth
// randomised twist profile for 120 s. The identical sensor stream is fed to three estimators:
// encoder dead reckoning, a naive EKF (always fuses wheel odometry) and the slip-aware EKF.
#include <cstdio>
#include <map>
#include <random>
#include <string>

#include "bench_util.hpp"
#include "qkrt/drive_controller.hpp"
#include "qkrt/ekf.hpp"
#include "qkrt/sim.hpp"

using namespace qkrt;

namespace {

struct Profile {
  double a1, f1, a2, f2, v0, dv, fv;
  // 3 s at standstill first (gyro-bias settling), then a smooth randomised drive.
  Twist2 at(double t) const {
    if (t < 3.0) return {};
    return {v0 + dv * std::sin(fv * t), a1 * std::sin(f1 * t) + a2 * std::sin(f2 * t)};
  }
};

struct RunMetrics {
  double rmse = 0, max_err = 0, final_err = 0, heading_rmse = 0;
};

enum class Scenario { Nominal, SlipZones, SlipBursts };
const char* name(Scenario s) {
  switch (s) {
    case Scenario::Nominal: return "nominal";
    case Scenario::SlipZones: return "slip_zones";
    case Scenario::SlipBursts: return "slip_bursts";
  }
  return "?";
}
const char* name(LocalizerMode m) {
  switch (m) {
    case LocalizerMode::DeadReckoning: return "dead_reckoning";
    case LocalizerMode::EkfNaive: return "ekf_naive";
    case LocalizerMode::EkfSlipAware: return "ekf_slip_aware";
  }
  return "?";
}

}  // namespace

int main(int argc, char** argv) {
  const std::string out = bench::outDir(argc, argv);
  const bool quick = bench::hasFlag(argc, argv, "--quick");
  const int seeds = quick ? 5 : 30;
  const double duration = 120.0, dt = 0.01;
  const double gps_std = 1.0, gps_rate = 1.0;

  bench::File runs(out + "/localization_runs.csv");
  if (!runs.f) return 1;
  std::fprintf(runs.f, "scenario,seed,estimator,pos_rmse_m,max_err_m,final_err_m,heading_rmse_deg,slip_fraction,"
                       "detector_precision,detector_recall\n");

  const Scenario scenarios[] = {Scenario::Nominal, Scenario::SlipZones, Scenario::SlipBursts};
  const LocalizerMode modes[] = {LocalizerMode::DeadReckoning, LocalizerMode::EkfNaive, LocalizerMode::EkfSlipAware};
  std::map<std::string, std::vector<double>> agg;  // key: scenario/mode/metric

  for (Scenario sc : scenarios) {
    for (int seed = 0; seed < seeds; ++seed) {
      std::mt19937 rng(1000 + seed);
      std::uniform_real_distribution<double> u(0.0, 1.0);
      const Profile prof{0.3 + 0.4 * u(rng), 0.1 + 0.3 * u(rng), 0.2 * u(rng), 0.03 + 0.05 * u(rng),
                         0.8 + 0.6 * u(rng), 0.3 * u(rng), 0.05 + 0.1 * u(rng)};
      SimConfig cfg;
      cfg.seed = 5000 + seed;
      cfg.gps_noise_std = gps_std;
      cfg.gps_rate_hz = gps_rate;
      if (sc == Scenario::SlipZones) {
        // Place low-traction patches on the nominal route.
        Pose2 p;
        for (int k = 0; k < 8; ++k) {
          const double t_hit = duration * (k + 0.5) / 8.0;
          Pose2 q = p;
          for (double t = 0; t < t_hit; t += 0.05) q = integrateArc(q, prof.at(t).v, prof.at(t).w, 0.05);
          cfg.slip_zones.push_back({q.position(), 2.0 + 3.0 * u(rng), 0.1 + 0.4 * u(rng), 0.1 + 0.4 * u(rng)});
        }
      } else if (sc == Scenario::SlipBursts) {
        cfg.slip_event_rate = 0.25;
      }

      DiffDriveSim sim(cfg);
      DriveControllerConfig dcfg;
      DriveController ctrl(dcfg);
      EkfConfig ecfg;
      ecfg.gps_std = gps_std;
      std::vector<Localizer> locs;
      for (LocalizerMode m : modes) locs.emplace_back(m, ctrl.kinematics(), ecfg);
      for (auto& l : locs) l.reset({});

      std::vector<double> se(3, 0.0), maxe(3, 0.0), he(3, 0.0);
      int n = 0, slip_steps = 0, tp = 0, fp = 0;
      SensorFrame s;
      for (double t = 0.0; t < duration; t += dt) {
        const DriveOutput o = ctrl.update(prof.at(t), {s.enc_left, s.enc_right}, dt);
        s = sim.step(o.duty_left, o.duty_right, dt);
        const GroundTruth& g = sim.truth();
        for (std::size_t i = 0; i < locs.size(); ++i) {
          locs[i].process(s, dt);
          const Pose2 e = locs[i].pose();
          const double d2 = (e.x - g.pose.x) * (e.x - g.pose.x) + (e.y - g.pose.y) * (e.y - g.pose.y);
          se[i] += d2;
          maxe[i] = std::max(maxe[i], std::sqrt(d2));
          const double dh = wrapAngle(e.theta - g.pose.theta);
          he[i] += dh * dh;
        }
        ++n;
        slip_steps += g.slipping;
        const bool flag = locs[2].slipFlag();
        tp += flag && g.slipping;
        fp += flag && !g.slipping;
      }
      const double precision = (tp + fp) ? double(tp) / (tp + fp) : 0.0;
      const double recall = slip_steps ? double(tp) / slip_steps : 0.0;
      for (std::size_t i = 0; i < locs.size(); ++i) {
        const Pose2 e = locs[i].pose();
        RunMetrics m{std::sqrt(se[i] / n), maxe[i], distance(e.position(), sim.truth().pose.position()),
                     std::sqrt(he[i] / n) * 180.0 / kPi};
        std::fprintf(runs.f, "%s,%d,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", name(sc), seed, name(modes[i]), m.rmse,
                     m.max_err, m.final_err, m.heading_rmse, double(slip_steps) / n, i == 2 ? precision : 0.0,
                     i == 2 ? recall : 0.0);
        const std::string key = std::string(name(sc)) + "/" + name(modes[i]);
        agg[key + "/rmse"].push_back(m.rmse);
        agg[key + "/max"].push_back(m.max_err);
        agg[key + "/head"].push_back(m.heading_rmse);
        if (i == 2 && slip_steps > 0) {
          agg[key + "/prec"].push_back(precision);
          agg[key + "/rec"].push_back(recall);
        }
        if (i == 2 && slip_steps == 0) agg[key + "/fpr"].push_back(double(fp) / n);
      }
      agg[std::string(name(sc)) + "/slipfrac"].push_back(double(slip_steps) / n);
    }
  }

  bench::File md(out + "/localization.md");
  std::fprintf(md.f, "Simulated, %d seeds x 120 s per scenario, GPS-like fixes at %.0f Hz with %.1f m noise per axis.\n\n",
               seeds, gps_rate, gps_std);
  std::fprintf(md.f, "| Scenario | Time slipping [%%] | Estimator | Position RMSE [m] (mean +/- sd) | Max error [m] | Heading RMSE [deg] |\n"
                     "|---|---|---|---|---|---|\n");
  for (Scenario sc : scenarios) {
    for (LocalizerMode m : modes) {
      const std::string key = std::string(name(sc)) + "/" + name(m);
      std::fprintf(md.f, "| %s | %.1f | %s | %.3f +/- %.3f | %.2f | %.2f |\n", name(sc),
                   100.0 * bench::mean(agg[std::string(name(sc)) + "/slipfrac"]), name(m), bench::mean(agg[key + "/rmse"]),
                   bench::stddev(agg[key + "/rmse"]), bench::mean(agg[key + "/max"]), bench::mean(agg[key + "/head"]));
      std::printf("%-12s %-15s rmse %.3f +/- %.3f max %.2f heading %.2f deg\n", name(sc), name(m),
                  bench::mean(agg[key + "/rmse"]), bench::stddev(agg[key + "/rmse"]), bench::mean(agg[key + "/max"]),
                  bench::mean(agg[key + "/head"]));
    }
  }
  std::fprintf(md.f, "\nSlip detector (per 10 ms step vs simulator ground truth):\n\n| Scenario | Precision | Recall | False-flag rate |\n|---|---|---|---|\n");
  for (Scenario sc : scenarios) {
    const std::string key = std::string(name(sc)) + "/ekf_slip_aware";
    if (sc == Scenario::Nominal)
      std::fprintf(md.f, "| %s | - | - | %.4f |\n", name(sc), bench::mean(agg[key + "/fpr"]));
    else
      std::fprintf(md.f, "| %s | %.3f | %.3f | - |\n", name(sc), bench::mean(agg[key + "/prec"]), bench::mean(agg[key + "/rec"]));
  }
  return 0;
}
