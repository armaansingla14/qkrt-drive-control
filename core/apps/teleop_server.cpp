// Simulated rover driven over the JSON/UDP operator protocol.
//
//   teleop_server [--port 47800] [--duration 0 (forever)] [--slip-rate 0.1]
//
// Runs the full onboard stack at 100 Hz against the diff-drive simulator:
// TeleopGate -> DriveController (wheel PIDs) -> DiffDriveSim -> slip-aware Localizer,
// and streams telemetry at 10 Hz to whichever operator endpoint spoke last.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "qkrt/drive_controller.hpp"
#include "qkrt/ekf.hpp"
#include "qkrt/sim.hpp"
#include "qkrt/teleop.hpp"

using namespace qkrt;
using Clock = std::chrono::steady_clock;

int main(int argc, char** argv) {
  uint16_t port = 47800;
  double duration = 0.0;
  double slip_rate = 0.1;
  for (int i = 1; i + 1 < argc; i += 2) {
    if (!std::strcmp(argv[i], "--port")) port = static_cast<uint16_t>(std::atoi(argv[i + 1]));
    else if (!std::strcmp(argv[i], "--duration")) duration = std::atof(argv[i + 1]);
    else if (!std::strcmp(argv[i], "--slip-rate")) slip_rate = std::atof(argv[i + 1]);
  }

  teleop::UdpSocket sock;
  if (!sock.bind(port)) {
    std::fprintf(stderr, "failed to bind UDP port %u\n", port);
    return 1;
  }
  std::printf("teleop_server listening on udp/%u\n", port);
  std::fflush(stdout);

  SimConfig sc;
  sc.slip_event_rate = slip_rate;
  DiffDriveSim sim(sc);
  DriveController ctrl;
  Localizer loc(LocalizerMode::EkfSlipAware, ctrl.kinematics());
  loc.reset({});
  teleop::TeleopGate gate;

  const double dt = 0.01;
  std::string op_host;
  uint16_t op_port = 0;
  uint32_t tel_seq = 0;
  SensorFrame s;
  double worst_us = 0.0;
  auto next = Clock::now();
  const auto t0 = next;
  for (uint64_t tick = 0;; ++tick) {
    next += std::chrono::microseconds(10000);
    std::this_thread::sleep_until(next);
    const auto work_start = Clock::now();
    const double now = std::chrono::duration<double>(work_start - t0).count();
    if (duration > 0.0 && now > duration) break;

    while (auto d = sock.receive()) {
      if (auto m = teleop::decodeOperatorMessage(d->payload)) {
        gate.handle(*m, now);
        op_host = d->host;
        op_port = d->port;
      }
    }
    const Twist2 cmd = gate.command(now, dt);
    const DriveOutput out = ctrl.update(cmd, {s.enc_left, s.enc_right}, dt);
    s = sim.step(out.duty_left, out.duty_right, dt);
    loc.process(s, dt);

    const double loop_us = std::chrono::duration<double, std::micro>(Clock::now() - work_start).count();
    worst_us = std::max(worst_us, loop_us);
    if (tick % 10 == 0 && op_port != 0) {
      teleop::Telemetry t;
      t.seq = tel_seq++;
      t.t = now;
      t.pose = loc.pose();
      t.v = loc.ekf().velocity();
      t.w = loc.ekf().yawRate();
      t.slip = loc.slipFlag();
      t.estop = gate.state(now) == teleop::TeleopGate::State::Estopped;
      t.state = teleop::TeleopGate::stateName(gate.state(now));
      t.loop_us = loop_us;
      sock.sendTo(teleop::encodeTelemetry(t), op_host, op_port);
    }
  }
  std::printf("done: accepted=%llu stale=%llu timeouts=%llu worst_loop_us=%.1f true_pose=(%.2f, %.2f) est=(%.2f, %.2f)\n",
              (unsigned long long)gate.accepted(), (unsigned long long)gate.rejectedStale(),
              (unsigned long long)gate.timeouts(), worst_us, sim.truth().pose.x, sim.truth().pose.y, loc.pose().x,
              loc.pose().y);
  return 0;
}
