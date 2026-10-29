#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <variant>

#include "qkrt/types.hpp"

namespace qkrt::teleop {

// ----------------------------------------------------------------------------- flat JSON
// The operator protocol only uses flat objects with number / bool / string values, so a
// small strict parser is enough and avoids a JSON dependency on the robot computer.
using JsonScalar = std::variant<double, bool, std::string>;
using FlatJson = std::map<std::string, JsonScalar>;

std::optional<FlatJson> parseFlatJson(std::string_view text, std::string* error = nullptr);
std::string toJson(const FlatJson& obj);

// ----------------------------------------------------------------------------- messages

enum class MsgType { Drive, Heartbeat, Estop, ClearEstop };

/// Operator -> robot. Example: {"type":"drive","seq":42,"v":0.5,"w":-0.2}
struct OperatorMessage {
  MsgType type = MsgType::Heartbeat;
  uint32_t seq = 0;
  double v = 0.0;
  double w = 0.0;
};

/// Robot -> operator.
struct Telemetry {
  uint32_t seq = 0;
  double t = 0.0;
  Pose2 pose;
  double v = 0.0;
  double w = 0.0;
  bool slip = false;
  bool estop = false;
  std::string state = "idle";
  double loop_us = 0.0;
};

std::optional<OperatorMessage> decodeOperatorMessage(std::string_view text);
std::string encodeOperatorMessage(const OperatorMessage& m);
std::optional<Telemetry> decodeTelemetry(std::string_view text);
std::string encodeTelemetry(const Telemetry& t);

// ----------------------------------------------------------------------------- safety gate

/// Turns a stream of operator messages into a safe velocity command:
/// stale/out-of-order packets are dropped, a missing stream times out to zero,
/// e-stop latches until an explicit clear, and output is limited in magnitude and rate.
class TeleopGate {
 public:
  struct Config {
    double timeout_s = 0.25;
    double v_max = 2.0;
    double w_max = 2.5;
    double accel_max = 3.0;      ///< [m/s^2]
    double yaw_accel_max = 8.0;  ///< [rad/s^2]
  };
  enum class State { Idle, Active, TimedOut, Estopped };

  TeleopGate() : TeleopGate(Config{}) {}
  explicit TeleopGate(const Config& cfg) : cfg_(cfg) {}

  /// Returns false if the message was rejected as stale.
  bool handle(const OperatorMessage& m, double now);
  /// Rate-limited command for this control tick.
  Twist2 command(double now, double dt);
  State state(double now) const;
  static const char* stateName(State s);

  uint64_t accepted() const { return accepted_; }
  uint64_t rejectedStale() const { return stale_; }
  uint64_t timeouts() const { return timeouts_; }

 private:
  Config cfg_;
  bool have_seq_ = false;
  uint32_t last_seq_ = 0;
  double last_rx_ = -1e9;
  bool estop_ = false;
  bool timed_out_reported_ = true;
  Twist2 target_;
  Twist2 out_;
  uint64_t accepted_ = 0, stale_ = 0, timeouts_ = 0;
};

// ----------------------------------------------------------------------------- UDP transport

struct Datagram {
  std::string payload;
  std::string host;
  uint16_t port = 0;
};

/// Minimal non-blocking IPv4 UDP socket (POSIX).
class UdpSocket {
 public:
  UdpSocket();
  ~UdpSocket();
  UdpSocket(const UdpSocket&) = delete;
  UdpSocket& operator=(const UdpSocket&) = delete;

  bool bind(uint16_t port, const std::string& address = "0.0.0.0");
  bool sendTo(const std::string& payload, const std::string& host, uint16_t port);
  std::optional<Datagram> receive();
  /// Wait up to timeout_ms for a datagram.
  std::optional<Datagram> receive(int timeout_ms);
  uint16_t localPort() const;
  bool valid() const { return fd_ >= 0; }

 private:
  int fd_ = -1;
};

}  // namespace qkrt::teleop
