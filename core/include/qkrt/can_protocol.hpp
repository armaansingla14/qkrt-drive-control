#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace qkrt::can {

/// Raw frame (classic CAN or CAN FD). `len` is the payload byte count.
struct CanFrame {
  uint32_t id = 0;
  bool extended = false;
  bool fd = false;
  bool brs = false;  ///< CAN FD bit-rate switch
  uint8_t len = 0;
  std::array<uint8_t, 64> data{};
};

/// CAN FD DLC <-> length mapping (0..8 linear, then 12,16,20,24,32,48,64).
std::size_t dlcToLength(uint8_t dlc);
/// Smallest DLC whose length is >= len (len <= 64).
uint8_t lengthToDlc(std::size_t len);
std::size_t paddedFdLength(std::size_t len);

/// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) used as an end-to-end check on FD telemetry.
uint16_t crc16(const uint8_t* data, std::size_t len);

// ----------------------------------------------------------------------------- IDs
// 11-bit identifiers: function code in the upper bits, motor-controller node id (0..15) below.
namespace id {
constexpr uint32_t kEstop = 0x000;          ///< broadcast, highest priority
constexpr uint32_t kDriveCommand = 0x100;   ///< + node, classic 8 bytes
constexpr uint32_t kMotorStatus = 0x180;    ///< + node, classic 8 bytes
constexpr uint32_t kTelemetryFd = 0x300;    ///< + node, CAN FD 32 bytes
constexpr uint32_t kHeartbeat = 0x700;      ///< + node, classic 1 byte
constexpr uint32_t kNodeMask = 0x00F;
}  // namespace id

enum class ControlMode : uint8_t { Disabled = 0, Duty = 1, Velocity = 2, Torque = 3 };

enum FaultFlags : uint16_t {
  kFaultNone = 0,
  kFaultOverCurrent = 1u << 0,
  kFaultOverVoltage = 1u << 1,
  kFaultUnderVoltage = 1u << 2,
  kFaultOverTemp = 1u << 3,
  kFaultEncoder = 1u << 4,
  kFaultCommTimeout = 1u << 5,
};

/// Host -> motor controller. Scaling: velocity 0.001 rad/s (int32), torque feed-forward 0.001 N*m (int16).
struct DriveCommand {
  uint8_t node = 0;
  ControlMode mode = ControlMode::Disabled;
  double setpoint = 0.0;      ///< rad/s in Velocity mode, duty [-1,1] in Duty mode, N*m in Torque mode
  double torque_ff = 0.0;
  uint8_t counter = 0;        ///< alive counter, wraps at 256
};

/// Motor controller -> host at 100 Hz. velocity 0.001 rad/s (int32), current 0.01 A (int16).
struct MotorStatus {
  uint8_t node = 0;
  double velocity = 0.0;
  double current = 0.0;
  uint8_t faults = 0;   ///< low byte of FaultFlags
  uint8_t counter = 0;
};

/// Extended telemetry over CAN FD (32-byte payload with CRC).
struct MotorTelemetryFd {
  uint8_t node = 0;
  uint32_t timestamp_us = 0;
  int32_t position_counts = 0;
  double velocity = 0.0;      ///< 0.001 rad/s
  double iq = 0.0;            ///< 0.01 A
  double id = 0.0;            ///< 0.01 A
  double bus_voltage = 0.0;   ///< 0.01 V (uint16)
  double motor_temp = 0.0;    ///< 0.1 C (int16)
  double fet_temp = 0.0;      ///< 0.1 C (int16)
  uint16_t faults = 0;
  uint8_t counter = 0;
};

CanFrame encode(const DriveCommand& m);
CanFrame encode(const MotorStatus& m);
CanFrame encode(const MotorTelemetryFd& m);
CanFrame encodeEstop(bool engaged);
CanFrame encodeHeartbeat(uint8_t node, uint8_t state);

std::optional<DriveCommand> decodeDriveCommand(const CanFrame& f);
std::optional<MotorStatus> decodeMotorStatus(const CanFrame& f);
/// Fails if the id/length is wrong or the CRC does not match.
std::optional<MotorTelemetryFd> decodeTelemetryFd(const CanFrame& f);

/// Tracks an alive counter and reports dropped / repeated frames.
class AliveCounterMonitor {
 public:
  /// Returns the number of frames missed before this one (0 if in sequence).
  int update(uint8_t counter);
  uint64_t missed() const { return missed_; }
  uint64_t repeats() const { return repeats_; }

 private:
  bool init_ = false;
  uint8_t last_ = 0;
  uint64_t missed_ = 0, repeats_ = 0;
};

}  // namespace qkrt::can
