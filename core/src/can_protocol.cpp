#include "qkrt/can_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

namespace qkrt::can {

namespace {
constexpr std::array<uint8_t, 16> kDlcLen{0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

template <typename T>
T scaleSat(double value, double scale) {
  const double raw = std::round(value / scale);
  const double lo = static_cast<double>(std::numeric_limits<T>::min());
  const double hi = static_cast<double>(std::numeric_limits<T>::max());
  if (!std::isfinite(raw)) return 0;
  return static_cast<T>(std::clamp(raw, lo, hi));
}

template <typename T>
void put(uint8_t* p, T v) {  // little-endian
  using U = std::make_unsigned_t<T>;
  U u = static_cast<U>(v);
  for (std::size_t i = 0; i < sizeof(T); ++i) p[i] = static_cast<uint8_t>(u >> (8 * i));
}

template <typename T>
T get(const uint8_t* p) {
  using U = std::make_unsigned_t<T>;
  U u = 0;
  for (std::size_t i = 0; i < sizeof(T); ++i) u |= static_cast<U>(static_cast<U>(p[i]) << (8 * i));
  return static_cast<T>(u);
}

CanFrame frame(uint32_t id, uint8_t len, bool fd = false) {
  CanFrame f;
  f.id = id;
  f.len = len;
  f.fd = fd;
  f.brs = fd;
  return f;
}
}  // namespace

std::size_t dlcToLength(uint8_t dlc) { return kDlcLen[dlc & 0x0F]; }

uint8_t lengthToDlc(std::size_t len) {
  for (uint8_t d = 0; d < 16; ++d)
    if (kDlcLen[d] >= len) return d;
  return 15;
}

std::size_t paddedFdLength(std::size_t len) { return dlcToLength(lengthToDlc(len)); }

uint16_t crc16(const uint8_t* data, std::size_t len) {
  uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

// Drive command layout (8 bytes):
// [0] mode | [1..4] setpoint int32 | [5..6] torque_ff int16 | [7] counter
CanFrame encode(const DriveCommand& m) {
  CanFrame f = frame(id::kDriveCommand + (m.node & id::kNodeMask), 8);
  f.data[0] = static_cast<uint8_t>(m.mode);
  const double sp_scale = m.mode == ControlMode::Duty ? 1e-6 : 1e-3;
  put<int32_t>(&f.data[1], scaleSat<int32_t>(m.setpoint, sp_scale));
  put<int16_t>(&f.data[5], scaleSat<int16_t>(m.torque_ff, 1e-3));
  f.data[7] = m.counter;
  return f;
}

std::optional<DriveCommand> decodeDriveCommand(const CanFrame& f) {
  if ((f.id & ~id::kNodeMask) != id::kDriveCommand || f.len != 8 || f.fd) return std::nullopt;
  if (f.data[0] > static_cast<uint8_t>(ControlMode::Torque)) return std::nullopt;
  DriveCommand m;
  m.node = f.id & id::kNodeMask;
  m.mode = static_cast<ControlMode>(f.data[0]);
  const double sp_scale = m.mode == ControlMode::Duty ? 1e-6 : 1e-3;
  m.setpoint = get<int32_t>(&f.data[1]) * sp_scale;
  m.torque_ff = get<int16_t>(&f.data[5]) * 1e-3;
  m.counter = f.data[7];
  return m;
}

// Motor status layout (8 bytes):
// [0..3] velocity int32 | [4..5] current int16 | [6] faults | [7] counter
CanFrame encode(const MotorStatus& m) {
  CanFrame f = frame(id::kMotorStatus + (m.node & id::kNodeMask), 8);
  put<int32_t>(&f.data[0], scaleSat<int32_t>(m.velocity, 1e-3));
  put<int16_t>(&f.data[4], scaleSat<int16_t>(m.current, 1e-2));
  f.data[6] = m.faults;
  f.data[7] = m.counter;
  return f;
}

std::optional<MotorStatus> decodeMotorStatus(const CanFrame& f) {
  if ((f.id & ~id::kNodeMask) != id::kMotorStatus || f.len != 8 || f.fd) return std::nullopt;
  MotorStatus m;
  m.node = f.id & id::kNodeMask;
  m.velocity = get<int32_t>(&f.data[0]) * 1e-3;
  m.current = get<int16_t>(&f.data[4]) * 1e-2;
  m.faults = f.data[6];
  m.counter = f.data[7];
  return m;
}

// FD telemetry layout (32 bytes):
// [0..3] timestamp_us u32 | [4..7] position i32 | [8..11] velocity i32 | [12..13] iq i16 |
// [14..15] id i16 | [16..17] vbus u16 | [18..19] motor temp i16 | [20..21] fet temp i16 |
// [22..23] faults u16 | [24] counter | [25..29] reserved | [30..31] CRC16 over bytes 0..29
constexpr std::size_t kFdLen = 32;

CanFrame encode(const MotorTelemetryFd& m) {
  CanFrame f = frame(id::kTelemetryFd + (m.node & id::kNodeMask), kFdLen, true);
  put<uint32_t>(&f.data[0], m.timestamp_us);
  put<int32_t>(&f.data[4], m.position_counts);
  put<int32_t>(&f.data[8], scaleSat<int32_t>(m.velocity, 1e-3));
  put<int16_t>(&f.data[12], scaleSat<int16_t>(m.iq, 1e-2));
  put<int16_t>(&f.data[14], scaleSat<int16_t>(m.id, 1e-2));
  put<uint16_t>(&f.data[16], scaleSat<uint16_t>(m.bus_voltage, 1e-2));
  put<int16_t>(&f.data[18], scaleSat<int16_t>(m.motor_temp, 0.1));
  put<int16_t>(&f.data[20], scaleSat<int16_t>(m.fet_temp, 0.1));
  put<uint16_t>(&f.data[22], m.faults);
  f.data[24] = m.counter;
  put<uint16_t>(&f.data[30], crc16(f.data.data(), 30));
  return f;
}

std::optional<MotorTelemetryFd> decodeTelemetryFd(const CanFrame& f) {
  if ((f.id & ~id::kNodeMask) != id::kTelemetryFd || !f.fd || f.len != kFdLen) return std::nullopt;
  if (crc16(f.data.data(), 30) != get<uint16_t>(&f.data[30])) return std::nullopt;
  MotorTelemetryFd m;
  m.node = f.id & id::kNodeMask;
  m.timestamp_us = get<uint32_t>(&f.data[0]);
  m.position_counts = get<int32_t>(&f.data[4]);
  m.velocity = get<int32_t>(&f.data[8]) * 1e-3;
  m.iq = get<int16_t>(&f.data[12]) * 1e-2;
  m.id = get<int16_t>(&f.data[14]) * 1e-2;
  m.bus_voltage = get<uint16_t>(&f.data[16]) * 1e-2;
  m.motor_temp = get<int16_t>(&f.data[18]) * 0.1;
  m.fet_temp = get<int16_t>(&f.data[20]) * 0.1;
  m.faults = get<uint16_t>(&f.data[22]);
  m.counter = f.data[24];
  return m;
}

CanFrame encodeEstop(bool engaged) {
  CanFrame f = frame(id::kEstop, 2);
  f.data[0] = engaged ? 0xA5 : 0x00;  // magic value avoids a stuck-bit false trigger
  f.data[1] = static_cast<uint8_t>(~f.data[0]);
  return f;
}

CanFrame encodeHeartbeat(uint8_t node, uint8_t state) {
  CanFrame f = frame(id::kHeartbeat + (node & id::kNodeMask), 1);
  f.data[0] = state;
  return f;
}

int AliveCounterMonitor::update(uint8_t counter) {
  if (!init_) {
    init_ = true;
    last_ = counter;
    return 0;
  }
  const uint8_t delta = static_cast<uint8_t>(counter - last_);
  last_ = counter;
  if (delta == 0) {
    ++repeats_;
    return 0;
  }
  const int missed = delta - 1;
  missed_ += missed;
  return missed;
}

}  // namespace qkrt::can
