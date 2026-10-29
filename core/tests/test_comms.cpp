#include <cstring>

#include "qkrt/can_protocol.hpp"
#include "qkrt/teleop.hpp"
#include "test_harness.hpp"

using namespace qkrt;
using namespace qkrt::teleop;

TEST_CASE("json parse flat object") {
  const auto o = parseFlatJson(R"( {"type":"drive","seq":7,"v":-0.5e0,"w":1.25,"ok":true,"s":"a\"bA"} )");
  CHECK(o.has_value());
  CHECK(std::get<std::string>(o->at("type")) == "drive");
  CHECK_NEAR(std::get<double>(o->at("v")), -0.5, 1e-12);
  CHECK(std::get<bool>(o->at("ok")));
  CHECK(std::get<std::string>(o->at("s")) == "a\"bA");
}

TEST_CASE("json rejects malformed input") {
  std::string err;
  CHECK(!parseFlatJson("{\"a\":1,}", &err));
  CHECK(!err.empty());
  CHECK(!parseFlatJson("{\"a\":[1,2]}"));
  CHECK(!parseFlatJson("{\"a\":{\"b\":1}}"));
  CHECK(!parseFlatJson("{\"a\":01}"));
  CHECK(!parseFlatJson("{\"a\":1} x"));
  CHECK(!parseFlatJson("{\"a\":nan}"));
  CHECK(!parseFlatJson("{\"a\":\"unterminated}"));
  CHECK(parseFlatJson("{}").has_value());
}

TEST_CASE("json round trip") {
  FlatJson o{{"x", 1.5}, {"n", 42.0}, {"flag", false}, {"name", std::string("rover\n1")}};
  const auto back = parseFlatJson(toJson(o));
  CHECK(back.has_value() && *back == o);
}

TEST_CASE("operator message encode/decode") {
  OperatorMessage m{MsgType::Drive, 123, 0.75, -0.3};
  const auto d = decodeOperatorMessage(encodeOperatorMessage(m));
  CHECK(d && d->type == MsgType::Drive && d->seq == 123);
  CHECK_NEAR(d->v, 0.75, 1e-9);
  CHECK(!decodeOperatorMessage(R"({"type":"drive","seq":1,"v":0.5})"));  // missing w
  CHECK(!decodeOperatorMessage(R"({"type":"fly","seq":1})"));
  CHECK(!decodeOperatorMessage(R"({"type":"heartbeat","seq":-1})"));
}

TEST_CASE("telemetry encode/decode") {
  Telemetry t;
  t.seq = 9;
  t.pose = {1.0, -2.0, 0.5};
  t.slip = true;
  t.state = "active";
  const auto d = decodeTelemetry(encodeTelemetry(t));
  CHECK(d && d->seq == 9 && d->slip && d->state == "active");
  CHECK_NEAR(d->pose.y, -2.0, 1e-9);
}

TEST_CASE("gate drops stale packets and handles wrap") {
  TeleopGate gate;
  CHECK(gate.handle({MsgType::Drive, 10, 1, 0}, 0.0));
  CHECK(!gate.handle({MsgType::Drive, 9, 1, 0}, 0.01));
  CHECK(!gate.handle({MsgType::Drive, 10, 1, 0}, 0.01));
  CHECK(gate.rejectedStale() == 2);
  TeleopGate g2;
  CHECK(g2.handle({MsgType::Heartbeat, 0xFFFFFFFEu, 0, 0}, 0.0));
  CHECK(g2.handle({MsgType::Heartbeat, 1u, 0, 0}, 0.01));
}

TEST_CASE("gate ramps, clamps and times out") {
  TeleopGate::Config cfg;
  cfg.v_max = 1.0;
  cfg.accel_max = 2.0;
  cfg.timeout_s = 0.2;
  TeleopGate gate(cfg);
  gate.handle({MsgType::Drive, 1, 5.0, 0.0}, 0.0);
  double t = 0.0;
  Twist2 c;
  for (int i = 0; i < 10; ++i) c = gate.command(t += 0.01, 0.01);
  CHECK_NEAR(c.v, 0.2, 1e-9);  // rate-limited
  for (int i = 0; i < 9; ++i) gate.handle({MsgType::Drive, uint32_t(2 + i), 5.0, 0.0}, t), c = gate.command(t += 0.05, 0.05);
  CHECK_NEAR(c.v, 1.0, 1e-9);  // clamped to v_max
  for (int i = 0; i < 100; ++i) c = gate.command(t += 0.01, 0.01);
  CHECK(gate.state(t) == TeleopGate::State::TimedOut);
  CHECK_NEAR(c.v, 0.0, 1e-9);
  CHECK(gate.timeouts() == 1);
}

TEST_CASE("estop latches until cleared") {
  TeleopGate gate;
  gate.handle({MsgType::Drive, 1, 1.0, 0.0}, 0.0);
  gate.command(0.1, 0.1);
  gate.handle({MsgType::Estop, 2, 0, 0}, 0.1);
  CHECK_NEAR(gate.command(0.11, 0.01).v, 0.0, 1e-12);
  gate.handle({MsgType::Drive, 3, 1.0, 0.0}, 0.12);
  CHECK(gate.state(0.12) == TeleopGate::State::Estopped);
  CHECK_NEAR(gate.command(0.13, 0.01).v, 0.0, 1e-12);
  gate.handle({MsgType::ClearEstop, 4, 0, 0}, 0.14);
  gate.handle({MsgType::Drive, 5, 1.0, 0.0}, 0.15);
  CHECK(gate.command(0.16, 0.01).v > 0.0);
}

TEST_CASE("udp loopback") {
  UdpSocket server, client;
  CHECK(server.bind(0, "127.0.0.1"));
  const uint16_t port = server.localPort();
  CHECK(port != 0);
  CHECK(client.sendTo(encodeOperatorMessage({MsgType::Drive, 1, 0.3, 0.1}), "127.0.0.1", port));
  const auto d = server.receive(500);
  CHECK(d.has_value());
  if (d) {
    const auto m = decodeOperatorMessage(d->payload);
    CHECK(m && m->seq == 1);
    CHECK(server.sendTo("{\"type\":\"telemetry\",\"seq\":1,\"x\":0,\"y\":0,\"theta\":0}", d->host, d->port));
    CHECK(client.receive(500).has_value());
  }
}

TEST_CASE("can fd dlc mapping") {
  CHECK(can::dlcToLength(8) == 8);
  CHECK(can::dlcToLength(9) == 12);
  CHECK(can::dlcToLength(15) == 64);
  CHECK(can::lengthToDlc(13) == 10);
  CHECK(can::paddedFdLength(33) == 48);
  CHECK(can::lengthToDlc(64) == 15);
}

TEST_CASE("crc16 ccitt-false check value") {
  const char* s = "123456789";
  CHECK(can::crc16(reinterpret_cast<const uint8_t*>(s), 9) == 0x29B1);
}

TEST_CASE("drive command round trip and saturation") {
  can::DriveCommand c{3, can::ControlMode::Velocity, 15.4321, -0.25, 200};
  const can::CanFrame f = can::encode(c);
  CHECK(f.id == 0x103 && f.len == 8 && !f.fd);
  const auto d = can::decodeDriveCommand(f);
  CHECK(d && d->node == 3 && d->counter == 200);
  CHECK_NEAR(d->setpoint, 15.432, 1e-9);
  CHECK_NEAR(d->torque_ff, -0.25, 1e-9);
  c.torque_ff = 1000.0;  // beyond int16 range at 1 mN*m
  CHECK_NEAR(can::decodeDriveCommand(can::encode(c))->torque_ff, 32.767, 1e-9);
  CHECK(!can::decodeMotorStatus(f));
}

TEST_CASE("fd telemetry round trip and crc rejects corruption") {
  can::MotorTelemetryFd t;
  t.node = 1;
  t.timestamp_us = 123456789;
  t.position_counts = -40960;
  t.velocity = -12.5;
  t.iq = 8.25;
  t.bus_voltage = 24.31;
  t.motor_temp = 41.2;
  t.faults = can::kFaultOverTemp | can::kFaultEncoder;
  t.counter = 17;
  can::CanFrame f = can::encode(t);
  CHECK(f.fd && f.len == 32 && can::paddedFdLength(f.len) == 32);
  const auto d = can::decodeTelemetryFd(f);
  CHECK(d && d->position_counts == -40960 && d->faults == t.faults);
  CHECK_NEAR(d->bus_voltage, 24.31, 1e-9);
  CHECK_NEAR(d->motor_temp, 41.2, 1e-9);
  f.data[9] ^= 0x10;
  CHECK(!can::decodeTelemetryFd(f));
}

TEST_CASE("alive counter monitor") {
  can::AliveCounterMonitor mon;
  mon.update(254);
  CHECK(mon.update(255) == 0);
  CHECK(mon.update(2) == 2);  // 0 and 1 missed across the wrap
  mon.update(2);
  CHECK(mon.missed() == 2 && mon.repeats() == 1);
}

TEST_MAIN()
