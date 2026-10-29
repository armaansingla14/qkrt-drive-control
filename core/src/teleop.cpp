#include "qkrt/teleop.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace qkrt::teleop {

// ============================================================================= JSON

namespace {

class Parser {
 public:
  explicit Parser(std::string_view s) : s_(s) {}

  std::optional<FlatJson> parse(std::string* error) {
    FlatJson out;
    ws();
    if (!eat('{')) return fail(error, "expected '{'");
    ws();
    if (eat('}')) return finish(out, error);
    while (true) {
      ws();
      std::string key;
      if (!string(key)) return fail(error, "expected string key");
      ws();
      if (!eat(':')) return fail(error, "expected ':'");
      ws();
      JsonScalar value;
      if (!scalar(value)) return fail(error, "unsupported or malformed value for key '" + key + "'");
      out[key] = std::move(value);
      ws();
      if (eat(',')) continue;
      if (eat('}')) return finish(out, error);
      return fail(error, "expected ',' or '}'");
    }
  }

 private:
  std::optional<FlatJson> finish(FlatJson& out, std::string* error) {
    ws();
    if (i_ != s_.size()) return fail(error, "trailing characters");
    return out;
  }
  std::optional<FlatJson> fail(std::string* error, const std::string& msg) {
    if (error) *error = msg + " at offset " + std::to_string(i_);
    return std::nullopt;
  }
  void ws() {
    while (i_ < s_.size() && (s_[i_] == ' ' || s_[i_] == '\t' || s_[i_] == '\n' || s_[i_] == '\r')) ++i_;
  }
  bool eat(char c) {
    if (i_ < s_.size() && s_[i_] == c) {
      ++i_;
      return true;
    }
    return false;
  }
  bool literal(std::string_view lit) {
    if (s_.substr(i_, lit.size()) == lit) {
      i_ += lit.size();
      return true;
    }
    return false;
  }
  bool string(std::string& out) {
    if (!eat('"')) return false;
    while (i_ < s_.size()) {
      const char c = s_[i_++];
      if (c == '"') return true;
      if (static_cast<unsigned char>(c) < 0x20) return false;
      if (c != '\\') {
        out.push_back(c);
        continue;
      }
      if (i_ >= s_.size()) return false;
      const char e = s_[i_++];
      switch (e) {
        case '"': out.push_back('"'); break;
        case '\\': out.push_back('\\'); break;
        case '/': out.push_back('/'); break;
        case 'b': out.push_back('\b'); break;
        case 'f': out.push_back('\f'); break;
        case 'n': out.push_back('\n'); break;
        case 'r': out.push_back('\r'); break;
        case 't': out.push_back('\t'); break;
        case 'u': {
          if (i_ + 4 > s_.size()) return false;
          unsigned cp = 0;
          for (int k = 0; k < 4; ++k) {
            const char h = s_[i_++];
            cp <<= 4;
            if (h >= '0' && h <= '9') cp |= h - '0';
            else if (h >= 'a' && h <= 'f') cp |= h - 'a' + 10;
            else if (h >= 'A' && h <= 'F') cp |= h - 'A' + 10;
            else return false;
          }
          if (cp < 0x80) {
            out.push_back(static_cast<char>(cp));
          } else if (cp < 0x800) {
            out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
          } else {
            out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
            out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
          }
          break;
        }
        default: return false;
      }
    }
    return false;
  }
  bool number(double& out) {
    const std::size_t start = i_;
    if (i_ < s_.size() && s_[i_] == '-') ++i_;
    if (i_ >= s_.size() || !std::isdigit(static_cast<unsigned char>(s_[i_]))) return false;
    if (s_[i_] == '0') ++i_;
    else while (i_ < s_.size() && std::isdigit(static_cast<unsigned char>(s_[i_]))) ++i_;
    if (i_ < s_.size() && s_[i_] == '.') {
      ++i_;
      if (i_ >= s_.size() || !std::isdigit(static_cast<unsigned char>(s_[i_]))) return false;
      while (i_ < s_.size() && std::isdigit(static_cast<unsigned char>(s_[i_]))) ++i_;
    }
    if (i_ < s_.size() && (s_[i_] == 'e' || s_[i_] == 'E')) {
      ++i_;
      if (i_ < s_.size() && (s_[i_] == '+' || s_[i_] == '-')) ++i_;
      if (i_ >= s_.size() || !std::isdigit(static_cast<unsigned char>(s_[i_]))) return false;
      while (i_ < s_.size() && std::isdigit(static_cast<unsigned char>(s_[i_]))) ++i_;
    }
    const std::string tok(s_.substr(start, i_ - start));
    out = std::strtod(tok.c_str(), nullptr);
    return std::isfinite(out);
  }
  bool scalar(JsonScalar& v) {
    if (i_ >= s_.size()) return false;
    const char c = s_[i_];
    if (c == '"') {
      std::string str;
      if (!string(str)) return false;
      v = std::move(str);
      return true;
    }
    if (literal("true")) { v = true; return true; }
    if (literal("false")) { v = false; return true; }
    double d;
    if (!number(d)) return false;
    v = d;
    return true;
  }

  std::string_view s_;
  std::size_t i_ = 0;
};

void appendEscaped(std::string& out, const std::string& s) {
  out.push_back('"');
  for (const char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out.push_back(c);
        }
    }
  }
  out.push_back('"');
}

template <typename T>
const T* get(const FlatJson& o, const char* key) {
  const auto it = o.find(key);
  return it == o.end() ? nullptr : std::get_if<T>(&it->second);
}

}  // namespace

std::optional<FlatJson> parseFlatJson(std::string_view text, std::string* error) {
  return Parser(text).parse(error);
}

std::string toJson(const FlatJson& obj) {
  std::string out = "{";
  bool first = true;
  for (const auto& [k, v] : obj) {
    if (!first) out.push_back(',');
    first = false;
    appendEscaped(out, k);
    out.push_back(':');
    if (const double* d = std::get_if<double>(&v)) {
      char buf[32];
      if (std::floor(*d) == *d && std::abs(*d) < 1e15) std::snprintf(buf, sizeof(buf), "%.0f", *d);
      else std::snprintf(buf, sizeof(buf), "%.9g", *d);
      out += buf;
    } else if (const bool* b = std::get_if<bool>(&v)) {
      out += *b ? "true" : "false";
    } else {
      appendEscaped(out, std::get<std::string>(v));
    }
  }
  out.push_back('}');
  return out;
}

// ============================================================================= messages

std::optional<OperatorMessage> decodeOperatorMessage(std::string_view text) {
  const auto obj = parseFlatJson(text);
  if (!obj) return std::nullopt;
  const std::string* type = get<std::string>(*obj, "type");
  const double* seq = get<double>(*obj, "seq");
  if (!type || !seq || *seq < 0 || *seq > 4294967295.0 || std::floor(*seq) != *seq) return std::nullopt;
  OperatorMessage m;
  m.seq = static_cast<uint32_t>(*seq);
  if (*type == "drive") {
    const double* v = get<double>(*obj, "v");
    const double* w = get<double>(*obj, "w");
    if (!v || !w) return std::nullopt;
    m.type = MsgType::Drive;
    m.v = *v;
    m.w = *w;
  } else if (*type == "heartbeat") {
    m.type = MsgType::Heartbeat;
  } else if (*type == "estop") {
    m.type = MsgType::Estop;
  } else if (*type == "clear_estop") {
    m.type = MsgType::ClearEstop;
  } else {
    return std::nullopt;
  }
  return m;
}

std::string encodeOperatorMessage(const OperatorMessage& m) {
  FlatJson o;
  o["seq"] = static_cast<double>(m.seq);
  switch (m.type) {
    case MsgType::Drive:
      o["type"] = std::string("drive");
      o["v"] = m.v;
      o["w"] = m.w;
      break;
    case MsgType::Heartbeat: o["type"] = std::string("heartbeat"); break;
    case MsgType::Estop: o["type"] = std::string("estop"); break;
    case MsgType::ClearEstop: o["type"] = std::string("clear_estop"); break;
  }
  return toJson(o);
}

std::string encodeTelemetry(const Telemetry& t) {
  FlatJson o;
  o["type"] = std::string("telemetry");
  o["seq"] = static_cast<double>(t.seq);
  o["t"] = t.t;
  o["x"] = t.pose.x;
  o["y"] = t.pose.y;
  o["theta"] = t.pose.theta;
  o["v"] = t.v;
  o["w"] = t.w;
  o["slip"] = t.slip;
  o["estop"] = t.estop;
  o["state"] = t.state;
  o["loop_us"] = t.loop_us;
  return toJson(o);
}

std::optional<Telemetry> decodeTelemetry(std::string_view text) {
  const auto obj = parseFlatJson(text);
  if (!obj) return std::nullopt;
  const std::string* type = get<std::string>(*obj, "type");
  if (!type || *type != "telemetry") return std::nullopt;
  Telemetry t;
  auto num = [&](const char* k, double& dst) {
    const double* d = get<double>(*obj, k);
    if (d) dst = *d;
    return d != nullptr;
  };
  double seq = 0;
  if (!num("seq", seq) || !num("x", t.pose.x) || !num("y", t.pose.y) || !num("theta", t.pose.theta)) return std::nullopt;
  t.seq = static_cast<uint32_t>(seq);
  num("t", t.t);
  num("v", t.v);
  num("w", t.w);
  num("loop_us", t.loop_us);
  if (const bool* b = get<bool>(*obj, "slip")) t.slip = *b;
  if (const bool* b = get<bool>(*obj, "estop")) t.estop = *b;
  if (const std::string* s = get<std::string>(*obj, "state")) t.state = *s;
  return t;
}

// ============================================================================= gate

bool TeleopGate::handle(const OperatorMessage& m, double now) {
  // Serial-number arithmetic so the 32-bit sequence may wrap.
  if (have_seq_ && static_cast<int32_t>(m.seq - last_seq_) <= 0) {
    ++stale_;
    return false;
  }
  have_seq_ = true;
  last_seq_ = m.seq;
  last_rx_ = now;
  timed_out_reported_ = false;
  ++accepted_;
  switch (m.type) {
    case MsgType::Drive:
      target_ = {std::clamp(m.v, -cfg_.v_max, cfg_.v_max), std::clamp(m.w, -cfg_.w_max, cfg_.w_max)};
      break;
    case MsgType::Heartbeat: break;
    case MsgType::Estop:
      estop_ = true;
      target_ = {};
      break;
    case MsgType::ClearEstop:
      estop_ = false;
      target_ = {};
      break;
  }
  return true;
}

TeleopGate::State TeleopGate::state(double now) const {
  if (estop_) return State::Estopped;
  if (!have_seq_) return State::Idle;
  if (now - last_rx_ > cfg_.timeout_s) return State::TimedOut;
  return State::Active;
}

const char* TeleopGate::stateName(State s) {
  switch (s) {
    case State::Idle: return "idle";
    case State::Active: return "active";
    case State::TimedOut: return "timeout";
    case State::Estopped: return "estop";
  }
  return "unknown";
}

Twist2 TeleopGate::command(double now, double dt) {
  const State s = state(now);
  if (s == State::Estopped) {
    out_ = {};  // hard stop, no ramp
    return out_;
  }
  Twist2 target = target_;
  if (s != State::Active) {
    if (s == State::TimedOut && !timed_out_reported_) {
      ++timeouts_;
      timed_out_reported_ = true;
    }
    target = {};
    target_ = {};
  }
  const double dv = cfg_.accel_max * dt, dw = cfg_.yaw_accel_max * dt;
  out_.v += std::clamp(target.v - out_.v, -dv, dv);
  out_.w += std::clamp(target.w - out_.w, -dw, dw);
  return out_;
}

// ============================================================================= UDP

UdpSocket::UdpSocket() {
  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ >= 0) {
    const int flags = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
  }
}

UdpSocket::~UdpSocket() {
  if (fd_ >= 0) ::close(fd_);
}

bool UdpSocket::bind(uint16_t port, const std::string& address) {
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (::inet_pton(AF_INET, address.c_str(), &addr.sin_addr) != 1) return false;
  const int one = 1;
  ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  return ::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0;
}

bool UdpSocket::sendTo(const std::string& payload, const std::string& host, uint16_t port) {
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) return false;
  const ssize_t n = ::sendto(fd_, payload.data(), payload.size(), 0, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  return n == static_cast<ssize_t>(payload.size());
}

std::optional<Datagram> UdpSocket::receive() {
  char buf[2048];
  sockaddr_in from{};
  socklen_t len = sizeof(from);
  const ssize_t n = ::recvfrom(fd_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&from), &len);
  if (n < 0) return std::nullopt;
  Datagram d;
  d.payload.assign(buf, static_cast<std::size_t>(n));
  char host[INET_ADDRSTRLEN] = {0};
  ::inet_ntop(AF_INET, &from.sin_addr, host, sizeof(host));
  d.host = host;
  d.port = ntohs(from.sin_port);
  return d;
}

std::optional<Datagram> UdpSocket::receive(int timeout_ms) {
  pollfd p{fd_, POLLIN, 0};
  if (::poll(&p, 1, timeout_ms) <= 0) return std::nullopt;
  return receive();
}

uint16_t UdpSocket::localPort() const {
  sockaddr_in addr{};
  socklen_t len = sizeof(addr);
  if (::getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &len) != 0) return 0;
  return ntohs(addr.sin_port);
}

}  // namespace qkrt::teleop
