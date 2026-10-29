// Bridges the JSON/UDP operator protocol (tools/operator_console.py) into ROS 2.
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "qkrt/teleop.hpp"

class TeleopBridgeNode : public rclcpp::Node {
 public:
  TeleopBridgeNode() : Node("teleop_bridge_node") {
    const auto port = static_cast<uint16_t>(declare_parameter("port", 47800));
    qkrt::teleop::TeleopGate::Config cfg;
    cfg.timeout_s = declare_parameter("timeout", cfg.timeout_s);
    cfg.v_max = declare_parameter("v_max", cfg.v_max);
    cfg.w_max = declare_parameter("w_max", cfg.w_max);
    gate_ = qkrt::teleop::TeleopGate(cfg);
    if (!sock_.bind(port)) throw std::runtime_error("teleop_bridge_node: cannot bind UDP port");

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odometry/filtered", 10,
                                                             [this](nav_msgs::msg::Odometry::SharedPtr m) { odom_ = *m; });
    slip_sub_ = create_subscription<std_msgs::msg::Bool>("slip_detected", 10,
                                                         [this](std_msgs::msg::Bool::SharedPtr m) { slip_ = m->data; });
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_teleop", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { tick(); });
    t0_ = now();
  }

 private:
  void tick() {
    const double t = (now() - t0_).seconds();
    while (auto d = sock_.receive()) {
      if (auto m = qkrt::teleop::decodeOperatorMessage(d->payload)) {
        gate_.handle(*m, t);
        host_ = d->host;
        port_ = d->port;
      }
    }
    const auto state = gate_.state(t);
    const qkrt::Twist2 c = gate_.command(t, 0.02);
    // Only publish while an operator is connected so autonomy can take over otherwise.
    if (state == qkrt::teleop::TeleopGate::State::Active || state == qkrt::teleop::TeleopGate::State::Estopped) {
      geometry_msgs::msg::Twist msg;
      msg.linear.x = c.v;
      msg.angular.z = c.w;
      cmd_pub_->publish(msg);
    }
    if (port_ != 0 && ++ticks_ % 5 == 0) {
      qkrt::teleop::Telemetry tel;
      tel.seq = seq_++;
      tel.t = t;
      const auto& q = odom_.pose.pose.orientation;
      tel.pose = {odom_.pose.pose.position.x, odom_.pose.pose.position.y, 2.0 * std::atan2(q.z, q.w)};
      tel.v = odom_.twist.twist.linear.x;
      tel.w = odom_.twist.twist.angular.z;
      tel.slip = slip_;
      tel.estop = state == qkrt::teleop::TeleopGate::State::Estopped;
      tel.state = qkrt::teleop::TeleopGate::stateName(state);
      sock_.sendTo(qkrt::teleop::encodeTelemetry(tel), host_, port_);
    }
  }

  qkrt::teleop::UdpSocket sock_;
  qkrt::teleop::TeleopGate gate_;
  std::string host_;
  uint16_t port_ = 0;
  uint32_t seq_ = 0;
  uint64_t ticks_ = 0;
  bool slip_ = false;
  nav_msgs::msg::Odometry odom_;
  rclcpp::Time t0_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr slip_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
