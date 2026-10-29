// Twist command mux (teleop has priority over autonomy) -> qkrt::DriveController -> wheel duty.
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "qkrt/drive_controller.hpp"

class DriveControllerNode : public rclcpp::Node {
 public:
  DriveControllerNode() : Node("drive_controller_node") {
    qkrt::DriveControllerConfig cfg;
    const double rate = declare_parameter("rate_hz", 100.0);
    cfg.wheel_radius = declare_parameter("wheel_radius", cfg.wheel_radius);
    cfg.track_width = declare_parameter("track_width", cfg.track_width);
    cfg.icr_factor = declare_parameter("icr_factor", cfg.icr_factor);
    cfg.max_wheel_speed = declare_parameter("max_wheel_speed", cfg.max_wheel_speed);
    cfg.wheel_pid.kp = declare_parameter("kp", cfg.wheel_pid.kp);
    cfg.wheel_pid.ki = declare_parameter("ki", cfg.wheel_pid.ki);
    timeout_ = declare_parameter("cmd_timeout", 0.25);
    ctrl_ = std::make_unique<qkrt::DriveController>(cfg);
    dt_ = 1.0 / rate;

    auto twist_cb = [this](Source& src) {
      return [this, &src](geometry_msgs::msg::Twist::SharedPtr m) {
        src.cmd = {m->linear.x, m->angular.z};
        src.stamp = now();
      };
    };
    teleop_sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel_teleop", 10, twist_cb(teleop_));
    auto_sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel_auto", 10, twist_cb(auto_));
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, [this](sensor_msgs::msg::JointState::SharedPtr m) {
          if (m->velocity.size() >= 2) measured_ = {m->velocity[0], m->velocity[1]};
        });
    duty_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("wheel_duty", 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(dt_), [this] { tick(); });
  }

 private:
  struct Source {
    qkrt::Twist2 cmd;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  bool fresh(const Source& s) { return (now() - s.stamp).seconds() < timeout_; }

  void tick() {
    qkrt::Twist2 cmd{};  // stop if no fresh source
    if (fresh(teleop_)) cmd = teleop_.cmd;
    else if (fresh(auto_)) cmd = auto_.cmd;
    const qkrt::DriveOutput out = ctrl_->update(cmd, measured_, dt_);
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {out.duty_left, out.duty_right};
    duty_pub_->publish(msg);
  }

  std::unique_ptr<qkrt::DriveController> ctrl_;
  double dt_ = 0.01, timeout_ = 0.25;
  Source teleop_, auto_;
  qkrt::WheelSpeeds measured_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_, auto_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr duty_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveControllerNode>());
  rclcpp::shutdown();
  return 0;
}
