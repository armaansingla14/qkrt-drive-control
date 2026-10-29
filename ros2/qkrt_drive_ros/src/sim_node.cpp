// Publishes simulated rover sensors from qkrt::DiffDriveSim and consumes wheel duty commands.
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "qkrt/sim.hpp"

class SimNode : public rclcpp::Node {
 public:
  SimNode() : Node("sim_node") {
    const double rate = declare_parameter("rate_hz", 100.0);
    qkrt::SimConfig cfg;
    cfg.slip_event_rate = declare_parameter("slip_event_rate", 0.1);
    cfg.gps_rate_hz = declare_parameter("gps_rate_hz", 1.0);
    cfg.gps_noise_std = declare_parameter("gps_noise_std", 1.0);
    sim_ = std::make_unique<qkrt::DiffDriveSim>(cfg);
    dt_ = 1.0 / rate;

    duty_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "wheel_duty", 10, [this](std_msgs::msg::Float64MultiArray::SharedPtr m) {
          if (m->data.size() >= 2) {
            duty_l_ = m->data[0];
            duty_r_ = m->data[1];
          }
        });
    joints_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    gps_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("gps/local_fix", 10);
    truth_pub_ = create_publisher<nav_msgs::msg::Odometry>("ground_truth/odom", 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(dt_), [this] { tick(); });
  }

 private:
  void tick() {
    const qkrt::SensorFrame s = sim_->step(duty_l_, duty_r_, dt_);
    const auto stamp = now();

    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    js.name = {"wheel_left", "wheel_right"};
    js.velocity = {s.enc_left, s.enc_right};
    joints_pub_->publish(js);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "imu_link";
    imu.angular_velocity.z = s.gyro_z;
    imu.linear_acceleration.x = s.accel_x;
    imu_pub_->publish(imu);

    if (s.gps_valid) {
      geometry_msgs::msg::PointStamped fix;
      fix.header.stamp = stamp;
      fix.header.frame_id = "map";
      fix.point.x = s.gps_x;
      fix.point.y = s.gps_y;
      gps_pub_->publish(fix);
    }

    const auto& g = sim_->truth();
    nav_msgs::msg::Odometry od;
    od.header.stamp = stamp;
    od.header.frame_id = "map";
    od.child_frame_id = "base_link";
    od.pose.pose.position.x = g.pose.x;
    od.pose.pose.position.y = g.pose.y;
    od.pose.pose.orientation.z = std::sin(0.5 * g.pose.theta);
    od.pose.pose.orientation.w = std::cos(0.5 * g.pose.theta);
    od.twist.twist.linear.x = g.v;
    od.twist.twist.angular.z = g.w;
    truth_pub_->publish(od);
  }

  std::unique_ptr<qkrt::DiffDriveSim> sim_;
  double dt_ = 0.01, duty_l_ = 0.0, duty_r_ = 0.0;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr duty_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr gps_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimNode>());
  rclcpp::shutdown();
  return 0;
}
