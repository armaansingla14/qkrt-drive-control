// Slip-aware EKF localization: wheel encoders + IMU + GPS-like local fixes -> odometry.
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include "qkrt/ekf.hpp"

class LocalizationNode : public rclcpp::Node {
 public:
  LocalizationNode() : Node("localization_node") {
    const double r = declare_parameter("wheel_radius", 0.1);
    const double b = declare_parameter("track_width", 0.5);
    const double chi = declare_parameter("icr_factor", 1.0);
    const bool slip_aware = declare_parameter("slip_aware", true);
    qkrt::EkfConfig ekf;
    ekf.gps_std = declare_parameter("gps_std", ekf.gps_std);
    loc_ = std::make_unique<qkrt::Localizer>(
        slip_aware ? qkrt::LocalizerMode::EkfSlipAware : qkrt::LocalizerMode::EkfNaive,
        qkrt::DiffDriveKinematics(r, b, chi), ekf);
    loc_->reset({});

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, [this](sensor_msgs::msg::JointState::SharedPtr m) {
          if (m->velocity.size() >= 2) {
            frame_.enc_left = m->velocity[0];
            frame_.enc_right = m->velocity[1];
          }
        });
    gps_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "gps/local_fix", 10, [this](geometry_msgs::msg::PointStamped::SharedPtr m) {
          frame_.gps_valid = true;
          frame_.gps_x = m->point.x;
          frame_.gps_y = m->point.y;
        });
    // The IMU drives the filter clock.
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu/data", 50, [this](sensor_msgs::msg::Imu::SharedPtr m) {
      const rclcpp::Time t(m->header.stamp);
      if (last_.nanoseconds() > 0) {
        const double dt = (t - last_).seconds();
        if (dt > 0.0 && dt < 0.2) {
          frame_.gyro_z = m->angular_velocity.z;
          frame_.accel_x = m->linear_acceleration.x;
          loc_->process(frame_, dt);
          frame_.gps_valid = false;
          publish(t);
        }
      }
      last_ = t;
    });
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry/filtered", 10);
    slip_pub_ = create_publisher<std_msgs::msg::Bool>("slip_detected", 10);
  }

 private:
  void publish(const rclcpp::Time& t) {
    const qkrt::Pose2 p = loc_->pose();
    const auto& P = loc_->ekf().covariance();
    nav_msgs::msg::Odometry od;
    od.header.stamp = t;
    od.header.frame_id = "map";
    od.child_frame_id = "base_link";
    od.pose.pose.position.x = p.x;
    od.pose.pose.position.y = p.y;
    od.pose.pose.orientation.z = std::sin(0.5 * p.theta);
    od.pose.pose.orientation.w = std::cos(0.5 * p.theta);
    od.pose.covariance[0] = P(0, 0);
    od.pose.covariance[1] = P(0, 1);
    od.pose.covariance[6] = P(1, 0);
    od.pose.covariance[7] = P(1, 1);
    od.pose.covariance[35] = P(2, 2);
    od.twist.twist.linear.x = loc_->ekf().velocity();
    od.twist.twist.angular.z = loc_->ekf().yawRate();
    odom_pub_->publish(od);
    std_msgs::msg::Bool slip;
    slip.data = loc_->slipFlag();
    slip_pub_->publish(slip);
  }

  std::unique_ptr<qkrt::Localizer> loc_;
  qkrt::SensorFrame frame_;
  rclcpp::Time last_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr slip_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
