// Plans on /map toward /goal_pose (A*, RRT or RRT*) and tracks the path with pure pursuit.
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "qkrt/planners.hpp"

class PlannerNode : public rclcpp::Node {
 public:
  PlannerNode() : Node("planner_node") {
    planner_ = declare_parameter("planner", std::string("astar"));
    robot_radius_ = declare_parameter("robot_radius", 0.35);
    tracker_ = std::make_unique<qkrt::PurePursuit>(declare_parameter("lookahead", 0.8),
                                                   declare_parameter("cruise_speed", 0.8),
                                                   declare_parameter("goal_tolerance", 0.3));
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", rclcpp::QoS(1).transient_local(), [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) {
          qkrt::OccupancyGrid g(m->info.width, m->info.height, m->info.resolution,
                                {m->info.origin.position.x, m->info.origin.position.y});
          for (int y = 0; y < static_cast<int>(m->info.height); ++y)
            for (int x = 0; x < static_cast<int>(m->info.width); ++x) {
              const int8_t v = m->data[y * m->info.width + x];
              g.set(x, y, (v < 0 || v >= 50) ? 100 : 0);  // unknown treated as occupied
            }
          grid_ = g.inflated(robot_radius_);
          have_map_ = true;
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 10, [this](nav_msgs::msg::Odometry::SharedPtr m) {
          const auto& q = m->pose.pose.orientation;
          pose_ = {m->pose.pose.position.x, m->pose.pose.position.y,
                   std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
          have_pose_ = true;
          follow();
        });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
          plan({m->pose.position.x, m->pose.position.y});
        });
    path_pub_ = create_publisher<nav_msgs::msg::Path>("plan", rclcpp::QoS(1).transient_local());
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_auto", 10);
  }

 private:
  void plan(const qkrt::Vec2& goal) {
    if (!have_map_ || !have_pose_) {
      RCLCPP_WARN(get_logger(), "goal ignored: no map or pose yet");
      return;
    }
    qkrt::PlanResult r;
    if (planner_ == "astar") {
      r = qkrt::planAStar(grid_, pose_.position(), goal);
    } else {
      qkrt::RrtConfig cfg;
      cfg.star = planner_ == "rrt_star";
      r = qkrt::planRrt(grid_, pose_.position(), goal, cfg);
    }
    if (!r.success) {
      RCLCPP_WARN(get_logger(), "%s failed to find a path", planner_.c_str());
      return;
    }
    const auto path = qkrt::shortcutPath(grid_, r.path);
    tracker_->setPath(path);
    nav_msgs::msg::Path msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    for (const auto& p : path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.orientation.w = 1.0;
      msg.poses.push_back(ps);
    }
    path_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "%s: %zu waypoints, %.1f m", planner_.c_str(), path.size(), qkrt::pathLength(path));
  }

  void follow() {
    if (tracker_->done()) return;
    const qkrt::Twist2 c = tracker_->compute(pose_);
    geometry_msgs::msg::Twist t;
    t.linear.x = c.v;
    t.angular.z = c.w;
    cmd_pub_->publish(t);
  }

  std::string planner_;
  double robot_radius_ = 0.35;
  qkrt::OccupancyGrid grid_;
  qkrt::Pose2 pose_;
  bool have_map_ = false, have_pose_ = false;
  std::unique_ptr<qkrt::PurePursuit> tracker_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
