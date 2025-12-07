#ifndef AUTONOMOUS_NAVIGATION_NAVIGATION_NODE_HPP_
#define AUTONOMOUS_NAVIGATION_NAVIGATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <path_smoothing/path_smoother.hpp>
#include <trajectory_generation/trajectory_generator.hpp>
#include <trajectory_controller/pure_pursuit_controller.hpp>

#include <vector>
#include <memory>
#include <mutex>

namespace autonomous_navigation {

struct Waypoint {
  double x, y;
  Waypoint(double x_val = 0.0, double y_val = 0.0) : x(x_val), y(y_val) {}
};

class NavigationNode : public rclcpp::Node {
 public:
  NavigationNode();
  ~NavigationNode();

 private:
  void waypoints_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void odom_callback(const sensor_msgs::msg::Odometry::SharedPtr msg);
  void timer_callback();

  // Components
  std::unique_ptr<path_smoothing::PathSmoother> path_smoother_;
  std::unique_ptr<trajectory_generation::TrajectoryGenerator> trajectory_generator_;
  std::unique_ptr<trajectory_controller::PurePursuitController> controller_;

  // State
  std::vector<path_smoothing::Point2D> smoothed_path_;
  trajectory_generation::Trajectory trajectory_;
  trajectory_controller::Pose2D current_pose_;
  bool new_waypoints_ = false;
  bool trajectory_ready_ = false;
  double current_time_ = 0.0;

  // Parameters
  double max_velocity_ = 1.0;
  double max_acceleration_ = 0.5;
  double lookahead_distance_ = 1.2;
  double control_frequency_ = 10.0;

  // Publishers/Subscribers
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoints_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_marker_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Thread safety
  std::mutex data_mutex_;
};

}  // namespace autonomous_navigation

#endif