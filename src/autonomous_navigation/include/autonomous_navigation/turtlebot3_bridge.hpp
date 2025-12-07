#ifndef AUTONOMOUS_NAVIGATION_TURTLEBOT3_BRIDGE_HPP_
#define AUTONOMOUS_NAVIGATION_TURTLEBOT3_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/empty.hpp>

#include <path_smoothing/path_smoother.hpp>
#include <trajectory_generation/trajectory_generator.hpp>
#include <trajectory_controller/pure_pursuit_controller.hpp>

#include <vector>
#include <memory>
#include <mutex>

namespace autonomous_navigation {

class TurtleBot3Bridge : public rclcpp::Node {
 public:
  TurtleBot3Bridge();
  ~TurtleBot3Bridge();

 private:
  void waypoints_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void control_loop();
  void reset_trajectory();

  // Components
  std::unique_ptr<path_smoothing::PathSmoother> path_smoother_;
  std::unique_ptr<trajectory_generation::TrajectoryGenerator> trajectory_generator_;
  std::unique_ptr<trajectory_controller::PurePursuitController> controller_;

  // State
  trajectory_generation::Trajectory trajectory_;
  trajectory_controller::Pose2D current_pose_;
  size_t current_trajectory_idx_ = 0;
  bool new_waypoints_ = false;
  bool trajectory_ready_ = false;
  double current_time_ = 0.0;

  // Parameters
  double max_velocity_ = 0.3;      // TurtleBot3 Burger: 0.22 m/s max
  double max_acceleration_ = 0.5;
  double lookahead_distance_ = 0.3;
  double control_frequency_ = 20.0;

  // Publishers/Subscribers
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoints_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Thread safety
  std::mutex state_mutex_;
};

}  // namespace autonomous_navigation

#endif