#include "autonomous_navigation/turtlebot3_bridge.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <math.h>
#include <cmath>

namespace autonomous_navigation {

TurtleBot3Bridge::TurtleBot3Bridge() : Node("turtlebot3_bridge") {
  // Parameters
  declare_parameter("max_velocity", 0.3);
  declare_parameter("max_acceleration", 0.5);
  declare_parameter("lookahead_distance", 0.3);
  declare_parameter("control_frequency", 20.0);
  
  max_velocity_ = get_parameter("max_velocity").as_double();
  max_acceleration_ = get_parameter("max_acceleration").as_double();
  lookahead_distance_ = get_parameter("lookahead_distance").as_double();
  control_frequency_ = get_parameter("control_frequency").as_double();

  // Subscribers
  waypoints_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/waypoints", 10,
      std::bind(&TurtleBot3Bridge::waypoints_callback, this, std::placeholders::_1));
      
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&TurtleBot3Bridge::odom_callback, this, std::placeholders::_1));

  // Publishers
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Service
  reset_srv_ = create_service<std_srvs::srv::Empty>(
      "/reset_navigation",
      std::bind(&TurtleBot3Bridge::reset_trajectory, this, std::placeholders::_1, std::placeholders::_2));

  // Control timer
  control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_frequency_),
      std::bind(&TurtleBot3Bridge::control_loop, this));

  RCLCPP_INFO(get_logger(), "TurtleBot3 Bridge initialized!");
  RCLCPP_INFO(get_logger(), "  Max velocity: %.2f m/s", max_velocity_);
  RCLCPP_INFO(get_logger(), "  Lookahead: %.2f m", lookahead_distance_);
}

TurtleBot3Bridge::~TurtleBot3Bridge() = default;

void TurtleBot3Bridge::waypoints_callback(const nav_msgs::msg::Path::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  std::vector<path_smoothing::Point2D> waypoints;
  for (const auto& pose : msg->poses) {
    waypoints.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  
  if (waypoints.size() < 2) {
    RCLCPP_WARN(get_logger(), "Need at least 2 waypoints, got %zu", waypoints.size());
    return;
  }

  try {
    // Path smoothing
    path_smoother_ = std::make_unique<path_smoothing::PathSmoother>(waypoints);
    auto smoothed_path = path_smoother_->sampleCurve(50);
    
    // Trajectory generation
    std::vector<trajectory_generation::Point3D> traj_points;
    for (const auto& point : smoothed_path) {
      traj_points.emplace_back(point.x, point.y, 0.0);
    }
    trajectory_generator_ = std::make_unique<trajectory_generation::TrajectoryGenerator>(
        traj_points, max_velocity_, max_acceleration_);
    trajectory_ = trajectory_generator_->generate(15.0, 150);
    
    // Pure pursuit controller
    controller_ = std::make_unique<trajectory_controller::PurePursuitController>(
        lookahead_distance_, max_velocity_, 2.0, 0.05);
    
    current_trajectory_idx_ = 0;
    trajectory_ready_ = true;
    
    RCLCPP_INFO(get_logger(), "Trajectory generated: %zu points, distance: %.2f m",
                trajectory_.positions.size(),
                path_smoother_->getArcLength());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to generate trajectory: %s", e.what());
    trajectory_ready_ = false;
  }
}

void TurtleBot3Bridge::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  current_pose_.x = msg->pose.pose.position.x;
  current_pose_.y = msg->pose.pose.position.y;
  
  tf2::Quaternion q(msg->pose.pose.orientation.x,
                   msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z,
                   msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, current_pose_.yaw);
}

void TurtleBot3Bridge::control_loop() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  if (!trajectory_ready_ || !controller_) {
    geometry_msgs::msg::Twist zero_twist;
    cmd_vel_pub_->publish(zero_twist);
    return;
  }

  // Convert trajectory to 2D poses
  std::vector<trajectory_controller::Pose2D> control_trajectory;
  for (const auto& pos : trajectory_.positions) {
    control_trajectory.emplace_back(pos.x, pos.y, 0.0);
  }

  try {
    auto cmd = controller_->compute(current_pose_, control_trajectory, 1.0 / control_frequency_);
    
    geometry_msgs::msg::Twist twist;
    twist.linear.x = cmd.linear_velocity;
    twist.angular.z = cmd.angular_velocity;
    cmd_vel_pub_->publish(twist);

    // Check if reached goal
    double dx = trajectory_.positions.back().x - current_pose_.x;
    double dy = trajectory_.positions.back().y - current_pose_.y;
    double dist_to_goal = std::sqrt(dx*dx + dy*dy);
    
    if (dist_to_goal < 0.1) {
      RCLCPP_INFO(get_logger(), "Goal reached!");
      trajectory_ready_ = false;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Control error: %s", e.what());
  }
}

void TurtleBot3Bridge::reset_trajectory() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  trajectory_ready_ = false;
  RCLCPP_INFO(get_logger(), "Trajectory reset");
}

}  // namespace autonomous_navigation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autonomous_navigation::TurtleBot3Bridge)