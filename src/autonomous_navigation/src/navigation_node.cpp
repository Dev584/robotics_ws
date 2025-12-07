#include "autonomous_navigation/navigation_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace autonomous_navigation {

NavigationNode::NavigationNode() : Node("autonomous_navigation_node") {
  // Parameters
  declare_parameter("max_velocity", 1.0);
  declare_parameter("max_acceleration", 0.5);
  declare_parameter("lookahead_distance", 1.2);
  declare_parameter("control_frequency", 10.0);
  
  max_velocity_ = get_parameter("max_velocity").as_double();
  max_acceleration_ = get_parameter("max_acceleration").as_double();
  lookahead_distance_ = get_parameter("lookahead_distance").as_double();
  control_frequency_ = get_parameter("control_frequency").as_double();

  // Subscribers
  waypoints_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/waypoints", 10,
      std::bind(&NavigationNode::waypoints_callback, this, std::placeholders::_1));
      
  odom_sub_ = create_subscription<sensor_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&NavigationNode::odom_callback, this, std::placeholders::_1));

  // Publishers
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  path_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/smoothed_path", 10);

  // Control timer
  control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_frequency_),
      std::bind(&NavigationNode::timer_callback, this));

  RCLCPP_INFO(get_logger(), "Autonomous Navigation Node Started!");
}

NavigationNode::~NavigationNode() {
  RCLCPP_INFO(get_logger(), "Autonomous Navigation Node Stopped!");
}

void NavigationNode::waypoints_callback(const nav_msgs::msg::Path::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  std::vector<path_smoothing::Point2D> waypoints;
  for (const auto& pose : msg->poses) {
    waypoints.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  
  if (waypoints.size() >= 2) {
    try {
      path_smoother_ = std::make_unique<path_smoothing::PathSmoother>(waypoints);
      smoothed_path_ = path_smoother_->sampleCurve(100);
      new_waypoints_ = true;
      
      // Generate trajectory
      std::vector<trajectory_generation::Point3D> traj_points;
      for (const auto& point : smoothed_path_) {
        traj_points.emplace_back(point.x, point.y, 0.0);
      }
      trajectory_generator_ = std::make_unique<trajectory_generation::TrajectoryGenerator>(
          traj_points, max_velocity_, max_acceleration_);
      trajectory_ = trajectory_generator_->generate(10.0, 100);
      trajectory_ready_ = true;
      
      RCLCPP_INFO(get_logger(), "New trajectory generated: %zu points", trajectory_.positions.size());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Trajectory generation failed: %s", e.what());
    }
  }
}

void NavigationNode::odom_callback(const sensor_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  current_pose_.x = msg->pose.pose.position.x;
  current_pose_.y = msg->pose.pose.position.y;
  tf2::Quaternion q(msg->pose.pose.orientation.x,
                   msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z,
                   msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  current_pose_.yaw = yaw;
}

void NavigationNode::timer_callback() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  if (!trajectory_ready_) {
    return;
  }
  
  // Initialize controller if needed
  if (!controller_) {
    controller_ = std::make_unique<trajectory_controller::PurePursuitController>(
        lookahead_distance_, max_velocity_, 1.0, 0.1);
  }
  
  // Convert trajectory to 2D poses for controller
  std::vector<trajectory_controller::Pose2D> controller_trajectory;
  for (size_t i = 0; i < trajectory_.positions.size(); ++i) {
    controller_trajectory.emplace_back(
        trajectory_.positions[i].x,
        trajectory_.positions[i].y,
        0.0  // Simplified yaw
    );
  }
  
  // Compute control command
  auto cmd = controller_->compute(current_pose_, controller_trajectory, 0.1);
  
  // Publish velocity command
  geometry_msgs::msg::Twist twist;
  twist.linear.x = cmd.linear_velocity;
  twist.angular.z = cmd.angular_velocity;
  cmd_vel_pub_->publish(twist);
  
  // Publish visualization
  visualization_msgs::msg::MarkerArray markers;
  for (const auto& point : smoothed_path_) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "smoothed_path";
    marker.id = markers.markers.size();
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = 0.1;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
    marker.color.a = 1.0; marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
    markers.markers.push_back(marker);
  }
  path_marker_pub_->publish(markers);
}

}  // namespace autonomous_navigation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autonomous_navigation::NavigationNode)