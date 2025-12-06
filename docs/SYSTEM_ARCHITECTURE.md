# System Architecture

## Data Flow
```bash
   Waypoints
       │
       ▼
[Path Smoother]
       │
       ▼
[Trajectory Generator] <───┐
       │                   │
       ▼                   │
  [Controller] <───────┐   │
       │               │   │
       ▼               │   │
    [Robot] ───────────┴───┘
            Odometry Feedback
```

## ROS2 Nodes
- waypoint_publisher (publishes waypoints)
- path_smoother_node (smooths path)
- trajectory_generator_node (generates trajectory)
- trajectory_controller_node (tracks trajectory)
- turtlebot3_simulator (Gazebo)

## Topics
| Topic | Type | Purpose |
|-------|------|---------|
| /waypoints | PointStamped[] | Input waypoints |
| /smooth_path | Path | Smooth curve |
| /trajectory | PointStamped[] | Time-stamped trajectory |
| /cmd_vel | Twist | Velocity commands |
| /odom | Odometry | Robot state feedback |
