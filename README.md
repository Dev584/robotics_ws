# Robotics Path Smoothing & Trajectory Control

Complete impelementation of path smoothing and trajectory tracking for TurtleBot3 differential drive robots using ROS2.

### Features
- Path Smoothing: Cubic spline interpolation
- Trajectory Generation: Time-parameterized trajectories
- Trajectory Tracking: Pure Pursuit controller
- Simulation: Gazebo-based testing on TurtleBot3

### Quick Start
```bash
cd ~/robotics_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Project Structure
```bash
robotics_ws/src/
|--path_smoothing/
|--trajectory_generation/
|--trajectory_controller/
|--trajectory_control_sim/
```

---
Updated: December 6, 2025
