# ros2_amiga_ws

ROS2 workspace for Amiga robot — robot-side deployment (no visualization tools).

## Packages

| Package | Description |
|---|---|
| `amiga_description` | URDF/xacro robot model + `robot_state_publisher` launch |
| `amiga_navsat_ekf` | `robot_localization` EKF (local + global) + NavSat transform |
| `amiga_navigation` | Nav2 stack (MPPI controller, BT navigator, costmaps) |
| `amiga_slam` | SLAM Toolbox `online_async` mapping launch |
| `task_manager` | Mission queue ROS2 node — enqueue waypoints, cancel, pause |

## Build

```bash
cd ~/ros2_amiga_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Launch

```bash
# Robot description
ros2 launch amiga_description description.launch.py

# EKF + NavSat
ros2 launch amiga_navsat_ekf navsat_ekf.launch.py

# SLAM mapping
ros2 launch amiga_slam slam.launch.py

# Navigation (Nav2)
ros2 launch amiga_navigation navigation.launch.py

# ROS2 Bridge (WebSocket port 9090)
ros2 launch amiga_navigation ros2_bridge.launch.py

# Task Manager node
ros2 run task_manager task_manager_node
```

## Task Manager API

| Interface | Type | Description |
|---|---|---|
| `/task_manager/add_waypoint` | `geometry_msgs/PoseStamped` sub | Enqueue a navigation waypoint |
| `/task_manager/cancel` | `std_msgs/Empty` sub | Cancel current goal and clear queue |
| `/task_manager/status` | `std_msgs/String` pub | Current state + queue size |
| `/task_manager/current_goal` | `geometry_msgs/PoseStamped` pub | Active navigation target |
| `/task_manager/clear_queue` | `std_srvs/Trigger` srv | Clear all queued tasks |
| `/task_manager/pause` | `std_srvs/SetBool` srv | Pause (`true`) / resume (`false`) |
