# ROS2 Racer Commands

## Build And Source

```bash
colcon build
source install/setup.bash
```

After changing Python nodes or launch files, rebuild/source before running them:

```bash
colcon build --packages-select racer_bringup autonomy perception telemetry
source install/setup.bash
```

## Bring Up Sensors And Dashboard Topics

Run this first. It starts lidar, camera, depth perception, line detector, SLAM,
rover I/O, telemetry, and rosbridge. It does not start `wall_line_nav_node`, so
the robot should not move from that controller.

```bash
ros2 launch racer_bringup bringup_launch.py
```

Override ports if needed:

```bash
ros2 launch racer_bringup bringup_launch.py lidar_port:=/dev/ttyUSB0 rover_port:=/dev/ttyACM1
```

Dashboard topics available from bringup:

```text
/telemetry/racer
/telemetry/camera
/telemetry/line_debug
/scan
/map
```

## Start Wall-Line Navigation

Run this only when you want the wall-line controller to publish `/cmd_vel`.

```bash
ros2 launch racer_bringup wl_nav_launch.py
```

## Test Cmd Vel

Slow forward, once:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.0}}"
```

Slow forward continuously at 5 Hz:

```bash
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.0}}"
```

Steered motion continuously at 5 Hz:

```bash
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.10}, angular: {z: 0.5}}"
```

Stop:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Quick Checks

Check controller commands:

```bash
ros2 topic echo /cmd_vel
```

Check only the command values during a turn:

```bash
ros2 topic echo /cmd_vel --field linear.x
ros2 topic echo /cmd_vel --field angular.z
```

Check odometry:

```bash
ros2 topic echo /odom
ros2 run tf2_ros tf2_echo odom base_link
```

Check sensors:

```bash
ros2 topic echo /scan --once
ros2 topic echo /perception/front_distance
ros2 topic echo /line_follow_point
ros2 topic echo /line_lookahead_point
```

Check dashboard streams:

```bash
ros2 topic echo /telemetry/racer
ros2 topic hz /telemetry/camera
ros2 topic hz /telemetry/line_debug
```

## Dashboard Dev Server

From the dashboard folder:

```bash
cd telemetry-dashboard
npm install
npm run dev -- --host 0.0.0.0
```

Then open the Vite URL in a browser. If needed, point it at a specific robot:

```text
http://localhost:5173/?ros=ws://ROBOT_IP:9090
```
