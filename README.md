# Advanced Robotics Final Project: ROS2 Racer

This repository contains the code for the ROS2 Racer, a project developed for the Advanced Robotics course at CU Boulder. The ROS2 Racer is a robotic vehicle designed to navigate autonomously using ROS2 (Robot Operating System 2) and various sensors.

## Getting started
To get started with the ROS2 Racer, follow these steps:
1. Clone the repository:
   ```bash
    git clone https://github.com/ranegray/ros2-racer.git
    cd ros2-racer
    ```
2. Install the required dependencies:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```
3. Build the workspace:
    ```bash
    colcon build
    ```
4. Source the workspace:
    ```bash
    source install/setup.bash
    ```
5. Launch the ROS2 Racer:
    ```bash
    ros2 launch racer_bringup master_bringup.launch.py
    ```

## Phase 1 Bringup

The `racer_bringup` package provides a single launch file that starts the entire Phase 1 hardware and infrastructure stack:

```bash
ros2 launch racer_bringup master_bringup.launch.py
```

This starts the following nodes in order:

| Node | Package | Topics |
|------|---------|--------|
| `rplidar_node` | `rplidar_ros` | publishes `/scan` |
| `rs_stream_node` | `rs_stream` | publishes `/camera/color/image_raw` |
| `rover_node` | `robo_rover` | publishes `/imu/gyro`, `/imu/accel`, `/rover/armed`; subscribes `/cmd_vel` |
| *(E-Stop — not yet implemented)* | — | — |
| `telemetry_node` | `telemetry` | publishes `/telemetry/racer` at 10 Hz |
| rosbridge WebSocket | `rosbridge_server` | WebSocket on port 9090 |

Hardware ports can be overridden at launch time:

```bash
ros2 launch racer_bringup master_bringup.launch.py \
    lidar_port:=/dev/ttyUSB0 \
    rover_port:=/dev/ttyACM1
```

Once all nodes are up the car is fully ready to receive commands (`/cmd_vel`) and broadcast telemetry to the dashboard.

## Telemetry Dashboard

React + Vite web dashboard in `telemetry-dashboard/`. Connects to the car via [roslibjs](https://github.com/RobotWebTools/roslibjs) over a rosbridge WebSocket and displays live `racers_msgs/RacerTelemetry` data from `/telemetry/racer`.

```bash
cd telemetry-dashboard
npm install
npm run dev
```

`rosbridge_server` is started automatically by `master_bringup.launch.py`. No separate terminal needed.
