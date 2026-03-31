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
    Coming soon!
    <!--```bash
    ros2 launch ros2_racer racer.launch.py
    ```-->

## Telemetry Dashboard

React + Vite web dashboard in `telemetry-dashboard/`. Connects to the car via [roslibjs](https://github.com/RobotWebTools/roslibjs) over a rosbridge WebSocket and displays live `racers_msgs/RacerTelemetry` data from `/telemetry/racer`.

```bash
cd telemetry-dashboard
npm install
npm run dev
```

Requires `rosbridge_server` running on the car:
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
