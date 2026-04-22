# Launch Modes

These launch files are installed with the `racer_bringup` package, so after building and sourcing the workspace you can run them with:

```bash
ros2 launch racer_bringup <launch_file>
```

The main entrypoint is `system_launch.py`. It starts the shared hardware stack from `master_bringup.launch.py` and then enables a controller mode with the `controller` launch argument.

By default, the shared stack includes lidar. The exception is `controller:=line`, which skips the lidar and lidar-based depth nodes because line following only needs the camera stack.

## Recommended launch

Start the full system with no controller:

```bash
ros2 launch racer_bringup system_launch.py
```

Start line following:

```bash
ros2 launch racer_bringup system_launch.py controller:=line
```

This mode skips lidar bringup.

Start wall following:

```bash
ros2 launch racer_bringup system_launch.py controller:=wall
```

Useful overrides:

```bash
ros2 launch racer_bringup system_launch.py controller:=line \
  lidar_port:=/dev/ttyUSB0 \
  rover_port:=/dev/ttyACM1
```

## Other launch files

- `line_vision_launch.py`: camera + line detector only. Useful for vision debugging.
- `line_follow_launch.py`: older line-follow stack without the shared bringup wrapper.
- `wall_stop_launch.py`: older lidar/depth/wall-nav stack without the shared bringup wrapper.
- `green_follow_launch.py`: green-paper detection and following mode.

If you only want sensors, telemetry, rosbridge, and manual `/cmd_vel` control, use `master_bringup.launch.py` directly:

```bash
ros2 launch racer_bringup master_bringup.launch.py
```
