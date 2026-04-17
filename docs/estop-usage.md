# E-Stop Quick Reference

The safety monitor node provides a keyboard e-stop and automatic staleness detection. It runs in the same terminal as `ros2 launch` - no separate window needed.

## Controls

| Key | Action |
|-----|--------|
| `SPACE` | Trigger e-stop - kills rover_node and stops the rover |
| `ENTER` | Recover - respawns rover_node and resumes operation |

## Automatic Triggers

The e-stop also fires automatically if:

- A watched topic goes silent for longer than `stale_timeout` (default 2s) - e.g. the perception pipeline dies
- `rover_node` exits unexpectedly

## Launch Parameters

These can be overridden on the command line (e.g. `stale_timeout:=3.0`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `connection_string` | `/dev/ttyACM1` | Serial port for the Pixhawk |
| `baud_rate` | `115200` | Serial baud rate |
| `watched_topics` | behavior-dependent | Topics monitored for staleness (see below) |
| `stale_timeout` | `2.0` | Seconds before a silent topic triggers e-stop |

### Watched topics by launch file

| Launch file | Watched topic |
|-------------|--------------|
| `wall_stop_launch.py` | `/perception/front_distance` |
| `green_follow_launch.py` | `/goal_point` |
