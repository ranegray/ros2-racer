# SLAM v2 Presentation Cheat Sheet (1-Page)

## 15-Second Elevator Pitch
We built a two-lap ROS2 autonomy pipeline: **Lap 1 maps + records**, coordinator **saves and gates mode switch**, **Lap 2 localizes + plans + tracks**. The main challenge was reliable real-world integration, not missing algorithms.

## Pipeline in One Breath
`/scan` -> `scan_filter_node` -> `rf2o odom TF` + `slam_toolbox map TF` -> `wall_nav` (Lap 1) / `pure_pursuit` (Lap 2) -> `/cmd_vel` -> `rover_node`.

## Mode Flow (Must Memorize)
`mapping -> saving -> ready -> racing`

- Trigger transition:
  - `ros2 topic pub --once /slam_coordinator/switch_mode std_msgs/String "data: racing"`
- Confirm race:
  - `ros2 topic pub --once /slam_coordinator/confirm std_msgs/Empty "{}"`

## What Each Core Node Does
- `wall_nav_node`: robust right-wall follow + corner/loss handling for map lap.
- `path_recorder_node`: records `map->base_link` poses to `~/.ros/recorded_path.yaml`.
- `slam_coordinator_node`: ensures map+path save complete before racing mode.
- `path_planner_node`: inflates map, samples via-points, chains 4 A* segments, publishes `/planned_path`.
- `pure_pursuit_node`: follows `/planned_path` at race lap; falls back to recorded YAML if needed.

## 3 High-Value Design Explanations
1. **Why via-point A\*?** Closed loop start=goal can return trivial path; via-points force full lap.
2. **Why rf2o + slam_toolbox?** Better odom prior improves scan matching/map quality.
3. **Why coordinator state machine?** Prevents unsafe/invalid handoff during save and race transition.

## 45-Second “How It Works” Script
Lap 1 launches mapping with wall-nav driving while slam_toolbox builds the occupancy map and path_recorder logs the trajectory in map frame. When we request racing mode, the coordinator enters saving, triggers path flush, and calls map save. After both complete, it enters ready and waits for explicit operator confirm. In racing mode, path_planner inflates the map and runs chained A* through sampled via-points to generate a closed-loop planned path. Pure pursuit then tracks that path in map frame.

## Likely Questions (Quick Answers)
- **“Is this just replay?”** No. Replay is fallback only; primary Lap 2 uses map-constrained A* path.
- **“Biggest remaining risk?”** Hardware repeatability and tuning sensitivity across runs.
- **“What next?”** Add lap metrics, command supervisor/E-stop layer, cleaner deployment packaging.

## Demo Sequence (If Live)
1. Launch mapping stack.
2. Show `/map` + `/recorded_path` growth.
3. Send switch_mode command.
4. Wait for READY.
5. Send confirm command.
6. Show `/planned_path` and pursuit.

## If Something Breaks Mid-Demo
“Architecture is validated if we can show state transitions plus artifact generation (saved map/path + published planned path); current gap is runtime robustness under this hardware condition.”

