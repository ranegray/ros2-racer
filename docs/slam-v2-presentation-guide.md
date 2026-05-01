# ROS2 Racer SLAM v2 Presentation Prep Guide

This guide is built from your current `nw/slam-v2` branch so you can explain what the code actually does (not just generic SLAM theory).

## 1) 30-Second Project Pitch

We built a two-lap autonomy pipeline in ROS 2:
- **Lap 1 (mapping):** wall-nav drives while `slam_toolbox` builds a map and `path_recorder` stores a closed-loop trajectory.
- **Transition (coordinator):** `slam_coordinator` saves both map and path, then waits for operator confirmation.
- **Lap 2 (racing):** `path_planner` inflates the map and runs A* through via-points sampled from the recorded lap; `pure_pursuit` follows the published planned path.

The key story: we turned a stable hallway wall-follower into a map-aware lap runner with explicit mode handoffs and safety-aware fallbacks.

## 2) The Real Node/Dataflow (What To Say On Architecture Slide)

Primary launch files:
- `launch/slam_mapping_launch.py` (Lap 1 + transition stack)
- `launch/slam_racing_launch.py` (Lap 2 localization + pursuit)

Core runtime graph:
1. `rplidar_node` publishes `/scan`.
2. `scan_filter_node` republishes:
   - `/scan_filtered` for SLAM (no fake wall fill),
   - `/scan_nav` for conservative navigation.
3. `rf2o_laser_odometry_node` publishes `odom -> base_link` TF (laser odometry prior).
4. `slam_toolbox`:
   - mapping mode in lap 1 (`async_slam_toolbox_node`),
   - localization mode in lap 2 (`localization_slam_toolbox_node`),
   - publishes `map -> odom` TF.
5. `wall_nav_node` controls `/cmd_vel` in mapping mode.
6. `path_recorder_node` records `map -> base_link` poses to `~/.ros/recorded_path.yaml`.
7. `slam_coordinator_node` controls mode transitions (`mapping -> saving -> ready -> racing`).
8. `path_planner_node` builds `/planned_path` on inflated occupancy grid.
9. `pure_pursuit_node` consumes `/planned_path` and drives lap 2.
10. `rover_node` bridges `/cmd_vel` to MAVLink steering/throttle.

## 3) SLAM v2 Pipeline, Step-by-Step

### Lap 1: Mapping
- Launch `slam_mapping_launch.py`.
- Rover drives with `wall_nav_node`.
- `slam_toolbox` builds occupancy map from `/scan_filtered`.
- `path_recorder_node` stores sparse trajectory points (distance-gated) and publishes `/recorded_path` for visualization.

### Transition: Save + Arm Race Mode
- Operator sends:
  - `ros2 topic pub --once /slam_coordinator/switch_mode std_msgs/String "data: racing"`
- Coordinator enters `saving`:
  - tells recorder to flush path (`/slam_coordinator/save_path`),
  - calls `/slam_toolbox/save_map` to `/home/pi/map/track`.
- Once both are confirmed, mode moves to `ready`.
- Operator confirms:
  - `ros2 topic pub --once /slam_coordinator/confirm std_msgs/Empty "{}"`
- Mode moves to `racing`.

### Lap 2: Plan + Track
- `path_planner_node` receives map and mode `ready`/`racing`, then:
  - inflates occupancy grid by `inflation_radius`,
  - loads recorded path file,
  - samples 1/4, 1/2, 3/4 via-points,
  - chains 4 A* segments: `start -> q1 -> q2 -> q3 -> start`,
  - publishes latched `/planned_path`.
- `pure_pursuit_node` activates on mode `racing` and follows `/planned_path`.
- If planned path is missing, it falls back to recorded YAML path.

## 4) Why These Design Choices Make Sense (Great for Q&A)

### Why A* via-points instead of one A* call?
Because start and goal are the same in a closed loop. A single A* might return a trivial path. Via-points force full-lap traversal while still requiring collision-free map-respecting segments.

### Why laser odometry (`rf2o`) + slam_toolbox?
`rf2o` gives better short-term odom prior than dead reckoning in this platform, so slam scan matching has cleaner initial alignment and better map consistency.

### Why two scan topics?
- `/scan_filtered` for mapping preserves unknown/open semantics.
- `/scan_nav` adds gap-fill/inflation so control is conservative near windows/glass/gaps.

### Why explicit coordinator states?
Without `mapping -> saving -> ready -> racing`, map and path save can race with control handoff. Coordinator serializes this and gives operator confirmation control before high-speed mode.

## 5) Wall-Nav Details You Can Explain Credibly

`wall_nav_node` is not "just PD":
- Uses right-wall geometric estimator from two LiDAR rays (`ray_a_deg`, `ray_b_deg`) with look-ahead projection.
- Adds alpha feedback, derivative damping, and IMU yaw-rate damping.
- Implements spike rejection for unrealistic scan jumps.
- Handles wall-loss in phases:
  - coast,
  - committed right turn,
  - release,
  - open-space idle timeout.
- Deactivates on coordinator modes `saving`, `ready`, `racing`.

If asked what was hard: say the hard part was not the PD equation, it was robust corner vs doorway/glass disambiguation and stable recovery timing under real sensor artifacts.

## 6) Pure Pursuit Details You Should Know

`pure_pursuit_node` behavior:
- Waits until mode `racing`.
- Seeds nearest index from current pose (does not blindly start at waypoint 0).
- Finds closest waypoint in a rolling window, then computes lookahead point by arc length.
- Applies Ackermann geometry steering and converts to `/cmd_vel.angular.z`.
- **Important hardware note:** steering sign is inverted on this rover, so output angular rate is negated intentionally in code.
- Adapts speed down for large heading error (`alpha`) so it slows in corners.

## 7) 6-Minute Talk Track (Slide-Friendly)

Use this as speaker notes:

1. **Problem framing (30s):** baseline autonomy first, SLAM as earned stretch.
2. **Stack overview (45s):** sensors -> filtering -> odom/SLAM TF -> control -> rover bridge -> dashboard.
3. **Controller comparison (60s):** LiDAR wall-nav steadier than vision-only under hallway variability.
4. **Telemetry value (45s):** dashboard exposed internal state; enabled causal tuning.
5. **SLAM v2 pipeline (90s):** map lap, save artifacts, mode gate, plan on inflated map, pursuit lap.
6. **Engineering tradeoffs (60s):** repeatability and integration risk dominate over isolated algorithm performance.
7. **Conclusion (30s):** integrated pipeline works in code, next milestone is repeatable hardware demonstration.

## 8) Demo Checklist (If You Present Live)

Before demo:
- Confirm static TF `base_link -> laser` matches physical mount.
- Verify map path in `config/slam_toolbox_localization.yaml` is correct (`/home/pi/map/track`).
- Ensure `~/.ros/recorded_path.yaml` exists and is recent.
- Open dashboard topics: `/map`, `/planned_path`, `/recorded_path`, mode state.

During demo:
1. Run mapping launch.
2. Show map growth and recorded path.
3. Trigger switch to racing (mode command).
4. Wait for READY status.
5. Confirm command.
6. Show `/planned_path` and pursuit lap.

Fallback line if failure occurs:
"The architecture is working as designed if mode/state transitions and artifacts are produced; current gap is runtime robustness under this hardware condition."

## 9) Likely Questions + Good Answers

**Q: Is this really SLAM or just replay?**  
A: Lap 1 is full online SLAM mapping. Lap 2 localizes against frozen map and tracks an A*-planned route constrained by occupancy, with replay path only as fallback.

**Q: Why not pure end-to-end planner from map?**  
A: Closed-loop start=goal made naive planning degenerate; via-point chaining preserves full-lap intent while still requiring collision-free map-consistent planning.

**Q: Biggest technical risk left?**  
A: Repeatability under hardware noise (timing, sensor artifacts, and controller tuning sensitivity), not absence of algorithmic components.

**Q: What would you improve next?**  
A: Add quantitative lap metrics, introduce command supervisor/E-stop layer, and package launch/dependencies into a cleaner deployment profile.

## 10) 1-Minute Exam-Cram Version

If you get called on unexpectedly:

"We implemented a two-lap ROS2 SLAM pipeline. Lap 1 uses wall-nav plus slam_toolbox mapping while recording the trajectory. A coordinator saves both map and path with explicit states before allowing race mode. Lap 2 localizes on the frozen map, inflates occupancy, plans a closed-loop A* path through via-points, and runs pure pursuit to execute it. The hard part was robust integration and repeatability on hardware, especially perception artifacts and controller handoffs."

