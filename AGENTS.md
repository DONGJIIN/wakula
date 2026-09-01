# Wakula repository guide for coding agents

This file is the machine-readable handoff for Codex and other coding agents. Read it
before changing the repository. Human operating instructions remain in `README.md`,
`instruction.txt`, `connect.txt`, and `quickstart.txt`.

## Current scope

Wakula currently owns the hardware-independent perception and navigation layer:

- ROS 2 Jazzy, SLAM Toolbox, Nav2, OpenCV, depth/3D point-cloud terrain analysis;
- unknown-map exploration, obstacle approach/alignment, task inventory, and the
  `TraverseObstacle` Action client contract;
- an independent Gazebo sensor/field test source and an independent Xbox teleop tool.

It does **not** own leg kinematics, gait generation, whole-body control, motor drivers,
`ros2_control`, or the real `TraverseObstacle` Action server. Never report a simulated
Action as proof that a real quadruped can traverse an obstacle.

## Architectural boundaries that must remain true

1. Gazebo, the core SLAM stack, and autonomous navigation remain three independent
   launch processes. Core algorithms must not read Gazebo entity names, world files, or
   fixed competition coordinates. The optional simulation-only TraverseObstacle server
   belongs exclusively to `quadruped_gazebo`: `robocon_field_teleport.launch.py` may
   start it with the field, while `autonomous_navigation.launch.py` must never start,
   detect, import, or depend on that backend.
2. The hardware boundary uses standard topics and frames: `/scan`, `/odom`, `/tf`,
   camera `Image`, terrain `PointCloud2`, `/cmd_vel`, `map`, `odom`, and `base_link`.
3. OpenCV is supporting evidence. Metric point-cloud geometry is authoritative for
   height, depth, slope, clearance, and safe Action handoff.
4. Nav2 moves through free space and reaches an obstacle entry. A real motion controller
   must execute the obstacle. Do not make an obstacle traversable by deleting it from a
   costmap.
5. Keep real-robot deployment/tuning values in the existing YAML files. Python parameter
   declarations may retain tested fallback values required before YAML is loaded, but they
   must stay synchronized and are not a second tuning surface. Never create another inactive
   "tuning" file; numeric values in documentation are versioned snapshots, not overrides.

## Parameter ownership

- `src/slam/config/slam.yaml`: scan matching, map resolution, loop closure, laser range.
- `src/slam/config/nav2.yaml`: footprint/radius, inflation, free-space speed, controller,
  planner, costmaps, sensor-health contracts.
- `src/slam/config/sensor_profiles.yaml`: driver topic remaps only; it starts no driver.
- `src/quadruped_perception/config/vision.yaml`: image quality, HSV/ROI, OpenCV temporal
  confirmation, camera/point-cloud synchronization.
- `src/quadruped_perception/config/terrain.yaml`: point-cloud ROI, ground segmentation,
  metric obstacle thresholds, and RK3588 point limits.
- `src/quadruped_planning/config/terrain_navigation.yaml`: perception-to-speed policy,
  emergency stop, obstacle approach/alignment/READY boundary.
- `src/quadruped_planning/config/autonomous_mission.yaml`: exploration, five-second stall
  recovery, Action handoff, traversal verification, task timeout, and return-to-finish.

Each file starts with a real-robot tuning index. Follow its symptom-to-parameter guidance
and change one parameter group at a time from a labelled rosbag. Topic/frame differences
belong in a sensor profile or launch override, not in algorithm source code.

## Comment and interface conventions

- Explain why a guard exists, its units/frame, the failure symptom, tuning direction,
  and the safety trade-off. Avoid comments that merely restate the next line.
- Python constants and machine interfaces use stable English IDs. Chinese text is for
  operator logs and documentation only.
- Any message/topic/frame/ownership change must update `connect.txt` and tests.
- Any behavior or parameter change must update all four root documents. Preserve their
  current structure; append concise dated evidence to `quickstart.txt` when appropriate.
- Git commit subjects are English. The normal target is `main`; do not rewrite history.

## Required verification

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --all --verbose
```

Also run `git diff --check`. Preserve unrelated user changes. For an online integration
test, start Gazebo, SLAM, and autonomy separately using the three commands documented in
`quickstart.txt`, then stop every process after collecting evidence.
