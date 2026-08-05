# Quadruped Autonomy Workspace

ROS 2 Jazzy workspace for an RK3588-based quadruped robot with autonomous
obstacle-crossing capability.

## Packages

- `quadruped_description`: parameterized 12-DOF Xacro model.
- `quadruped_control`: ros2_control controller and joint-limit configuration.
- `quadruped_perception`: initial point-cloud terrain feature extractor.
- `quadruped_planning`: obstacle-crossing mode and speed decision manager.
- `quadruped_bringup`: unified launch and project-level parameters.
- `slam`: SLAM Toolbox and Nav2 configuration with a navigation-code skeleton.

## Quick start

```bash
./scripts/bootstrap.sh
./scripts/build.sh
source install/setup.bash
ros2 launch quadruped_bringup bringup.launch.py
```

To inspect only the robot model:

```bash
ros2 launch quadruped_description display.launch.py
```

To start online mapping and navigation (after the LiDAR publishes `/scan` and
the robot publishes the `odom -> base_link` transform):

```bash
ros2 launch slam slam.launch.py
```

## SLAM + Nav2 + obstacle-crossing skeleton

The `slam.launch.py` launch file now starts the complete software chain:

```text
/scan + odom -> SLAM Toolbox -> map/odom -> Nav2 -> /cmd_vel_nav
point cloud -> terrain_analyzer -> crossing mode/speed -> cmd_vel_gate -> /cmd_vel
```

Start it without RViz when testing on a headless computer:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch slam slam.launch.py rviz:=false
```

The terrain node publishes `/terrain/features` as
`[ground_z, high_z, obstacle_height, valid_points, slope, roughness,
frontal_obstacle_height, lookahead, traversability]`. The crossing manager publishes
`/crossing/mode` (`WALK`, `STEP`, `CLIMB`, or `STOP`) and
`/crossing/action` (`NAVIGATE`, `CROSS_STEP`, `CROSS_CLIMB`, or a safe-stop
action) plus `/crossing/speed_scale`. Missing or stale terrain data forces
`STOP`. `cmd_vel_gate` scales or zeros Nav2 commands as a safe placeholder. A
real robot adapter must subscribe to `/cmd_vel` and map the crossing action to
the vendor's gait/footstep API; this skeleton does not yet perform online
footstep planning or MPC.

The launch assumes these external interfaces are supplied by a sensor and
robot driver:

- `/scan` (`sensor_msgs/LaserScan`)
- `/camera/depth/color/points` (`sensor_msgs/PointCloud2`)
- `/odom` and the `odom -> base_link` TF
- a hardware driver consuming `/cmd_vel`

### Robocon obstacle-course mode

The supplied V1.0 rule PDF describes an 8-obstacle course with a 210-second
limit: straight poles, gravel/wood pit, height bar, slope, bridge A, bridge B,
T stairs and high wall. Start the competition state machine with:

```bash
ros2 launch slam slam.launch.py competition:=true
```

Competition mode also starts `course_waypoint_navigator`. It sends Nav2
`NavigateToPose` goals to the configured approach pose for each obstacle and
automatically publishes `/competition/obstacle_hint` after Nav2 reports the
approach successful. Replace the placeholder coordinates in
`course_waypoints.yaml` with measured map coordinates from the actual venue
before testing; the rules allow the obstacle order and positions to be
published before the match.

The competition node is intentionally event-driven so a camera detector and
foot controller can be replaced without changing scoring logic. It accepts:

```text
/competition/obstacle_hint          std_msgs/String
/competition/obstacle_complete      std_msgs/Bool
/competition/obstacle_failed        std_msgs/Bool
/competition/retry                  std_msgs/Bool
/competition/returned_to_start      std_msgs/Bool
/competition/stair_levels_touched   std_msgs/Int32
/competition/stair_sides_completed   std_msgs/Int32 (0/1/2)
/foot_contacts                      std_msgs/UInt8MultiArray (four 0/1 values)
```

It publishes the current obstacle, action, state, score and remaining time.
The implementation enforces the rule constraints that can be observed by
software: 210 s timeout, no duplicate scoring, 100-point return bonus, one or
fewer ground-contact feet on the gravel pit/T stairs/slope/bridges, at least
1 m slope travel, four configurable T-stair levels, and 75/150 scoring for
one/both T-stair directions. A physical detector
must publish completion only after verifying the S-path/must-pass zones, bar
not knocked down, wall crossing, bridge/platform transition and stair-top
contacts required by the referee rules.

## Hardware integration checklist

1. Replace dimensions, masses and inertia values in
   `quadruped_description/urdf/quadruped.urdf.xacro`.
2. Replace mock ros2_control hardware with the motor-driver hardware plugin.
3. Calibrate joint zero positions and update `joint_limits.yaml`.
4. Set the depth-camera or LiDAR point-cloud topic in `terrain.yaml`.
5. Add IMU, foot-force and joint-state fusion.
6. Validate one leg at low current before enabling all controllers.
