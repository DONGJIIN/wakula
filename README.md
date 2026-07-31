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

## Hardware integration checklist

1. Replace dimensions, masses and inertia values in
   `quadruped_description/urdf/quadruped.urdf.xacro`.
2. Replace mock ros2_control hardware with the motor-driver hardware plugin.
3. Calibrate joint zero positions and update `joint_limits.yaml`.
4. Set the depth-camera or LiDAR point-cloud topic in `terrain.yaml`.
5. Add IMU, foot-force and joint-state fusion.
6. Validate one leg at low current before enabling all controllers.
