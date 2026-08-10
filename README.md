# Wakula 四足机器人感知与自主导航底座

当前阶段只开发硬件无关的软件：ROS 2 Jazzy、SLAM Toolbox、Nav2、OpenCV、点云地形分析、相机/点云融合和 rosbag 离线评估。项目**没有实现**真机驱动、运动学、步态、状态估计、全身控制或真实越障执行。

## 根目录文档

| 文件 | 用途 |
|---|---|
| `README.md` | 项目范围、架构、安装、启动与当前状态 |
| `instruction.txt` | 算法作用、原理和调参方法 |
| `connect.txt` | 节点、话题、消息、TF 和速度链通信合同 |
| `quickstart.txt` | 新电脑复现、常用命令、验收和后续接入顺序 |

## 整机研发待完成工作

以下内容等真机结构、执行器和传感器选型确定后再做；仓库当前不包含这些“假实现”。

1. 机械、电气与硬件选型
   - 机身与腿部结构、关节电机/减速器、驱动器、编码器、足端力传感器、电池与电源分配。
   - 实测连杆长度、质量、质心、惯量、关节零位/方向/限位和负载能力。
   - 确定相机、2D/3D 雷达、IMU 的型号、安装刚度、视场、供电、带宽和防护。
2. 硬件接入与状态估计
   - 编写厂家 SDK 适配或 `ros2_control` hardware plugin，发布真实 `/joint_states`、`/imu/data`、电池和故障状态。
   - 融合编码器、IMU、足端接触与视觉/雷达里程计，可靠发布 `/odom` 和 `odom -> base_link`。
   - 完成时间同步、内外参标定、关节零位标定和传感器到 `base_link` 的实测 TF。
3. 运动学、动力学与基础运动
   - 基于实测几何建立 FK/IK、雅可比、动力学、重力补偿和力/扭矩限制。
   - 实现站起、趴下、静态站立、爬行/小跑、步态切换、落足检测与跌倒恢复。
4. 真正越障控制
   - 落脚点规划、摆动腿轨迹、接触力闭环及 MPC/WBC。
   - 实现 STEP/CLIMB/LOW-BAR 的控制接口、取消、超时、失败重试及 Nav2/越障控制切换。
   - 真机验证越障高度、稳定裕度、打滑、碰撞和失败恢复。
5. 系统安全与比赛逻辑
   - 硬件急停、驱动器看门狗、过流/过温/欠压、姿态异常和关节超限保护。
   - 比赛计时、计分、障碍顺序、返回起点及裁判接口。
6. 真机验证
   - 按真实数据重新标定 SLAM、Nav2、OpenCV 和点云参数；完成长期运行、断流、TF 丢失、定位漂移和极端光照测试。

## 当前已经完成

- SLAM Toolbox 在线建图参数和一键入口。
- Nav2 规划、控制、平滑、恢复行为、碰撞监控、启动条件与导航健康诊断。
- OpenCV 轻量障碍证据：CLAHE、HSV、轮廓、形态学、ROI、自适应 Canny、多帧稳定；未使用 YOLO。
- 点云 ROI、地面估计、坡度、粗糙度、台阶高度、坑洞、墙面、横杆和立柱几何分类。
- 带时间戳的相机/点云消息及近似时间配对融合。
- 地形决策与失效安全速度门。STEP/CLIMB 等当前只分类并停车，绝不输出腿部动作。
- 常见相机/雷达 profile、显式话题覆盖、rosbag 记录和离线准确率统计。
- 单元测试、启动文件测试与 GitHub Actions。

## 软件架构

```text
/scan + /odom + TF -> SLAM Toolbox -> /map -> Nav2
                                      Nav2 -> /cmd_vel_nav
                                             -> velocity_smoother
                                             -> /cmd_vel_smoothed

PointCloud2 -> terrain_analyzer -> 地形特征 + /perception/obstacle_points -> Nav2 local costmap
Image       -> vision_obstacle_detector -> OpenCV 辅助证据
两路带时间戳消息 -> perception_fusion -> /perception/fused_obstacle
地形 + 视觉证据 -> obstacle_crossing_manager -> 模式/建议/速度比例
/cmd_vel_smoothed + 速度比例 -> cmd_vel_gate -> /cmd_vel_terrain_safe
collision_monitor -> /cmd_vel（未来由真机底盘接口消费）
```

点云几何是安全决策的主证据；单目 OpenCV 没有可靠米制尺度，只能帮助发现颜色/轮廓并减速复核。感知断流、字段非法或 TF 不可用时系统按未知危险处理。

## ROS 2 包

| 包 | 作用 |
|---|---|
| `slam` | 一键启动、SLAM/Nav2 参数、行为树、传感器 profile、健康检查 |
| `quadruped_perception` | OpenCV、点云几何、时间同步融合 |
| `quadruped_planning` | 保守地形决策和 Nav2 速度门；不含腿部控制 |
| `quadruped_interfaces` | `TerrainFeatures`、`VisionObstacle`、`FusedObstacle` |
| `quadruped_bringup` | 启动感知、决策、速度门与占位 TF 模型 |
| `quadruped_description` | RViz/传感器 TF 占位 URDF；未真机标定 |
| `quadruped_tools` | rosbag 感知评估工具 |

## 安装与构建

推荐 Ubuntu 24.04 + ROS 2 Jazzy。安装 ROS 后：

```bash
cd ~/wakula
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-opencv \
  ros-jazzy-cv-bridge ros-jazzy-slam-toolbox ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup ros-jazzy-pointcloud-to-laserscan \
  ros-jazzy-tf2-sensor-msgs ros-jazzy-robot-state-publisher ros-jazzy-xacro \
  ros-jazzy-rviz2 ros-jazzy-rosbag2
rosdep install --from-paths src --ignore-src -r -y
./scripts/build.sh
source install/setup.bash
```

VS Code 建议插件：Python、Pylance、C/C++、CMake Tools、XML、YAML、ROS（可用时）。`.vscode/tasks.json` 已提供构建、测试、启动、录包和诊断任务。

## 一键启动

先启动真实传感器驱动，确保至少有 `/scan`、`/odom`、相关 TF；若要地形识别，还需 RGB 图像和 `PointCloud2`。

```bash
cd ~/wakula
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch slam slam.launch.py
```

无图形界面：

```bash
ros2 launch slam slam.launch.py rviz:=false
```

指定常见设备：

```bash
ros2 launch slam slam.launch.py sensor_profile:=realsense_d400
```

任意设备直接覆盖话题，无需改源码：

```bash
ros2 launch slam slam.launch.py \
  scan_topic:=/front_lidar/scan odom_topic:=/robot/odom \
  camera_topic:=/rgb/image_raw point_cloud_topic:=/depth/points
```

如果暂时没有相机，可用 `vision:=false`；如果外部已提供 `robot_state_publisher`，可用 `robot_model:=false`。回放 rosbag 时加 `use_sim_time:=true`。

## 传感器安装预留

- 2D 建图雷达：机身顶部中央附近，扫描面高于机身遮挡，水平安装；建议 360°、10 Hz 以上、ROS 2 驱动稳定。
- RGB-D/双目相机：机身前方中央，约离地 0.35–0.55 m，向下俯视约 10–20°，看见前方 0.2–2 m 地面。
- 3D 雷达（可选）：顶部中央、无遮挡；当前 2D SLAM 仍需要 `/scan`，可由 3D 点云切片产生。
- 安装后必须实测 TF 和相机内参，不能直接沿用占位 URDF 数值。

常见起点：RealSense D435i/D455、Orbbec Gemini 2 等 RGB-D；RPLIDAR/LDLIDAR/YDLIDAR 等 2D 雷达；需要更强地形点云时再考虑 Ouster/Livox/Hesai/Robosense。最终选择以室外光照、量程、帧率、ROS 2 驱动和 RK3588 带宽实测为准。

## 录包、评估与测试

```bash
./scripts/record_bag.sh
ros2 run quadruped_tools perception_bag_evaluator --help
./scripts/diagnose.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test
colcon test-result --verbose
```

标签与评估格式见 `instruction.txt`。算法阈值只是无真机阶段的保守初值，必须用真实 rosbag 重新统计精确率、召回率和混淆矩阵后再用于移动平台。

## 当前安全边界

- `/cmd_vel` 只是标准输出接口，当前仓库没有电机消费者。
- `WALK` 才允许非零速度；STEP、CLIMB、坑洞、墙面、横杆均停车等待重规划或未来控制器。
- URDF 只用于显示与传感器 TF 占位，不代表已完成运动学、动力学或碰撞验证。
- 不含 YOLO、Gazebo、硬件 SDK、`ros2_control`、FK/IK、步态、状态估计、越障 Action、比赛状态机。
