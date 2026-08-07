# Wakula 四足机器人自主导航与越障

Wakula 是面向 Ubuntu 24.04、ROS 2 Jazzy 和 RK3588 的四足机器人调试工作空间。
当前版本以 **2D 雷达 SLAM + Nav2 路径规划 + 深度点云地形分析 + OpenCV 障碍提示**
构成轻量融合链路，不包含深度学习推理框架或神经网络模型。

工程目前提供完整的软件雏形和安全接口；真正的抬腿、攀爬、跳跃仍需由机器狗厂商
SDK、MPC/WBC 或落脚点规划器实现，不能把本工程未经标定就用于高速实机。

## 文档导航（先看这里）

| 文件 | 功能 | 适合什么时候看 |
|---|---|---|
| `README.md` | 项目简介、总体架构、安装和启动入口 | 第一次了解或运行项目 |
| `quickstart.txt` | 新电脑完整复现、依赖插件、当前进度和后续路线 | 换电脑部署或接手开发 |
| `instruction.txt` | 各模块作用、算法原理、参数和调试步骤 | 修改 SLAM、Nav2、OpenCV 或越障算法 |
| `connect.txt` | Topic、Action、Service、TF、QoS、字段和超时 | 接入相机、雷达、SDK 或其他节点 |

## 当前状态与待完成工作

已完成：6 个 ROS 2 包可编译，19 项测试通过；已有 SLAM、Nav2、OpenCV、深度点云、
越障决策、速度安全链路，以及常见雷达/相机 profile 和自定义话题兼容入口。

待完成（按建议优先级）：

- **P0 真机接入**：确定机器狗型号和 SDK，实现 `/cmd_vel`、越障动作、里程计、关节及足端接触适配。
- **P0 标定与安全**：实测 URDF、相机/雷达外参、IMU 和里程计，并验证急停、驱动看门狗及失联停车。
- **P0 真实越障控制**：把 `STEP/CLIMB/LOW_PROFILE` 请求接入 IK、MPC/WBC 或厂商步态控制器。
- **P1 感知升级**：完成 RGB 与深度时间同步和空间关联，增加高程图、坡面及可落脚区域分析。
- **P1 比赛调试**：替换正式场地点位，现场标定 OpenCV/点云阈值，逐项验证 Robocon 障碍。
- **P2 工程验证**：补齐 Gazebo/Isaac 自动场景，并进行 RK3588 长时间负载、温度和延迟测试。

更完整的已完成清单、未完成细项和开发顺序见 `quickstart.txt` 第七至九节。

## 1. 目录结构

```text
wakula/
├── src/
│   ├── quadruped_description/  # 12 自由度 URDF/Xacro、RViz、传感器 TF
│   ├── quadruped_control/      # ros2_control 控制器及关节限制
│   ├── quadruped_bringup/      # 机器人、感知、规划和安全节点统一入口
│   ├── quadruped_perception/   # OpenCV 视觉及 PointCloud2 地形分析
│   ├── quadruped_planning/     # 越障决策、速度门、比赛状态机
│   └── slam/                   # SLAM Toolbox、Nav2 参数与自主启动
├── scripts/
│   ├── bootstrap.sh            # rosdep 安装依赖
│   └── build.sh                # 统一编译
├── .colcon/defaults.yaml
├── .vscode/settings.json
└── README.md
```

`build/`、`install/`、`log/` 与 Python 缓存都是可重建产物，不提交到 Git。

## 2. 各模块职责

| 包 | 主要职责 | 关键入口 |
|---|---|---|
| `quadruped_description` | 机器人模型、关节、雷达/相机坐标系 | `display.launch.py` |
| `quadruped_control` | ros2_control 控制器和关节限制 | `controllers.yaml` |
| `quadruped_bringup` | 公共启动入口，避免重复节点 | `bringup.launch.py` |
| `quadruped_perception` | 图像障碍证据、点云地形几何、Nav2 障碍点云 | 两个分析节点 |
| `quadruped_planning` | `WALK/STEP/CLIMB/STOP`、速度安全门、比赛 FSM | 四个规划节点 |
| `slam` | 建图、定位、全局/局部规划、碰撞监控 | `slam.launch.py` |

主要节点：

- `vision_obstacle_detector`：OpenCV HSV + Canny 轮廓识别，并做多帧确认。
- `terrain_analyzer`：将深度点云转换到 `base_link`，分析高度、坡度和粗糙度。
- `obstacle_crossing_manager`：融合视觉证据和点云几何，生成越障模式与速度倍率。
- `cmd_vel_gate`：检查导航命令及决策心跳并缩放速度，任何一项超时立即输出零速。
- `competition_obstacle_manager`：Robocon 障碍进度、计时、接触约束和计分。
- `course_waypoint_navigator`：比赛模式下向 Nav2 发送障碍接近点。

## 3. SLAM、Nav2、OpenCV 与点云如何协同

```text
2D 雷达 /scan ──> SLAM Toolbox ──> /map + map→odom
       │                                  │
       └──────────────────────────────────> Nav2 全局/局部规划

RGB 相机 ──> OpenCV ──> /vision/obstacle_evidence ─┐
                                                  ├─> crossing manager
深度点云 ──> TF(base_link) ──> /terrain/features ─┘       │
       └────────────────> /perception/obstacle_points ─> Nav2 local_costmap

Nav2 /cmd_vel_nav ─> velocity_smoother ─> cmd_vel_gate
     ─> collision_monitor ─> /cmd_vel ─> 厂商 SDK / 自研步态控制器
```

职责边界如下：

1. **SLAM** 用 `/scan` 在陌生环境生成地图和定位，不负责跨越动作。
2. **Nav2** 根据地图规划路线；激光和深度点云共同写入局部代价地图用于绕障。
3. **OpenCV** 识别杆、限高横杆、墙面和大面积有色障碍，用于提前减速和提示。
4. **深度点云** 测量障碍高度、坡度和粗糙度，是 `STEP/CLIMB/STOP` 的几何依据。
5. **Collision Monitor** 是 `/cmd_vel` 的唯一发布者，负责最后一层碰撞保护。

OpenCV 不估计真实距离，也不能独立触发抬腿或跳跃。只有视觉和点云时间上有效、且
点云确认几何条件后，才进入对应越障模式；点云缺失、无效或超时默认 `STOP`。

## 4. 默认与可替换传感器接口

算法内部始终保持 ROS 2 标准合同，不写死厂商品牌：

| 数据 | 内部默认 | 消息类型 | 兼容入口覆盖参数 |
|---|---|---|---|
| 2D 激光 | `/scan` | `sensor_msgs/msg/LaserScan` | `scan_topic` |
| 里程计 | `/odom` | `nav_msgs/msg/Odometry` | `odom_topic` |
| RGB | 自动选择 | `sensor_msgs/msg/Image` | `camera_topic` |
| 深度/3D 点云 | 自动选择 | `sensor_msgs/msg/PointCloud2` | `point_cloud_topic` |

`slam.launch.py` 是纯默认入口；`sensor_compat.launch.py` 是硬件兼容入口。后者通过一层
集中 remap/参数转发适配驱动，SLAM、Nav2、碰撞保护和感知源码都不用修改。预置 profile：

```text
2D 雷达：ros_default、rplidar、ydlidar、ldlidar、hokuyo
RGB-D：  realsense_d400、orbbec_gemini2、zed2、oak_d
3D 雷达：velodyne、ouster、livox、hesai、robosense、lslidar
```

profile 是常见驱动命名的起点，不绑定具体驱动版本；实际名称不同时用四个参数覆盖。
配置集中在 `slam/config/sensor_profiles.yaml`，以后新增型号只需复制一个 YAML 段。

图像和点云使用 Sensor Data QoS；不指定 profile/显式话题时自动监听 ROS 常用默认话题：

```text
RGB:
  /camera/image_raw
  /camera/color/image_raw
  /image_raw

PointCloud2:
  /camera/depth/points
  /camera/depth/color/points
  /camera/points
  /points
```

所有消息必须有有效时间戳和 `header.frame_id`，并存在传感器 frame 到 `base_link` 的 TF。
相机建议同时发布同命名空间 `sensor_msgs/msg/CameraInfo`，为后续图像—深度投影预留。
只有 3D 点云而没有 `LaserScan` 时，需要用 `pointcloud_to_laserscan` 或厂商转换节点生成
二维扫描；压缩图像需先经 `image_transport` 转为标准 `Image`。

## 5. 安装、编译与测试

全新 Ubuntu 24.04 电脑的 ROS 2 软件源、完整 apt/rosdep 依赖、VS Code 安装与六个推荐
插件、Git 克隆、硬件驱动、双机 DDS 网络和逐项验收说明见 `quickstart.txt` 第三节。
仓库的 `.vscode/settings.json` 使用 `${workspaceFolder}` 相对路径，可放在任意用户目录。

```bash
cd ~/wakula
source /opt/ros/jazzy/setup.bash
./scripts/bootstrap.sh
./scripts/build.sh
source install/setup.bash
```

视觉仅依赖 `python3-opencv`、`python3-numpy` 和 `ros-jazzy-cv-bridge`，不会加载模型，
默认 8 Hz、最大 640 像素宽；点云默认 10 Hz 且限制采样数量，适合 RK3588 起步调试。

运行单元测试：

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

## 6. 启动方式

只查看模型：

```bash
ros2 launch quadruped_description display.launch.py
```

完整启动 SLAM + Nav2 + 感知 + 越障安全链路：

```bash
ros2 launch slam slam.launch.py
```

无图形界面：

```bash
ros2 launch slam slam.launch.py rviz:=false
```

没有 RGB 相机时：

```bash
ros2 launch slam slam.launch.py vision:=false
```

覆盖非默认相机话题：

```bash
ros2 launch slam slam.launch.py \
  camera_topic:=/my_camera/image_raw \
  point_cloud_topic:=/my_camera/points
```

用常见硬件 profile 启动（示例为 RealSense D400）：

```bash
ros2 launch slam sensor_compat.launch.py sensor_profile:=realsense_d400
```

任意未知设备无需新增代码，直接覆盖实际话题：

```bash
ros2 launch slam sensor_compat.launch.py \
  scan_topic:=/front_lidar/scan \
  odom_topic:=/robot/odometry \
  camera_topic:=/rgb/image_raw \
  point_cloud_topic:=/depth/points
```

只启动模型、控制、感知和安全门，不启用 SLAM/Nav2：

```bash
ros2 launch quadruped_bringup bringup.launch.py
```

Robocon 比赛模式：

```bash
ros2 launch slam slam.launch.py competition:=true
```

Nav2 节点启动后先保持未激活。就绪监视器收到 `/scan`、`/odom`，并确认
`map -> base_link` TF 可用后才会自动激活整套导航生命周期；因此没有连接传感器时可
安全打开和关闭调试环境，不会在等待 TF 的生命周期切换中崩溃。地形节点仍会等待相机
外参，这是正常安全行为。若只检查参数、不希望自动激活，可使用：

```bash
ros2 launch slam slam.launch.py rviz:=false nav2_autostart:=false
```

## 7. OpenCV 障碍识别

节点同时使用两类轻量特征：

- HSV 橙色/蓝色区域：对比赛场地中颜色明显的杆和横杆优先识别。
- 灰度 Canny 轮廓：颜色不可靠时，利用细长双立柱、宽横条和大矩形补充判断。

每帧先做形态学去噪和轮廓几何筛选，再在最近 5 帧中要求至少 3 帧类型一致。因此单帧
反光或运动模糊不会直接触发减速。输出接口：

```text
/vision/obstacle_evidence  std_msgs/Float32MultiArray
/vision/obstacle_hint      std_msgs/String
/vision/color_features     std_msgs/Float32MultiArray  # 标定/兼容接口
/vision/debug_mask         sensor_msgs/Image           # 默认关闭
```

`/vision/obstacle_evidence` 是越障决策使用的原子结果：

```text
[type_code, confidence, center_x, center_y, width, height]

type_code: 0=none, 1=poles, 2=height_bar, 3=wall, 4=colored_obstacle
其余字段均归一化到 0.0～1.0
```

只有证据置信度达到 `vision_min_confidence`、目标位于行进方向中央且结果未超时，才会
将正常 `WALK` 降速为 `VERIFY_VISUAL_OBSTACLE_WITH_DEPTH`。视觉不会覆盖已经由点云
给出的 `STEP`、`CLIMB` 或 `STOP`。

现场必须按真实相机和光照标定 `vision.yaml` 中的 HSV、Canny、最小轮廓和多帧参数。
可临时开启 `publish_debug_mask`，在 `/vision/debug_mask` 检查分割与边缘效果；标定完成后
关闭，以减少图像复制。

## 8. 点云地形与 Nav2 融合

`terrain_analyzer` 只保留最新一帧，将点云按消息时间戳转换到 `base_link`，裁剪机器人
正前方 ROI 并发布：

```text
/terrain/features              std_msgs/Float32MultiArray
/perception/obstacle_points    sensor_msgs/PointCloud2
/diagnostics                   diagnostic_msgs/DiagnosticArray
```

`/terrain/features` 字段：

```text
[ground_z, high_z, obstacle_height, valid_points, slope, roughness,
 frontal_obstacle_height, lookahead, traversability]
```

判定默认值：高度 `0.08 m` 起进入 `STEP`，`0.18 m` 起进入 `CLIMB`，`0.32 m` 起停止并
重规划；坡度、粗糙度也会使模式升级。阈值必须依据机器狗的实际腿长、质心、步态能力
和相机安装误差重新标定。

同一 ROI 被降采样后发布为 `/perception/obstacle_points`，Nav2 的 local costmap 以
`PointCloud2` 障碍源进行 marking，2D 雷达继续负责 marking + clearing。点云层不主动
clearing，防止短暂深度空洞错误清除障碍；激光清障和滚动窗口会移除离开视野的旧区域。

## 9. 速度与失效安全

速度链路固定为：

```text
/cmd_vel_nav -> /cmd_vel_smoothed -> /cmd_vel_terrain_safe -> /cmd_vel
```

- Nav2 controller 只发布 `/cmd_vel_nav`。
- Velocity Smoother 限制加速度并发布 `/cmd_vel_smoothed`。
- `cmd_vel_gate` 应用 `/crossing/speed_scale`，同时检查命令和决策心跳。
- Collision Monitor 读取 `/scan`，并作为 `/cmd_vel` 唯一发布者。

规划命令或越障决策任意一项超时，速度门都发布零速度。机器狗 SDK 还应实现独立的通信
看门狗、急停和姿态保护，不能仅依赖 ROS 进程。

## 10. Robocon 障碍赛模式

当前比赛状态机按提供的 V1.0 规则建立了 210 秒计时、障碍完成/重试、足端接触限制、
T 字台阶双向计分和返回启动区奖励。事件接口：

```text
/competition/obstacle_hint          std_msgs/String
/competition/obstacle_complete      std_msgs/Bool
/competition/obstacle_failed        std_msgs/Bool
/competition/retry                  std_msgs/Bool
/competition/returned_to_start      std_msgs/Bool
/competition/stair_levels_touched   std_msgs/Int32
/competition/stair_sides_completed  std_msgs/Int32
/foot_contacts                      std_msgs/UInt8MultiArray
```

`course_waypoints.yaml` 仍是调试坐标。获得正式场地测量结果后只修改 YAML，不要将点位
硬编码到节点。规则状态机管理流程，不代替实际足端接触检测和越障动作控制器。

## 11. 配置文件索引

| 配置 | 内容 |
|---|---|
| `quadruped_description/urdf/` | 尺寸、惯性、关节、传感器坐标系 |
| `quadruped_control/config/controllers.yaml` | ros2_control 控制器 |
| `quadruped_perception/config/vision.yaml` | HSV、Canny、多帧确认、图像资源限制 |
| `quadruped_perception/config/terrain.yaml` | 点云话题、ROI、采样和地形阈值 |
| `quadruped_planning/config/crossing.yaml` | 越障阈值、视觉融合、速度门超时 |
| `quadruped_planning/config/competition.yaml` | 比赛时间、计分和约束 |
| `quadruped_planning/config/course_waypoints.yaml` | 障碍接近点 |
| `slam/config/slam.yaml` | SLAM Toolbox |
| `slam/config/nav2.yaml` | Nav2、代价地图、速度平滑和碰撞监控 |
| `slam/config/sensor_profiles.yaml` | 常见雷达/相机话题 profile，可直接扩展 |

公共启动只有一份：`quadruped_bringup/launch/bringup.launch.py`；
`slam/launch/slam.launch.py` 在其上增加 SLAM、Nav2 与 RViz，避免重复维护节点。
`slam/launch/sensor_compat.launch.py` 只负责硬件名称适配，并包含上述标准入口。

## 12. 实机接入清单

1. 用实测值替换 URDF 尺寸、质量和惯性。
2. 将 mock ros2_control 替换为厂商硬件接口或自研控制器。
3. 标定关节零位、IMU、雷达、RGB/深度相机内外参和时间同步。
4. 检查 `/odom`、`odom -> base_link`、传感器 TF 的方向、频率和协方差。
5. 录制 rosbag，在离线数据上标定 HSV、点云 ROI、高度和坡度阈值。
6. 检查 local costmap 中激光与 `/perception/obstacle_points` 是否准确重合。
7. 空载验证断相机、断雷达、断里程计、决策超时、急停和恢复行为。
8. 先低速单障碍测试，再接入真实跨越动作和足端接触反馈。

目前的 OpenCV 是可解释的规则识别，适合起步、比赛固定障碍和 RK3588 低负载运行，
但准确率取决于视角、光照与标定。需要更强泛化时，应先采集误检/漏检数据，再决定是否
增加学习模型，而不是直接让视觉模型控制机器狗动作。

项目根目录维护四份互补文档：

- `README.md`：安装、启动、总体架构和使用入口。
- `instruction.txt`：各模块作用、算法原理和现场调试顺序。
- `connect.txt`：全部节点、话题、消息类型、字段、Action、Service、TF、QoS 和超时合同。
- `quickstart.txt`：跨电脑完整复现、启动命令、当前成果、常见问题和后续开发路线。

后续每次完成修改都同步检查这四份文档：接口变化更新 `connect.txt`，算法或职责变化
更新 `instruction.txt`，启动和使用方式变化更新 `README.md`，完成进度与后续计划变化
更新 `quickstart.txt`。验证通过后直接提交并推送 `main`，不使用强制推送。
