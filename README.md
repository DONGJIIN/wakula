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

## 四足整机研发状态与待完成工作

> **当前只完成了上层 ROS 2 软件雏形，不是一台已经完成的四足机器人。** SLAM、Nav2、
> OpenCV 和点云解决的是“看环境、定位置、选路线、发动作请求”；机器能否稳定站立、行走、
> 抬腿和越障，还取决于机械、电气、驱动、状态估计及运动控制等尚未完成的整机系统。

| 整机子系统 | 当前状态 | 还需要完成 |
|---|---|---|
| 需求与总体方案 | 🟡 部分完成 | 确定整机尺寸、重量、速度、续航、载荷、障碍指标、成本和比赛验收标准 |
| 机械结构 | ⬜ 仅有通用 URDF 外形 | 设计机身、腿部、关节、轴承、限位、防护、散热和加工装配，完成强度/刚度校核 |
| 关节执行器与传动 | ⬜ 未接入 | 选型电机、减速器、编码器和驱动器，确定扭矩/转速余量、零位、摩擦及温升模型 |
| 电气与电源 | ⬜ 未设计 | 电池、BMS、DC-DC、配电、保险、线束、接地、急停、充电和功耗/续航验证 |
| 嵌入式与底层驱动 | ⬜ 未实现 | MCU/实时控制、CAN/串口/EtherCAT 通信、电机闭环、采样同步、故障码和底层看门狗 |
| 计算平台与网络 | 🟡 目标为 RK3588 | 完成载板、存储、散热、启动服务、DDS 网络、时钟同步和长期稳定性测试 |
| ROS 2 硬件接口 | 🟡 接口已预留 | 接入具体机器狗 SDK 或 ros2_control hardware plugin，打通关节、IMU、足端力和电池状态 |
| 运动学与动力学模型 | ⬜ 未实现 | 实测 URDF/惯量，完成正逆运动学、雅可比、动力学、重力补偿、关节/力矩限制 |
| 状态估计 | ⬜ 只有 `/odom` 接口 | 融合编码器、IMU、足端接触和视觉/雷达，输出可靠姿态、速度、落足状态及 `odom -> base_link` |
| 站立与基础步态 | ⬜ 未实现 | 上电标定、站起/趴下、姿态稳定、原地踏步、行走/转向、步态切换和跌倒恢复 |
| 全身与越障控制 | ⬜ 仅发布动作请求 | 实现 IK、落脚点规划、MPC/WBC、摆动腿轨迹、接触力控制，以及 STEP/CLIMB/LOW_PROFILE 真动作 |
| 环境感知 | 🟡 软件雏形完成 | 真机标定雷达/相机，完成 RGB-深度同步、障碍跟踪、高程图、坡面和可落脚区域识别 |
| SLAM 与自主导航 | 🟡 软件雏形完成 | 使用真实 `/scan`、`/odom` 和 TF 调参，验证重定位、动态避障、狭窄通道及失效恢复 |
| 任务与比赛逻辑 | 🟡 状态机雏形完成 | 测量正式场地坐标，联动真实越障反馈，完善失败重试、计时、计分和任务恢复 |
| 整机安全 | 🟡 只有 ROS 超时停车 | 增加硬件急停、驱动失能、过流/过温/欠压保护、姿态/关节保护和通信断链保护 |
| 仿真与测试 | 🟡 只有 RViz 和单元测试 | 建立 Gazebo/Isaac 物理模型、传感器噪声、完整赛道、SIL/HIL、回归和故障注入测试 |
| 真机联调与工程化 | ⬜ 未完成 | 架空→保护绳→低速→单障碍→整场测试，完成标定工具、日志、rosbag、CI、版本和维护流程 |

当前代码已经完成的是上表中“环境感知、SLAM 与自主导航、任务逻辑、ROS 软件安全”的
第一版雏形：6 个 ROS 2 包可编译，19 项测试通过，并提供常见雷达/相机兼容入口。
其余项目不能因仿真话题或 URDF 能运行就视为完成。详细清单与开发顺序见
`quickstart.txt` 第七至九节。

### 后续工作的实施内容与阶段验收

下面各阶段必须按顺序推进；上一阶段没有形成可检查的交付物，不应直接进入高速整机测试。

1. **阶段 0：冻结需求和总体方案**
   - 工作：确定自研整机还是购买平台二次开发；冻结自由度、尺寸、质量、速度、续航、载荷、最大台阶/坡度、预算及比赛指标。
   - 工作：完成机械、电气、控制、传感器、计算平台和通信框图，建立质量、功耗、算力、带宽及成本预算。
   - 交付物：需求规格书、系统框图、接口清单、初版 BOM、风险清单、研发负责人和版本基线。
   - 验收：每项指标可测量，电机/电池/RK3588/传感器选型有余量依据，接口不存在无人负责的空白。

2. **阶段 1：机械、电气和单关节验证**
   - 工作：完成机身、三段腿、关节安装、轴承、限位、外壳、散热、线束活动空间和可维护性设计，并做强度/刚度校核。
   - 工作：完成电池、BMS、DC-DC、配电、保险、急停、充电、接地和峰值电流设计。
   - 工作：搭建单关节台架，测量扭矩、速度、回差、摩擦、编码器零位、温升、效率和过载行为。
   - 交付物：CAD/加工图、装配图、BOM、原理图/线束图、单关节测试记录和保护阈值。
   - 验收：关节在目标载荷下稳定闭环，温度和电流不超过设计值；断电、急停和超限能进入安全状态。

3. **阶段 2：嵌入式驱动和 ROS 2 硬件层**
   - 工作：实现 MCU/实时控制器的电流、速度、位置闭环，打通 CAN/串口/EtherCAT 与 12 个关节。
   - 工作：统一编码器、IMU、足端力、电池、温度、故障码和时间戳，完成上电标定及零位保存。
   - 工作：实现厂商 SDK 适配器或 `ros2_control` hardware plugin，落实命令超时、驱动失能和诊断上报。
   - 交付物：通信协议、固件、硬件接口包、标定工具、故障码表、台架测试和抓包记录。
   - 验收：所有关节可单独使能/失能和低增益跟踪；反馈连续；拔线、超时、过流、过温及急停均不会保持危险输出。

4. **阶段 3：模型、状态估计和基础步态**
   - 工作：实测连杆尺寸、质量、质心、惯量、关节方向和限位，替换通用 URDF 数据。
   - 工作：实现正逆运动学、雅可比、动力学/重力补偿，以及编码器、IMU、足端接触融合状态估计。
   - 工作：按架空、保护绳、低增益顺序实现站起、趴下、姿态保持、踏步、行走、转向、步态切换和跌倒恢复。
   - 交付物：标定后的模型、状态估计节点、基础步态控制器、参数集、日志和安全测试报告。
   - 验收：机器人能在保护条件下重复站立与低速行走；姿态/速度估计连续；失足、超姿态和通信中断可停止或恢复。

5. **阶段 4：传感器和自主导航真机化**
   - 工作：安装并标定 2D/3D 雷达、RGB-D 相机和 IMU，完成 TF、内参、外参、时钟同步及遮挡检查。
   - 工作：生成可靠 `/odom` 与 `odom -> base_link`，采集 rosbag 调整 SLAM、Nav2、OpenCV、点云 ROI 和碰撞保护参数。
   - 工作：验证建图、保存/加载地图、重定位、动态避障、窄通道、低纹理/反光环境和传感器失效恢复。
   - 交付物：传感器安装图、标定文件、标准话题/TF、参数包、测试地图、rosbag 和重复路线报告。
   - 验收：多次执行同一路线结果稳定；传感器断流时不误激活导航且速度归零；地图、激光、点云和模型在 RViz 中重合。

6. **阶段 5：真实越障和全身控制**
   - 工作：建立局部高程图，识别台阶、坡面、坑洞、边缘和可落脚区域，并完成 RGB—深度同步与障碍跟踪。
   - 工作：实现落脚点规划、摆动腿轨迹、MPC/WBC 或等价控制、接触力分配和滑移/碰撞检测。
   - 工作：将 `STEP/CLIMB/LOW_PROFILE` 从字符串请求升级为可反馈、可取消、可超时的 ROS 2 Action。
   - 交付物：高程/落脚模块、越障 Action、动作参数库、失败恢复策略和逐类障碍测试数据。
   - 验收：每类障碍完成重复低速试验；动作失败可取消并安全站立；越障期间普通 `/cmd_vel` 不会与动作控制争用。

7. **阶段 6：比赛、可靠性和工程交付**
   - 工作：建立 Gazebo/Isaac 动力学赛道，执行 SIL/HIL、自动回归、故障注入和规则状态机验证。
   - 工作：按单模块→单障碍→组合障碍→完整赛道递进测试，并在 RK3588 上记录 CPU、内存、温度、功耗、延迟和网络稳定性。
   - 工作：固化部署脚本、开机服务、标定流程、日志/rosbag、CI、版本、参数备份、备件和维护说明。
   - 交付物：正式场地参数、整场测试报告、性能/可靠性报告、发布镜像、操作手册、维修清单和验收记录。
   - 验收：完成规定工况的连续整场运行和断电/断网/传感器/驱动故障演练，能够从全新电脑按文档恢复同一版本。

### 接下来优先开展

1. 先明确“购买现成机器狗接 SDK”还是“自研机械、电气和驱动”；两条路线的工作量完全不同。
2. 填写整机目标参数和现有硬件清单，确认电机、驱动器、电池、IMU、足端力、雷达及相机的具体型号。
3. 获得 SDK/API、通信协议、URDF/CAD、关节限制和坐标系资料；没有这些资料前不编写真机动作控制。
4. 建立单关节台架和硬件急停，先验证低层闭环及保护，再连接全部关节。
5. 完成实机 URDF、关节零位和 IMU 标定，依次实现站立、低速行走与可靠 `/odom`。
6. 接入本仓库的标准传感器和速度接口，录制 rosbag，完成 SLAM/Nav2/感知真机调参。
7. 最后开发真实越障 Action、落脚点规划和 MPC/WBC，并逐个障碍低速验收。

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
