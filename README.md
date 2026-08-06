# Wakula 四足机器人自主越障工作空间

Wakula 是一个基于 ROS 2 Jazzy 的四足机器人自主导航与越障原型。工程按
“机器人描述 → 状态/控制 → 感知 → 规划 → 系统启动”的层次组织，支持 RViz
调试、SLAM + Nav2 导航和 Robocon 障碍赛规则状态机。

## 1. 目录结构

```text
wakula/
├── src/                                  # 所有 ROS 2 源码包
│   ├── quadruped_description/             # URDF/Xacro、RViz模型显示
│   ├── quadruped_bringup/                  # 基础硬件、感知和越障节点启动
│   ├── quadruped_control/                 # ros2_control控制器和关节限制
│   ├── quadruped_perception/              # 点云地形与OpenCV视觉特征提取
│   ├── quadruped_planning/                # 越障状态机、速度安全门、比赛管理
│   └── slam/                              # SLAM Toolbox、Nav2配置和启动文件
├── scripts/
│   ├── bootstrap.sh                        # 安装ROS依赖
│   └── build.sh                            # 编译整个工作空间
├── .colcon/defaults.yaml                   # colcon默认构建参数
├── .vscode/settings.json                   # VS Code工作区设置
└── README.md
```

`build/`、`install/`、`log/` 和 Python 缓存均为可重建产物，不纳入版本控制。

## 2. ROS 包职责

| 包 | 类型 | 作用 | 主要入口 |
|---|---|---|---|
| `quadruped_description` | CMake | 12自由度四足模型、传感器TF、mock ros2_control | `display.launch.py` |
| `quadruped_bringup` | Python | 启动模型、控制、感知和普通越障节点 | `bringup.launch.py` |
| `quadruped_control` | CMake | 控制器列表和关节限位 | `config/controllers.yaml` |
| `quadruped_perception` | Python | 点云、OpenCV颜色检测、可选YOLO接口 | `terrain_analyzer`、`vision_obstacle_detector` |
| `quadruped_planning` | Python | 普通越障模式、比赛规则FSM、Nav2速度安全门 | 见下方节点表 |
| `slam` | Python | SLAM Toolbox、Nav2参数和完整自主启动 | `slam.launch.py` |

### 主要节点

- `terrain_analyzer`：订阅点云，发布 `/terrain/features`。
- `vision_obstacle_detector`：使用 OpenCV 提取橙色/蓝色区域，发布视觉辅助特征和调试掩膜。
- `yolo_obstacle_detector`：可选的轻量 YOLO ONNX 检测节点，默认不启动。
- `obstacle_crossing_manager`：普通模式下发布 `WALK/STEP/CLIMB/STOP`。
- `competition_obstacle_manager`：比赛限时、障碍完成、重试和计分。
- `cmd_vel_gate`：对 Nav2 速度进行缩放和超时/危险停止。
- `course_waypoint_navigator`：比赛模式下调用 Nav2 到达障碍接近点。

## 3. 数据流

```text
3D雷达/深度相机 ─> terrain_analyzer ─> terrain/features ─┐
RGB相机 ─> OpenCV视觉节点 ─> vision/color_features ──────┤
       └─> YOLO（默认关闭，后期RKNN/NPU）─> yolo/detections
2D雷达 /scan ─> SLAM Toolbox ─> map/odom ─> Nav2         ├─> crossing manager
                                                │        │
                                                └─> cmd_vel_nav ─> cmd_vel_gate ─> 机器狗SDK
```

真实硬件必须提供：

```text
/scan                         sensor_msgs/LaserScan
/camera/depth/color/points   sensor_msgs/PointCloud2
/camera/color/image_raw      sensor_msgs/Image
/odom                         nav_msgs/Odometry
odom -> base_link             TF
```

机器狗驱动负责订阅 `/cmd_vel`，并将速度或越障动作转换为厂商 SDK、步态控制器
或自研 MPC/WBC 的接口。

## 4. 安装与编译

系统要求：Ubuntu 24.04、ROS 2 Jazzy、Python 3、colcon。视觉节点依赖
`python3-opencv`、`python3-numpy` 和 `ros-jazzy-cv-bridge`，均由安装脚本根据
`package.xml` 安装。

```bash
cd ~/wakula
source /opt/ros/jazzy/setup.bash
./scripts/bootstrap.sh
./scripts/build.sh
source install/setup.bash
```

手动构建等价命令：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

## 5. 调试启动

只查看机器人模型：

```bash
ros2 launch quadruped_description display.launch.py
```

启动 SLAM + Nav2 + 普通越障链路：

```bash
ros2 launch slam slam.launch.py
```

无图形界面测试：

```bash
ros2 launch slam slam.launch.py rviz:=false
```

没有 RGB 相机时可关闭 OpenCV 节点：

```bash
ros2 launch slam slam.launch.py vision:=false
```

启动 Robocon 障碍赛模式：

```bash
ros2 launch slam slam.launch.py competition:=true
```

YOLO 默认关闭。只有准备好兼容的 ONNX 模型后才用于开发机验证：

```bash
ros2 launch slam slam.launch.py \
  yolo:=true \
  yolo_model:=/绝对路径/obstacles_nano.onnx
```

启动前请确认传感器、里程计和 TF 已经发布；否则 Nav2 会等待 `odom → base_link`，
这是预期行为。

## 6. 地形特征接口

`/terrain/features` 为 `std_msgs/Float32MultiArray`，数组字段固定为：

```text
[ground_z,
 high_z,
 obstacle_height,
 valid_points,
 slope,
 roughness,
 frontal_obstacle_height,
 lookahead,
 traversability]
```

地形数据不足或超过传感器超时后，越障管理器和速度门默认进入 `STOP`，这是安全设计。

## 7. OpenCV 视觉接口

`vision_obstacle_detector` 订阅 `/camera/color/image_raw`，在 HSV 色彩空间提取橙色和
蓝色区域，并发布：

```text
/vision/color_features   std_msgs/Float32MultiArray
/vision/color_mask       sensor_msgs/Image
```

`/vision/color_features` 的字段固定为：

```text
[orange_area_ratio, orange_cx, orange_cy, orange_width_ratio, orange_height_ratio,
 blue_area_ratio,   blue_cx,   blue_cy,   blue_width_ratio,   blue_height_ratio]
```

坐标和尺寸均已归一化到 `0.0～1.0`；未检测到对应颜色时，该颜色的五个字段为零。
HSV 阈值、最小区域面积、形态学滤波尺寸和相机话题位于
`src/quadruped_perception/config/vision.yaml`。比赛现场光照变化较大，必须用实际相机
重新标定 HSV 范围。颜色结果只作为障碍提示，不代替点云测高、可通行性分析和足端
接触判断。

普通越障模式已经加入视觉—点云安全融合：当橙色或蓝色区域面积超过阈值且位于
图像中央，而点云仍判断可以正常行走时，系统保持 `WALK`，但将动作改为
`VERIFY_VISUAL_OBSTACLE_WITH_DEPTH` 并按 `vision_speed_scale` 减速。只有点云几何
信息能够触发 `STEP` 或 `CLIMB`；无效或超时的点云仍触发 `STOP`。视觉结果超时后
自动退出辅助状态，因此没有 RGB 相机也不会阻塞点云越障链路。

融合状态可通过 `/crossing/visual_assist_active`（`std_msgs/Bool`）观察。相关参数位于
`src/quadruped_planning/config/crossing.yaml`：

```text
vision_assist_enabled      是否启用视觉辅助
vision_timeout             视觉结果有效时间（秒）
vision_min_area_ratio      触发减速的最小画面面积比例
vision_center_margin       忽略图像两侧区域的比例
vision_speed_scale         等待深度确认时的速度倍率
```

## 8. 可选 YOLO 后期方案

YOLO 节点已经接入工程，但 `slam.launch.py` 和 `bringup.launch.py` 中的 `yolo` 参数
默认均为 `false`。默认启动时不会创建 YOLO 进程、不会读取模型，也不会订阅相机，
因此不会给 RK3588 增加运行压力。工程也没有安装 PyTorch、Ultralytics 或 ONNX
Runtime；桌面验证直接复用现有 OpenCV DNN。

资源限制位于 `src/quadruped_perception/config/yolo.yaml`：

```text
input_width/input_height  320×320
inference_hz              5 Hz，上限限制为10 Hz
opencv_threads            2，上限限制为4
max_detections            每帧最多20个目标
publish_debug_image       false，避免额外图像拷贝
```

输出接口：

```text
/vision/yolo/detections    vision_msgs/Detection2DArray
/vision/yolo/inference_ms  std_msgs/Float32
/vision/yolo/debug_image   sensor_msgs/Image  # 仅显式开启调试图时发布
```

节点兼容常见 YOLOv8/YOLO11 ONNX 输出；使用带 objectness 的 YOLOv5 导出模型时，将
`output_has_objectness` 设为 `true`。标签文件为每行一个类别名，通过 `labels_path`
指定。模型文件较大且与硬件相关，`.onnx`、`.rknn` 和 `.pt` 已排除在 Git 之外。

RK3588 正式部署建议训练 `nano` 级自定义障碍模型，将其量化转换为 RKNN，并使用
RK3588 NPU 推理。当前 OpenCV DNN 后端仅用于功能验证；在真机上启用前必须实测
`/vision/yolo/inference_ms`、CPU 占用、温度和导航延迟。YOLO 检测结果目前不参与
越障动作决策，后续也应先与深度/点云匹配，不能直接触发攀爬或跳跃。

## 9. Robocon 障碍赛模式

比赛规则状态机按照提供的 V1.0 规则实现：

- 比赛限时 210 秒。
- 支持直角绕杆、砂砾碎木坑、限高杆、斜坡、木桥 A、木桥 B、T 字台阶和高墙。
- 障碍可以不按固定顺序完成，重复完成不重复计分。
- 自动模式每个完整障碍 150 分，遥控模式按 100 分计算。
- T 字台阶只完成单向计 75 分，完成上下两个方向计 150 分。
- 全部障碍完成并返回选定启动区，额外增加 100 分。
- 砂砾坑、T 台阶、斜坡和木桥越障时，最多允许一个足端接触地面。
- 斜坡有效行走距离至少 1 米；T 台阶要求每级顶面有足端接触。

比赛事件接口：

```text
/competition/obstacle_hint          std_msgs/String
/competition/obstacle_complete      std_msgs/Bool
/competition/obstacle_failed        std_msgs/Bool
/competition/retry                  std_msgs/Bool
/competition/returned_to_start      std_msgs/Bool
/competition/stair_levels_touched   std_msgs/Int32
/competition/stair_sides_completed  std_msgs/Int32  # 0/1/2
/foot_contacts                      std_msgs/UInt8MultiArray
```

比赛路线接近点位于 `src/quadruped_planning/config/course_waypoints.yaml`，目前是
调试占位值。赛前获得正式场地尺寸、障碍位置和方向后，只修改这个配置文件，不要把
场地坐标硬编码到 Python 节点中。

## 10. 配置文件归属

- 机器人尺寸、质量、惯性、关节名称：`quadruped_description/urdf/`。
- 控制器和关节限位：`quadruped_control/config/`。
- 点云ROI和地形阈值：`quadruped_perception/config/terrain.yaml`。
- OpenCV相机话题、HSV颜色范围和滤波参数：`quadruped_perception/config/vision.yaml`。
- 可选YOLO模型接口和资源限制：`quadruped_perception/config/yolo.yaml`。
- 普通越障阈值和速度门：`quadruped_planning/config/crossing.yaml`。
- 比赛时间、障碍顺序和计分：`quadruped_planning/config/competition.yaml`。
- 比赛障碍接近点：`quadruped_planning/config/course_waypoints.yaml`。
- SLAM和Nav2：`slam/config/`。

## 11. 接入真实机器狗前检查

1. 用实测数据替换 URDF 中的尺寸、质量和惯性。
2. 将 mock ros2_control 替换为厂商硬件插件。
3. 标定关节零位、IMU、雷达、相机外参和 TF 时间戳。
4. 确认 `/odom` 和 `odom → base_link` 的方向、频率和协方差。
5. 将点云坐标统一为前方 `x`、侧向 `y`、上方 `z`。
6. 用低电流单腿测试，再开启全身控制。
7. 在仿真和空载场景验证急停、速度超时和 `STOP` 行为。
8. 最后接入真实越障动作、足端接触检测和落脚规划器。

当前工程是自主越障软件框架，不包含具体机器狗厂商 SDK，也不应直接用于未经验证的实机高速运行。
