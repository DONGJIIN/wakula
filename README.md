# Wakula 四足机器人自主导航与越障

Wakula 是面向 Ubuntu 24.04、ROS 2 Jazzy 和 RK3588 的四足机器人调试工作空间。
当前版本以 **2D 雷达 SLAM + Nav2 路径规划 + 深度点云地形分析 + OpenCV 障碍提示 +
自主前沿探索 + 越障任务编排**构成轻量融合链路，不包含 YOLO、深度学习推理、硬件驱动
或腿部运动控制。

现阶段只开发硬件无关的环境感知与自主导航底座，并提供一个与算法完全解耦的 Gazebo
比赛障碍参考场地。仓库没有实现机器狗动力学仿真、厂家 SDK、`ros2_control`、状态估计、
FK/IK、站立步态、全身控制或真实越障动作；这些内容等真机结构、执行器和传感器选型
确定后再开发。

## 文档导航（先看这里）

| 文件 | 功能 | 适合什么时候看 |
|---|---|---|
| `README.md` | 项目简介、总体架构、安装和启动入口 | 第一次了解或运行项目 |
| `quickstart.txt` | 新电脑完整复现、依赖插件、当前进度和后续路线 | 换电脑部署或接手开发 |
| `instruction.txt` | 各模块作用、算法原理、参数和调试步骤 | 修改 SLAM、Nav2、OpenCV 或越障算法 |
| `connect.txt` | 节点作用、关键输入输出、接口字段和真机通信约定 | 接入相机、雷达、真机或排查通信 |

## 四足整机研发状态与待完成工作

> **当前只完成了上层 ROS 2 软件雏形，不是一台已经完成的四足机器人。** SLAM、Nav2、
> OpenCV 和点云解决的是“看环境、定位置、选路线、生成安全导航约束”；机器能否稳定站立、行走、
> 抬腿和越障，还取决于机械、电气、驱动、状态估计及运动控制等尚未完成的整机系统。

| 整机子系统 | 当前状态 | 还需要完成 |
|---|---|---|
| 需求与总体方案 | 🟡 部分完成 | 确定整机尺寸、重量、速度、续航、载荷、障碍指标、成本和比赛验收标准 |
| 机械结构 | ⬜ 仅有通用 URDF 外形 | 设计机身、腿部、关节、轴承、限位、防护、散热和加工装配，完成强度/刚度校核 |
| 关节执行器与传动 | ⬜ 未接入 | 选型电机、减速器、编码器和驱动器，确定扭矩/转速余量、零位、摩擦及温升模型 |
| 电气与电源 | ⬜ 未设计 | 电池、BMS、DC-DC、配电、保险、线束、接地、急停、充电和功耗/续航验证 |
| 嵌入式与底层驱动 | ⬜ 未实现 | MCU/实时控制、CAN/串口/EtherCAT 通信、电机闭环、采样同步、故障码和底层看门狗 |
| 计算平台与网络 | 🟡 目标为 RK3588 | 完成载板、存储、散热、启动服务、DDS 网络、时钟同步和长期稳定性测试 |
| ROS 2 硬件接口 | ⬜ 未实现，等待真机 | 接入具体机器狗 SDK 或 ros2_control hardware plugin，打通关节、IMU、足端力和电池状态 |
| 机器人仿真模型与参数标定 | ⬜ 未实现，等待实测参数 | 根据真实机器人结构建立 URDF/仿真模型，配置质量、惯量、关节、碰撞、摩擦、执行器及 PD 参数，为强化学习训练和 Sim-to-Real 提供基础 |
| 强化学习训练环境 | ⬜ 未实现，等待真机 | 基于 Isaac Gym/Isaac Sim 等搭建强化学习环境，设计观测空间、动作空间、奖励函数、终止条件及训练任务 |
| 复杂地形运动控制 | ⬜ 未实现，等待真机 | 通过地形随机化和扰动训练，使强化学习策略适应平地、斜坡、台阶、坑洼、连续障碍及非结构化地形，实现速度跟踪、姿态稳定和足端协调 |
| 环境感知 | 🟡 软件雏形完成 | 真机标定雷达/相机，完成 RGB-深度同步、障碍跟踪、高程图、坡面和可落脚区域识别 |
| SLAM 与自主导航 | 🟡 软件雏形完成 | 使用真实 `/scan`、`/odom` 和 TF 调参，验证重定位、动态避障、狭窄通道及失效恢复 |
| 任务与比赛逻辑 | 🟡 自主探索雏形完成 | 已能从未知地图选择前沿、逐个接近确认障碍并经 Action 交接后继续探索；仍需接入真实越障反馈、正式障碍顺序、失败重试、计时、计分、返回起点和裁判接口 |
| 整机安全 | ⬜ 仅有导航速度超时门 | 实现并验证硬件急停、驱动失能、过流/过温/欠压及真实姿态/关节保护 |
| 仿真与测试 | 🟡 已有独立 Gazebo 参考场地 | 已复现规则 V1.0 已公布尺寸/颜色并提供传感器测试载体；正式坐标、真机动力学、Isaac、SIL/HIL 仍待后续完成 |
| 真机联调与工程化 | 🟡 CI 与 rosbag 评估工具已有 | 架空→保护绳→低速→单障碍→整场测试，完成部署服务、日志策略和维护流程 |

当前代码完成的是环境感知、SLAM/Nav2、传感器通用 profile、导航健康检查、保守地形
决策、速度超时门、未知地图前沿探索、Nav2 越障入口接近、`TraverseObstacle` Action 编排、
Xbox 手柄适配、独立比赛场地、强类型真机对接合同、rosbag 离线评估和全栈长时间回归工具：
9 个 ROS 2 包可编译，151 项测试通过，并提供一键启动、独立停止、对接检查和 CI。URDF 只用于 RViz 外形与
传感器 TF 占位，
不能视为运动学或整机控制已完成。
详细清单与开发顺序见 `quickstart.txt`。

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
   - 工作：建立强化学习策略与底层执行器之间的接口，将策略输出转换为安全的关节目标或执行器控制指令。
   - 交付物：通信协议、固件、硬件接口包、标定工具、故障码表、台架测试和抓包记录。
   - 验收：所有关节可单独使能/失能和低增益跟踪；反馈连续；拔线、超时、过流、过温及急停均不会保持危险输出。

4. **阶段 3：模型、状态估计和基础步态**
   - 工作：实测机器人连杆尺寸、质量、质心、惯量、关节方向、关节限位和执行器参数，建立与真实机器狗对应的 URDF/仿真模型。
   - 工作：将真实机器人参数映射到 Isaac Gym/Isaac Sim 等强化学习环境，配置质量、惯量、碰撞、摩擦、关节阻尼、执行器响应、PD 参数和控制频率。
   - 工作：建立强化学习训练所需的本体状态输入，包括关节位置、关节速度、机身角速度、重力方向、速度指令以及足端接触等信息。
   - 工作：构建基础状态估计接口，为强化学习策略提供稳定、统一、具有正确时间戳的观测数据。
   - 工作：不以传统完整动力学建模、MPC/WBC 或解析动力学控制器作为本阶段目标；重点是建立能够支撑强化学习训练和 Sim-to-Real 的机器人仿真环境。
   - 交付物：实测机器人参数集、URDF、仿真模型、观测接口、状态估计节点、强化学习环境和参数配置文件。
   - 验收：仿真机器人结构、关节方向、质量和运动范围与真实机器人一致；观测接口完整；仿真环境能够稳定运行并支持批量强化学习训练。

5. **阶段 4：传感器和自主导航真机化**
   - 工作：安装并标定 2D/3D 雷达、RGB-D 相机和 IMU，完成 TF、内参、外参、时钟同步及遮挡检查。
   - 工作：生成可靠 `/odom` 与 `odom -> base_link`，采集 rosbag 调整 SLAM、Nav2、OpenCV、点云 ROI 和碰撞保护参数。
   - 工作：验证建图、保存/加载地图、重定位、动态避障、窄通道、低纹理/反光环境和传感器失效恢复。
   - 交付物：传感器安装图、标定文件、标准话题/TF、参数包、测试地图、rosbag 和重复路线报告。
   - 验收：多次执行同一路线结果稳定；传感器断流时不误激活导航且速度归零；地图、激光、点云和模型在 RViz 中重合。

6. **阶段 5：真实越障和全身控制**
   - 工作：建立局部高程图，识别台阶、坡面、坑洞、边缘和可落脚区域，并完成 RGB—深度同步与障碍跟踪。
   - 工作：实现落脚点规划、摆动腿轨迹、MPC/WBC 或等价控制、接触力分配和滑移/碰撞检测。
   - 工作：设计并实现真机越障接口，接入全身控制器，完成落脚、接触和姿态闭环。
   - 交付物：高程/落脚模块、越障控制接口、动作参数库、失败恢复策略和逐类障碍测试数据。
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
7. 最后设计越障控制接口，接入落脚点规划和 MPC/WBC，并逐个障碍低速验收。

## 1. 目录结构

```text
wakula/
├── src/
│   ├── quadruped_description/  # 12 自由度 URDF/Xacro、RViz、传感器 TF
│   ├── quadruped_gazebo/       # 独立规则场地、传感器测试载体与 Gazebo launch
│   ├── quadruped_bringup/      # 感知、决策、速度门和占位 TF 统一入口
│   ├── quadruped_interfaces/   # 感知消息与 TraverseObstacle Action 合同
│   ├── quadruped_perception/   # OpenCV 视觉及 PointCloud2 地形分析
│   ├── quadruped_planning/     # 地形决策、入口引导与自主探索任务
│   ├── quadruped_teleop/       # Xbox /joy 到独立 /cmd_vel_joy 的安全适配
│   ├── quadruped_tools/        # rosbag 准确率评估与全栈长时间回归
│   └── slam/                   # 核心算法入口与完全独立的自主导航 launch
├── scripts/
│   ├── record_bag.sh           # 记录感知、导航与诊断数据
│   ├── diagnose.sh             # 检查 ROS 话题和 TF
│   ├── check_integration.sh    # 一键核验未来真机对接合同
│   └── build.sh                # 统一编译
├── .colcon/defaults.yaml
├── .github/workflows/ros2-ci.yaml
├── .vscode/settings.json
└── README.md
```

`build/`、`install/`、`log/` 与 Python 缓存都是可重建产物，不提交到 Git。

## 2. 各模块职责

| 包 | 主要职责 | 关键入口 |
|---|---|---|
| `quadruped_description` | 未标定的 RViz 外形和雷达/相机占位坐标系 | `display.launch.py` |
| `quadruped_gazebo` | 独立比赛障碍 world、测试载体与传感器数据源，不启动任何算法 | `robocon_field.launch.py` |
| `quadruped_bringup` | 感知、地形决策、速度门和占位模型公共入口 | `bringup.launch.py` |
| `quadruped_interfaces` | 带时间戳的感知、导航与越障交接合同 | 五个 `msg/` + `TraverseObstacle.action` |
| `quadruped_perception` | OpenCV、栅格地面分割、几何分类、时间同步融合 | 三个感知节点 |
| `quadruped_planning` | 地形风险分类、入口引导、速度安全门和未知地图自主任务 | 四个导航/任务节点 |
| `quadruped_teleop` | Xbox 按键安全状态机和独立 Twist 候选 | `xbox_teleop` |
| `quadruped_tools` | rosbag 标注评估、SLAM/Nav2 长测与资源报告 | `perception_bag_evaluator`、`stack_regression` |
| `slam` | 核心 SLAM/Nav2/OpenCV 入口，以及需显式启动的独立自主导航入口 | 两个互不 include 的 launch |

主要节点：

- `vision_obstacle_detector`：OpenCV 双光照 HSV + Canny 轮廓识别，并用原始曝光、清晰度、
  高光抑制、Hue 0/179 环绕、投票率和目标框 IoU 做多帧确认；同时发布带 ROI、候选框、
  稳定类别和图像质量的 `/vision/annotated_image`，供 RViz 直接观察。限高杆还要求蓝色、
  细长比例和合理画面占比；无颜色的单条水平边缘不能独立判为横杆，避免地平线和地面边界误报。
- `terrain_analyzer`：将点云转换到 `base_link`，以稳健高度栅格、连通域和原始回波支撑量
  估计台阶、坡度、坑洞、墙、悬空横杆和立柱；送入 Nav2 前按拟合地面移除平地/坡面。
- `perception_fusion`：在小队列中全局寻找时间戳最接近的相机/点云对；相机断流时在
  0.25 s 后退化为纯点云结果；视觉框还必须与前向通道相交，点云始终掌握尺度权限。
- `terrain_safety_assessor`：优先读取按时间戳配对的融合观测，原子发布地形模式、Nav2
  速度上限、有效性与几何摘要，并在终端周期显示正前方障碍中文名称、置信度、距离、
  高度和视觉介入状态，供调试及未来运动团队只读接入。名称优先采用点云量测：可显示
  主斜坡、木桥引坡、T 字形台阶、限高杆、直角绕杆区、坑区和高墙；仅有颜色证据时明确
  标注“点云待分类”，不会把笼统视觉结果冒充具体障碍。
- `traversal_guidance`：把已确认的赛道障碍转换为 `APPROACH → ALIGN → READY`，发布
  `base_link` 中位于障碍前方的相对入口位姿；它不调用 Nav2 Action，也不生成抬腿、关节
  或足端命令。同类目标的距离和横向中心经过低通，READY 需要连续 3 帧确认并具有独立的
  距离/角度退出迟滞；输入失效立即撤销历史。`autonomous_mission` 据此向 Nav2 下发入口
  目标，READY 后再交给运动控制器。
- `autonomous_mission`：从 `/map` 的已知—未知边界提取真实自由前沿，在 Nav2 处于活动状态
  后逐个探索；连续确认比赛障碍时冻结其 `map` 坐标，先到达入口，再调用
  `/traverse_obstacle`。Action 成功后记录已完成障碍并选择下一前沿；Action 拒绝、超时、
  Nav2 取消及运行中 STOP 均有显式状态，不读取 Gazebo 模型名或 world 坐标。
- `navigation_health_monitor`：运行期检查 `/scan`、`/odom`、TF、扫描结构、frame、协方差
  和里程计突跳；跳变会锁存到连续稳定样本确认恢复。
- `nav2_readiness_monitor`：复用同一数据合同，等待有效 `/scan`、`/odom` 和定位 TF 后激活 Nav2。
- `navigation_speed_gate`：检查 Nav2 命令、地形评估和导航健康心跳，任一失效立即输出零速。
- `xbox_teleop`：将 `/joy` 转换为带 LB 使能、B 急停和断流归零的 `/cmd_vel_joy`。
- `perception_bag_evaluator`：将 rosbag 预测与人工标签对齐，统计准确率、召回率和混淆矩阵。
- `stack_regression`：在明确允许仿真运动后自动执行连续旋转、前进、倒退、回环、多目标、
  1 m 绕杆窄通道和不可达目标恢复，并记录数据断流、闭环误差、越障阶段稳定性/安全合同、
  CPU 与常驻内存峰值。

## 3. SLAM、Nav2、OpenCV 与点云如何协同

导航采用 Nav2 标准的“全局规划器 + 局部规划器”两层框架：`NavFnPlanner` 在 SLAM 地图和
全局代价地图上生成整段路径，`DWBLocalPlanner` 在局部代价地图上结合实时 `/scan` 与
去地面点云跟踪路径、避开新障碍。OpenCV/点云为规划层补充类别、地形与安全约束，不替代
这两个规划器。

```text
2D 雷达 /scan ──> SLAM Toolbox ──> /map + map→odom
       │                                  │
       └──────────────────────────────────> Nav2 全局/局部规划

RGB 相机 ──> OpenCV ──> /vision/obstacle_stamped ─┐
                                                  ├─> 时间同步融合
深度点云 ──> 地面分割 ──> /terrain/features_stamped┘       │
                          └─> /terrain/features（无相机兼容）│
                     /perception/fused_obstacle <───────────┘
                                      └─> terrain safety assessor + rosbag
                                           └─> /terrain/navigation_safety
                                                └─> traversal guidance
                                                     ├─> /traversal/guidance
                                                     └─> /traversal/approach_pose
                                                             │
/map + map→base_link ─> autonomous mission <─────────────────┘
                         ├─> Nav2 /navigate_to_pose
                         └─> /traverse_obstacle ─> 未来真机越障控制器
       └────────────────> /perception/obstacle_points ─> Nav2 local_costmap

Nav2 /cmd_vel_nav ─> velocity_smoother ─> navigation_speed_gate
     ─> collision_monitor ─> /cmd_vel ─> 未来真机底盘接口

Xbox /joy ─> xbox_teleop ─> /cmd_vel_joy ─> 未来 twist_mux 仲裁 ─┐
                                                               └─> collision_monitor

/scan + /odom + TF ─> navigation health + Nav2 readiness
```

职责边界如下：

1. **SLAM** 用 `/scan` 在陌生环境生成地图和定位，不负责跨越动作。
2. **Nav2** 根据地图规划自由空间路线。普通环境障碍继续正常避碰；赛道越障目标不把终点
   直接设在实体后方，而是先使用 `/traversal/approach_pose` 到达并对正入口。激光和“高于
   局部拟合地面的深度点”仍写入代价地图，防止运动控制器接管前误闯障碍。
3. **OpenCV** 识别杆、限高横杆、墙面和大面积有色障碍，用于提前减速和提示。
4. **深度点云** 测量障碍高度、坡度、横向偏移和粗糙度，是 `STEP/CLIMB/STOP` 及入口
   对正的几何依据；当前任务层会完成入口导航和 Action 交接，但不生成真实跨越动作。
5. **Collision Monitor** 是 `/cmd_vel` 的唯一发布者，负责最后一层碰撞保护。
6. **Xbox 手柄节点** 默认独立发布 `/cmd_vel_joy`，不加入主导航 launch，也不绕过
   Collision Monitor；真机阶段通过 `twist_mux` 与 Nav2 速度仲裁。
7. **自主任务节点** 只使用 `/map`、TF 和感知输出决定“去哪里、何时交接”，不包含步态；
   停止服务会取消当前 Nav2/越障目标并发布零速，之后可以继续启动。

OpenCV 不估计真实距离，也不能独立触发抬腿或跳跃。只有视觉和点云时间上有效、且
点云确认几何条件后，才进入对应越障模式；点云缺失、无效或超时默认 `STOP`。

### 3.1 与未来硬件、运动控制团队的一键对齐边界

本仓库采用标准 ROS 2 接口隔离厂家 SDK 和上层算法。真机团队不需要修改 SLAM、Nav2 或
OpenCV 源码，只需完成以下合同：

1. 传感器/状态估计提供 `/scan`、`/odom`、`odom -> base_link`、传感器 TF，以及 Image、
   PointCloud2；非默认名称通过 `sensor_profile` 或 launch remap 对齐。
2. 真机底盘或运动控制器订阅最终 `/cmd_vel`。速度含义遵循 ROS REP-103：线速度 m/s、
   角速度 rad/s，`base_link` 的 `+x` 向前、`+y` 向左、`+z` 向上。
3. 运动/越障团队实现 `/traverse_obstacle` Action 服务端，并用
   `/terrain/navigation_safety` 复核同一时间戳的模式、限速、有效性和障碍几何；两者都
   是输入合同，不得在未完成动作仲裁和硬件安全的情况下直接转成关节命令。服务端只有
   在真实姿态/接触闭环确认稳定落地后才能返回 `success=true`。
4. 真机自身发布 URDF/TF 时使用 `robot_model:=false`，避免两个
   `robot_state_publisher` 同时发布传感器固定 TF。
5. 全栈启动后执行 `./scripts/check_integration.sh`；若相机/点云名称不同，可将实际话题
   作为两个参数传入。所有项通过后才算完成上层—真机最小对接。

```bash
ros2 launch slam slam.launch.py robot_model:=false sensor_profile:=generic
./scripts/check_integration.sh /camera/image_raw /camera/depth/points
```

接口的消息类型、字段、TF、超时与设备替换规则以 `connect.txt` 为准。未来可以在独立包中
新增厂家 SDK、状态估计和运动控制，但不应反向让核心感知算法依赖厂家类型。

## 4. 默认与可替换传感器接口

### 4.1 真机传感器选型与预留安装位置

当前 URDF 只是约 520 × 240 × 120 mm 的通用机身，`base_link` 名义离地约 440 mm。
在整机尺寸尚未冻结时，可先按下表预留长孔、线束和保护罩；坐标采用 ROS 约定：
`base_link` 的 `+x` 向前、`+y` 向左、`+z` 向上。数值是初始范围，不是最终标定值。

| 传感器 | 建议类型 | 大概位置（相对 `base_link`） | 安装姿态与用途 |
|---|---|---|---|
| 主雷达 | 360° 2D ToF 激光雷达，10 Hz 以上，室内有效距离 12 m 以上，直接发布 `LaserScan` | 机身顶部中心或略靠前：`x=0～+0.08 m`、`y≈0`、`z=+0.10～+0.15 m` | 扫描面水平，正方向对齐 `+x`；用于 SLAM、Nav2 和碰撞保护 |
| 主相机 | 主动双目 RGB-D，深度端优先全局快门，水平视场约 80°～95°，近端深度不大于 0.3 m，USB 3 | 前脸中央：`x=+0.28～+0.32 m`、`y≈0`、`z=0～+0.06 m` | 光轴朝前并向下俯 `10°～15°`；用于 OpenCV、台阶/坡面高度和局部点云 |
| 可选 3D 雷达 | 小型多线 3D ToF 雷达，输出 `PointCloud2` | 顶部中心，尽量接近主雷达位置并高于遮挡物 | 仅在 RGB-D 受强光或需要更远三维感知时增加；初版不是必需 |

除这两类传感器外，机身 IMU、关节编码器和足端接触仍要参与状态估计并生成可靠 `/odom`；
雷达不能替代里程计。若暂时只装普通 RGB 相机，OpenCV 仍可提示障碍，但无法提供真实高度
和距离，现有安全逻辑不会允许它单独触发 `STEP/CLIMB`。

这里的绝对离地高度约为：2D 雷达扫描面 0.54～0.59 m，相机光心 0.44～0.50 m。
若真机站立高度变化较大，以“雷达扫描平面不被机身和四腿遮挡、相机能同时看到
约 0.3～2.0 m 前方地面及障碍上沿”为准。安装时还应满足：

1. 两个传感器都固定在刚性机架上，不装在会摆动的外壳上；可隔离高频振动，但支架不能晃动。
2. 雷达上方和周围保留完整视野，不让提手、天线、线束穿过扫描平面；保护罩必须使用
   厂商允许的透光材料并避开出光窗口。
3. 相机放在左右中心线，避免腿在正常步态中反复进入画面；镜头前留出散热、插头和拆装空间。
4. USB 3 相机使用短线、锁紧或应力释放；雷达和相机电源与电机功率线分开走线并可靠接地。
5. 机械孔位至少提供前后、上下各约 20 mm 调整量；真机完成后再测量六自由度外参写入 URDF，
   不能把上述范围直接当成标定结果。
6. 使用标定板校准相机内参与畸变，再测量 `base_link -> camera_link` 和
   `base_link -> lidar_link`；最后在 RViz 中以墙面、地面重合情况复核，并录制站立、行走、
   蹲伏和转弯 rosbag。

仓库通用 URDF 已在范围内采用一个便于 RViz 调试的名义值：`lidar_link=(0.04, 0, 0.12) m`，
`camera_link=(0.29, 0, 0.03) m` 且向下俯 12°。真机完成后必须用实测值替换。

当前方案优先推荐“一个 2D 雷达 + 一个主动双目 RGB-D”，无需一开始购买 3D 雷达。RGB-D
可参考 Orbbec Gemini 2 一类设备：官方参数为主动双目、近端约 0.15 m、理想范围约
0.2～5 m、USB 3 且深度在设备端处理，较适合近距离地形；若机器运动很快，应优先考虑
RGB 和深度均为全局快门的型号。OAK-D Pro 一类设备也能在设备端计算主动双目深度，
但其官方理想深度范围约从 0.8 m 起，购买前需确认近距离台阶是否满足要求。2D 雷达可从
RPLIDAR S2、YDLIDAR/LDLiDAR 或 Hokuyo 的 ROS 2 兼容型号中选择；重点不是品牌，而是稳定
`LaserScan`、有效时间戳、供电、重量、抗环境光和驱动对 Jazzy/aarch64 的支持。
若计划在阳光直射环境运行，必须先实测主动红外深度有效率；不满足时再考虑室外双目或
3D 激光雷达，不能只依据室内标称距离采购。

参考资料：[Orbbec Gemini 2 官方规格](https://store.orbbec.com/products/gemini-2)、
[Luxonis OAK-D Pro 官方规格](https://docs.luxonis.com/hardware/products/OAK-D%20Pro)、
[SLAMTEC RPLIDAR S2 数据表](https://wiki.slamtec.com/download/attachments/83066883/SLAMTEC_rplidar_datasheet_S2_v2.0_en.pdf?api=v2)。

### 4.2 默认通信接口

算法内部始终保持 ROS 2 标准合同，不写死厂商品牌：

| 数据 | 内部默认 | 消息类型 | 一键入口覆盖参数 |
|---|---|---|---|
| 2D 激光 | `/scan` | `sensor_msgs/msg/LaserScan` | `scan_topic` |
| 里程计 | `/odom` | `nav_msgs/msg/Odometry` | `odom_topic` |
| RGB | 自动选择 | `sensor_msgs/msg/Image` | `camera_topic` |
| 深度/3D 点云 | 自动选择 | `sensor_msgs/msg/PointCloud2` | `point_cloud_topic` |

`slam.launch.py` 是唯一推荐的一键入口，内部直接读取 profile，并将雷达/里程计 remap
作用到 SLAM、Nav2、Collision Monitor、就绪监视器和 RViz；图像/点云参数同时传给
OpenCV 与地形节点。整个过程不复制高带宽数据，也不用修改算法源码。预置 profile：

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
默认 5 Hz、最大 576 像素宽；点云默认 5 Hz，TF 前最多 40000 点、几何分析最多 12000 点，
适合 RK3588 起步调试。关闭标注图发布还可省去一次图像复制与编码。
Xbox 输入使用标准 `ros-jazzy-joy`，不依赖厂家手柄 SDK。
独立仿真还需 Gazebo Harmonic / Gazebo Sim 8 对应的 `ros-jazzy-ros-gz`；`rosdep` 会按
`quadruped_gazebo/package.xml` 自动补齐桥接组件。

运行单元测试：

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

2026-08-13 的 Gazebo 全栈长测执行 5 轮“双向整圈旋转—前进—倒退”，约 3 分 20 秒内
地图更新 787 次、`/scan` 2996 帧、`/odom` 4941 帧，1962 个导航健康样本全部为 true；
最大闭环位置误差 0.0103 m、偏航误差 0.0152 rad。另一次联调已通过三点
`NavigateThroughPoses` 和规则 1 m 柱间窄通道，高墙占用体内目标按预期失败并走失败/恢复链；
该轮 700 个导航健康样本全部为 true。核心算法进程峰值约 861 MiB、瞬时约 1.55 个 CPU
核；这是当前 x86 电脑的近似 `/proc` 采样，不是 RK3588
温升、功耗或真机精度结论。

同日新增越障引导时序回归：两轮完整联合测试中 `/traversal/guidance` 收到 665 帧、最大
间隔 0.753 s，错误 READY 合同为 0；针对阈值边界的复测实际进入 READY 6 帧，
`ALIGN ↔ READY` 小于 0.35 s 的快速往返为 0。该轮最大位置闭环误差 0.0047 m，说明连续帧
确认和退出迟滞没有阻塞正常交接。入口 Pose 仅在存在有效越障目标时发布，避免消费者误用
无障碍状态下的零位姿。

OpenCV 合成矩阵已覆盖正常光、约 62% 暗光、局部阴影、全白过曝和 31 像素运动模糊：
暗光/阴影保留有效杆体候选，过曝帧被质量门拒绝，模糊帧的质量与置信度均下降。点云含噪
阈值扫测覆盖约 0.08 m 低台阶、0.09 m 坑、10°/14° 坡面和约 0.30 m 限高杆；这些是
确定性软件回归，换镜头、曝光、雷达和安装角度后必须用真机 rosbag 重做统计。

可复现长测（会让 Gazebo 测试狗运动，真机不得运行）：

```bash
ros2 run quadruped_tools stack_regression --allow-motion --cycles 5 \
  --report reports/stack_regression.json
```

## 6. 启动方式

本机已经在 `~/.bashrc` 中自动加载 ROS 2 Jazzy 和 `~/wakula/install/setup.bash`。修改配置
后让当前终端立即生效，或在其他尚未配置的终端手动执行：

```bash
source ~/.bashrc
# 等价的手动方式：
source /opt/ros/jazzy/setup.bash
source ~/wakula/install/setup.bash
```

若出现 `Package 'slam' not found`，说明当前终端尚未加载工作空间；执行上面的命令，
再用 `ros2 pkg prefix slam` 确认输出指向 `~/wakula/install/slam`。

只查看模型：

```bash
ros2 launch quadruped_description display.launch.py
```

完整启动 SLAM + Nav2 + 感知 + 地形速度门：

```bash
ros2 launch slam slam.launch.py
```

这条命令会启动占位模型、SLAM Toolbox、Nav2、OpenCV、点云地形分析、保守决策、
速度门和 RViz，但不会自动启动雷达/相机厂商驱动，也不会实现真实机器狗步态。
该命令不会创建自主导航节点，也不会自行下发探索目标。

常用参数都集中在同一入口：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `sensor_profile` | `ros_default` | 选择常见雷达/相机话题组合 |
| `scan_topic`、`odom_topic` | 空 | 非空时覆盖 profile 的雷达/里程计来源 |
| `camera_topic`、`point_cloud_topic` | 空 | 非空时覆盖 Image/PointCloud2 来源 |
| `slam_enabled`、`nav2_enabled` | `true` | 分别启停 SLAM Toolbox 和 Nav2 |
| `nav2_autostart` | `true` | 数据与 TF 就绪后是否自动激活 Nav2 |
| `vision` | `true` | 是否启动 OpenCV 障碍识别 |
| `robot_model` | `auto` | 自动在 Gazebo 关闭占位 TF、真机开启；也可显式覆盖 |
| `rviz` | `true` | 是否启动 RViz |
| `use_sim_time` | `auto` | 重试检测 `/clock`；也可显式设为 `true/false` |
| `*_params_file` | 项目默认 YAML | 覆盖 SLAM、Nav2、视觉、地形和决策参数文件 |

查看全部参数：

```bash
ros2 launch slam slam.launch.py --show-args
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
ros2 launch slam slam.launch.py sensor_profile:=realsense_d400
```

任意未知设备无需新增代码，直接覆盖实际话题：

```bash
ros2 launch slam slam.launch.py \
  scan_topic:=/front_lidar/scan \
  odom_topic:=/robot/odometry \
  camera_topic:=/rgb/image_raw \
  point_cloud_topic:=/depth/points
```

只启动占位模型、感知、决策和速度门，不启用 SLAM/Nav2：

```bash
ros2 launch quadruped_bringup bringup.launch.py
```

Nav2 节点启动后先保持未激活。就绪监视器收到 `/scan`、`/odom`，确认时间戳、frame、
扫描角度/样本数及里程计数值有效，再确认 `map -> base_link` TF 可用后才会自动激活；因此没有连接传感器时可
安全打开和关闭调试环境，不会在等待 TF 的生命周期切换中崩溃。地形节点仍会等待相机
外参，这是正常安全行为。若只检查参数、不希望自动激活，可使用：

```bash
ros2 launch slam slam.launch.py rviz:=false nav2_autostart:=false
```

### 6.1 自主探索与逐障碍越障编排

正常联调只运行下面三个命令，每个终端一个命令，职责互不包含：

```bash
ros2 launch quadruped_gazebo robocon_field.launch.py
ros2 launch slam slam.launch.py
ros2 launch slam autonomous_navigation.launch.py
```

第一条只提供独立 Gazebo 场地、测试载体和标准传感器/运动接口；第二条只运行核心
SLAM、Nav2、OpenCV、点云和 RViz，且默认没有自主任务；第三条才启动自主探索与越障编排。
真机联调时用真实驱动替换第一条，后两条保持不变。

启动该 launch 就立即执行，回到该终端按 `Ctrl-C` 就停止并取消任务；核心 SLAM、Nav2、
OpenCV、RViz 和地图继续运行。退出时先通过 `/navigation/autonomy_stop` 锁住自动导航速度，
再取消 Nav2/越障 Action；没有人工输入时机械狗立即停车，键盘或手柄持续发布时仍可人工
接管。它不 include `slam.launch.py`，也不读取 Gazebo world。真机最终速度仲裁器必须遵守
同一优先级：有效人工输入 > 自动导航锁 > 自动导航输入 > 零速度。

运行逻辑为：选择未知地图前沿 → Nav2 探索 → 连续确认障碍 → 冻结障碍位置并导航至入口 →
调用 `/traverse_obstacle` → 成功后登记该障碍并继续下一前沿。前沿目标来自 `/map`，代码不读
比赛 world 坐标；正式坐标改变不需要修改任务算法。真机必须由运动控制团队实现
`quadruped_interfaces/action/TraverseObstacle` 服务端。第三个命令检测到 `/clock` 时会自动
启动通用测试狗的平面越障 Action 替身，使三命令仿真能够完成“入口—越障—继续探索”；
真机时间下不会启动该替身。

入口导航若在已确认比赛障碍的膨胀边界中止，任务层只在最新点云仍指向同一 `map`
位置且置信度、距离、横偏全部满足守卫时交给 Action，避免永久重试，也不会把普通规划
失败当成越障条件。Action 成功后继续选择下一前沿。全局代价地图采用 16 m × 8 m 滚动
窗口；任务层只容忍 0.30 m 的 SLAM 栅格发布滞后，真正离图时仍等待地图恢复。

Gazebo 场地与算法完全分开。仿真时场地只提供测试模型和传感器数据：

```bash
ros2 launch quadruped_gazebo robocon_field.launch.py
```

然后分别运行核心 `slam.launch.py` 和可选 `autonomous_navigation.launch.py`。Gazebo 入口
不加载 SLAM、Nav2、OpenCV、自主任务或越障 Action。仿真 Action 替身由第三个命令按
`/clock` 自动选择，仍不属于 Gazebo 场地入口，也不读取 world 坐标；它只验证流程，不能
代表真实四足越障能力。

启动 `slam.launch.py` 后先看终端摘要。仿真联调必须显示
`simulation_detected=true, use_sim_time=true, robot_model=false`；入口会对 `/clock` 做多次
DDS 发现重试，避免 Gazebo 已运行却因首次查询漏报而错误使用系统时间。若摘要不是这三个值，
不要开启自动任务，可直接使用等价的显式仿真入口 `ros2 launch slam slam_sim.launch.py`。

### 6.2 比赛障碍参考场地（独立启动，不属于 slam.launch.py）

规则 V1.0 已公布的 14 m × 6 m 场地、8 类障碍尺寸和规定颜色位于
`src/quadruped_gazebo/worlds/robocon_obstacle_field.sdf`。规则明确说明障碍排列和安装位置
赛前另行公布，因此当前全局坐标只是图 1 的易修改参考布局；八个坐标集中在 world 顶部的
`REFERENCE LAYOUT` 框架区，取得正式坐标后只改这八个 `layout_*` frame，不修改 SLAM、
Nav2 或 OpenCV。

| 障碍 | world 中锁定的规则数据 |
|---|---|
| 直角绕杆 | 相邻杆 1.00 m；杆高 0.55 m，满足“不低于 0.50 m”；必达区距杆 0.40 m、直径约 0.35 m；橘色 |
| 砂砾碎木坑 | L 形外包络 2.00 m × 2.00 m、臂宽 1.00 m、深约 0.10 m、护栏/木槛高 0.15 m；护栏橘色 |
| 限高杆 | 蓝白 PVC 横杆长 1.00 m，横杆底部离地 0.30 m |
| 斜坡 | 长 3.00 m、总宽 2.00 m、坡角 10°；橘色 |
| 木桥 A | 桥段长 1.50 m、三条桥条各宽 0.10 m、平台高 0.20 m；橘色 |
| 木桥 B | 跨段 2.90 m、六块踏板各宽 0.15 m、净间隔 0.40 m、平台高 0.20 m；橘色 |
| T 字形台阶 | 总长 2.80 m、总宽 1.90 m、高 0.40 m、中心平台 1.00 m × 1.00 m；橘色 |
| 高墙 | 长 1.00 m、高 0.30 m、厚 0.05 m；橘色基体、顶部钢色 |

木桥 A、B 共保留两条规则规定的 14° 上下平台小坡。橘色严格使用 RGB(223,117,0)，
蓝色使用 RGB(31,65,159)，场地黄色使用 RGB(255,255,0)。

只启动 Gazebo 场地、仿真传感器载体和 ROS 桥：

```bash
ros2 launch quadruped_gazebo robocon_field.launch.py
```

无显示器运行，或只看场地而不生成测试载体：

```bash
ros2 launch quadruped_gazebo robocon_field.launch.py gui:=false
ros2 launch quadruped_gazebo robocon_field.launch.py spawn_test_robot:=false
```

该 launch 不 include 任何算法。需要联合测试时另开终端使用仿真算法入口：

```bash
ros2 launch slam slam_sim.launch.py
```

`slam_sim.launch.py` 也不会启动 Gazebo；它只包装核心 `slam.launch.py`，强制
`use_sim_time=true`、`robot_model=false`，防止漏写参数后出现传感器时间基准不一致或测试狗
TF 与占位 URDF 重复。真机仍使用 `slam.launch.py`，两套入口共享同一套算法和参数。
即使误用普通 `slam.launch.py`，当前入口也会直接探测实时 `/clock` 发布者，并在启动首行
打印 `simulation_detected=true, use_sim_time=true, robot_model=false`，不再静默使用错误时间。

仿真载体现为 `models/generic_quadruped/model.sdf` 中的蓝色通用机械狗外形，只用于验证
`/scan`、`/odom`、`/imu/data`、`/cmd_vel`、RGB 图像和深度点云链路。它的腿是固定外观，
机身是保持水平、无动力学碰撞的平面“幽灵载体”，因此不会再出现旧轮式测试底盘俯仰后
让二维雷达扫到地面、在地图中生成放射状假墙的问题。它不能用于评价站立、步态、接触或
真实越障能力；直接向 `/cmd_vel` 发命令也可以穿过障碍，真实碰撞必须等待正式动力学模型。
规则没有给出的杆径、材料随机形态和地面启动区尺寸仅作可复现近似；正式坐标未发布前
不得把当前 pose 当作官方坐标。

正式机械狗 SDF 到位后可在场地 launch 上一次替换，不需要改 SLAM、Nav2 或 OpenCV：

```bash
ros2 launch quadruped_gazebo robocon_field.launch.py \
  robot_sdf:=/绝对路径/real_quadruped/model.sdf \
  robot_name:=real_quadruped \
  publish_test_sensor_tf:=false
```

新模型需继续发布标准 `/scan`、`/odom`、`/tf`、相机/点云话题；若 frame 或话题不同，优先在
驱动/profile/remap 层对齐。`publish_test_sensor_tf:=false` 可避免真实模型外参与测试外参重复。

完整 SLAM/RViz 测试需保持两个终端，不要重复启动 Gazebo，否则多个 `/clock` 会导致 TF
时间回跳：

```bash
# 终端 1：场地和仿真传感器
ros2 launch quadruped_gazebo robocon_field.launch.py

# 终端 2：既有 SLAM + Nav2 + OpenCV + RViz（固定仿真时间和 TF 所有权）
ros2 launch slam slam_sim.launch.py
```

通用机械狗测试替身已直接对齐算法默认接口：720 点 360° `/scan`（15 Hz、12 m）、`/odom` 和
`odom -> base_link`（30 Hz）、424×240 RGB-D（15 Hz）、`/imu/data`（100 Hz）。424×240 是
独立仿真的轻量联调档，算法仍按归一化 ROI 工作，替换真机常见 640×480/640×360 输入无需改源码。
参考 world 的物理时钟为 100 Hz，足够支撑测试 IMU，并避免 Gazebo GUI、RViz、OpenCV 与
点云同机运行时因高频 `/clock` 挤压 `/scan`、`/odom` 和地图更新。时钟、导航关键数据与
辅助传感器分别桥接，未被算法使用的 `/scan/points` 不再重复进入 ROS。
360° 雷达位于机身中心且保持水平，并通过 Gazebo 可见掩码忽略测试狗自身外观，避免自遮挡写入地图。RGB 图像
使用 `camera_optical_frame`；Gazebo 当前生成的 PointCloudPacked 数值轴实际采用
`camera_link` 约定，因此仿真专用 bridge 会覆写点云 frame，避免算法把点云重复旋转。
真机仍应由驱动发布真实 frame 和 TF，不需要这一仿真修正。RViz 中应看到 `/map`、
LaserScan、机器人 TF、Nav2 代价地图，以及 `Camera Detection` 面板中的识别标注画面。
默认 RViz 已关闭容易遮挡地图的 TF、网格和实时 LaserScan；需要查原始雷达时再手动勾选
LaserScan。地图中白色是已观测自由区、黑色是占用区、灰色是未知区。开放场地初始地图
会从出生点向可见障碍展开，白色射线边缘是探索范围而不是墙；应低速覆盖通道、在转角
旋转观测并完成回环，再用黑色墙线是否重合评价地图质量。实测闭环已消除旧模型的黑色
放射假墙；地图以 0.25 s（4 Hz）周期发布，扫描匹配最多约 10 Hz，移动 8 cm 或旋转
0.08 rad 即可加入新关键帧，改善倒退和原地旋转时的跟随。RViz 顶视图跟随 `base_link`，但全局固定坐标仍是
`map`。地图整体相对屏幕旋转只代表 `map` 坐标方向，不是几何错误。

算法运行时不要让键盘和 Collision Monitor 同时直接发布 `/cmd_vel`。Gazebo 场地 launch
现内置仿真专用速度仲裁器：算法保持标准 `/cmd_vel`，键盘走 `/cmd_vel_teleop`，Xbox 走
`/cmd_vel_joy`；有效人工输入拥有最高优先级，唯一输出 `/cmd_vel_gazebo` 再送入模型。
自主任务退出只锁自动导航分支，键盘和手柄仍能人工接管。启动键盘请另开终端运行：

```bash
cd ~/wakula
./scripts/keyboard_teleop.sh
```

继续使用 `i/j/k/l`；`j`、`l` 为原地左右旋转。该仲裁只属于 Gazebo，不改变未来真机的
`/cmd_vel` 合同，也不会被加入算法 launch。

联合测试时不要关闭“终端 1”或 Gazebo 窗口：Gazebo 服务端退出后，窗口和 RViz 仍可能
保留最后一帧，但 `/scan`、`/odom`、点云已经全部停止，此时“感知数据无效”是安全降级。
可用 `ros2 topic hz /scan`、`ros2 topic hz /odom`、
`ros2 topic hz /camera/depth/points` 分别确认数据仍在流动；障碍名称用
`ros2 topic echo /perception/front_obstacle_name` 查看。

从 Snap 版 VS Code 集成终端启动时，本项目的 Gazebo/RViz launch 会清理其注入的
`GTK_PATH=/snap/code/...`，避免加载 core20 `libpthread` 后出现 `GLIBC_PRIVATE` 错误；终端中
偶尔出现 `canberra-gtk-module` 提示只影响提示音模块，不影响仿真或算法。
Ubuntu 的崩溃报告窗口可能延迟显示上一次关闭留下的报告；先核对窗口内 `Date` 与最新
`~/.ros/log`。若最新日志写明 `rviz2 process has finished cleanly`，它不是当前算法故障。
诊断 TF 请运行 `./scripts/diagnose.sh`；不要将持续输出的 `tf2_echo` 直接连接到 `head`，
否则读取端提前关闭可能让 ROS 2 Jazzy 报 `BrokenPipeError`，但这不代表算法节点崩溃。

### 6.3 Xbox 手柄节点（独立启动，不属于 slam.launch.py）

先连接手柄并确认系统识别：

```bash
ros2 run joy joy_enumerate_devices
```

使用独立 launch 同时启动手柄驱动与 Wakula 适配器：

```bash
source ~/wakula/install/setup.bash
ros2 launch quadruped_teleop xbox_teleop.launch.py
```

多手柄时可以指定 `device_id:=1`；用 `--show-args` 查看设备编号、话题和参数文件覆盖项。
该 launch 只启动 `joy_node` 与 `xbox_teleop`，不会启动 SLAM、Nav2 或真实运动控制器。

| 控件 | 当前作用 |
|---|---|
| 左摇杆上下/左右 | 前后移动/横移 |
| 右摇杆左右 | 左右偏航转向 |
| LB | 摇杆回中时按下解锁，持续按住使能；松开立即归零 |
| A / X / Y | 低速档 / 正常档 / 快速档 |
| B | 锁存软件急停 |
| Start | 松开 LB 且摇杆回中时解除软件急停 |
| RB、Back、Guide、左右摇杆按下 | 预留，当前不产生动作 |
| LT、RT、十字键、右摇杆上下 | 预留，当前不产生动作 |

若带着非零摇杆按下 LB，节点会拒绝解锁；即使随后回中也必须松开并重新按下 LB，避免
手柄放置姿态造成突然起步。若 `/joy` 断流，重连后同样必须先松开再重新按下 LB，避免
沿用断流前的使能状态。调试时查看 `/cmd_vel_joy`、`/teleop/active`、
`/teleop/emergency_stop` 和 `/teleop/speed_mode`。若某个摇杆方向与表格相反，只需在
`xbox.yaml` 将对应 `*_direction` 改成 `-1.0`。默认不直接发布
`/cmd_vel`，防止与 Nav2 同时控制；真机应增加 `twist_mux`，在手柄和 Nav2 间仲裁后再进入
Collision Monitor。该节点只输出机身速度，仍需运动控制团队把 Twist 转换为四足步态。

## 7. OpenCV 障碍识别

节点同时使用两类轻量特征：

- HSV 橙色/蓝色区域：对比赛场地中颜色明显的杆和横杆优先识别。
- 灰度 Canny 轮廓：用于补充双立柱和大矩形结构；不带颜色的单条水平边缘不会独立声称是限高杆。

每帧先在未经增强的原图上评估曝光、动态范围和清晰度，避免 CLAHE 把暗光噪声伪装成
有效纹理；随后合并原图与 CLAHE 图的 HSV 掩膜，在保留正常色相的同时补回阴影中的橙/蓝
区域。接近纯白且低饱和的高光区域会从 Canny 边缘中膨胀剔除，降低场馆灯光、金属反射和
局部过曝形成假障碍的概率。蓝色横杆还需满足最小长宽比、最大宽度占比和最大高度占比，
因此远处障碍行、地平线或贴满画面的地面边界不会仅凭 Canny 被当作限高杆。严重欠曝、过曝或失焦图像在进入历史窗口前被拒绝；最小轮廓
面积同时采用像素下限和图像面积比例，避免切换分辨率后检测尺度突变。
双立柱必须同时满足高度、垂直重叠、间距、宽度和填充率一致性；颜色候选与边缘支持共同
计算置信度。最近 5 帧不仅要求至少 3 帧且达到 60% 同类投票，还要求位置、尺寸和目标框
IoU 连续，因此反光、画面边缘、无关竖条和跨帧跳变不容易形成稳定证据。输出接口保持不变：
相机帧间隔超过 `history_reset_timeout` 或 ROS 时钟回拨时会清空历史，恢复后必须重新积累
完整确认帧数，防止断流前后的旧、新画面共同形成一次误确认。

```text
/vision/obstacle_evidence  std_msgs/Float32MultiArray
/vision/obstacle_hint      std_msgs/String
/vision/color_features     std_msgs/Float32MultiArray  # 标定/兼容接口
/vision/annotated_image    sensor_msgs/Image           # RViz 默认显示的原图标注
/vision/debug_mask         sensor_msgs/Image           # 默认关闭
```

`/vision/obstacle_evidence` 是越障决策使用的原子结果：

```text
[type_code, confidence, center_x, center_y, width, height]

type_code: 0=none, 1=poles, 2=height_bar, 3=wall, 4=colored_obstacle
其余字段均归一化到 0.0～1.0
```

只有证据置信度达到 `vision_min_confidence`、目标位于行进方向中央且结果未超时，才会
把正常 `WALK` 的速度上限降到 `vision_speed_scale`，并设置
`visual_assist_active=true`。视觉不会覆盖已经由点云给出的 `STEP`、`CLIMB` 或 `STOP`。

现场必须按真实相机和光照标定 `vision.yaml` 中的 HSV、ROI、Canny、图像质量、高光、轮廓和多帧参数。
建议依次调整 ROI → HSV → `min_image_quality` → `min_area_px`/`min_area_ratio` → Canny →
`temporal_match_ratio`/`min_temporal_iou`，
避免同时修改全部参数而无法定位误差来源。CLAHE、双光照掩膜、高光抑制和自适应 Canny
均可单独关闭以做 A/B 对照。
可临时开启 `publish_debug_mask`，在 `/vision/debug_mask` 检查分割与边缘效果；标定完成后
关闭，以减少图像复制。

RViz 的 `Camera Detection` 面板默认订阅 `/vision/annotated_image`：青框是实际检测 ROI，
黄框是当前帧候选，绿框是多帧确认后的稳定障碍；顶部 `FRONT` 显示视觉类别，
`IMAGE QUALITY` 显示输入质量。安全判断仍以点云融合结果为准，因此终端中的
`[正前方障碍]` 可能比单帧视觉框更保守。融合结果还会发布中文速查话题：

当 OpenCV 发现目标而点云尚未确认尺度时，终端会明确显示“视觉疑似××（点云未确认，
已限速）”；一旦点云确认台阶、墙等几何类别，中文名称以点云融合结果为准。超大、贴近
整幅画面的纯边缘轮廓会被视为地面/天空边界或近距遮挡，不单独触发视觉限速。

比赛专名采用可移植的传感器证据，不读取 Gazebo 模型名或坐标：约 10° 的低横滚坡面显示
“主斜坡”，约 14° 显示“木桥引坡”；宽且约 0.40 m 高的阶梯显示“T 字形台阶”。仅凭
局部单帧无法可靠区分木桥 A/B 或普通踏板时，会如实显示“A/B 待结构确认”或“台阶或木桥
踏板（待结构确认）”。木桥 B 的宽平桥板与 0.40 m 间隙组合、砂砾坑的低护栏/粗糙填料、
限高杆的约 0.32 m 支柱会给出对应的接近阶段名称；证据不足时保留“待确认”，不猜测坐标。

```bash
ros2 topic echo /perception/front_obstacle_name
```

## 8. 当前地形决策边界

| 模式/类别 | 当前处理 | 是否执行腿部动作 |
|---|---|---|
| `WALK` | Nav2 速度上限为 1；视觉证据可将上限降至 0.35 | 否 |
| `POLE` | 速度上限为 0.35，由 Nav2 代价地图规划绕行 | 否 |
| `STEP` / `PIT` / `WALL` / `BAR` | 远处低速接近入口，随后对正；进入 0.90 m 交接区发布 `READY` 并停车 | 否 |
| 可量测坡面 | 发布坡面越障候选及入口引导；交接区停车 | 否 |
| 数据断流、TF 失败或字段非法 | 发布 `STOP` 和零速度上限 | 否 |

`/traversal/guidance`、`/traversal/phase` 和 `/traversal/approach_pose` 仍只表达
`APPROACH/ALIGN/READY` 与相对入口建议；`autonomous_mission` 消费这些消息并负责
“选择前沿→发送入口目标→READY 交接→等待 Action 结果→继续探索”。仓库已定义
`TraverseObstacle` Action 合同，但没有 SDK 网关、关节轨迹或真实越障服务端；仿真服务端
只用于验证编排，不等价于机器狗跨越能力。

## 9. 点云地形与 Nav2 融合

`terrain_analyzer` 只保留最新一帧，将点云按消息时间戳转换到 `base_link`，裁剪机器人
正前方 ROI 并发布：

```text
/terrain/features              std_msgs/Float32MultiArray
/terrain/features_stamped      quadruped_interfaces/TerrainFeatures
/perception/obstacle_points    sensor_msgs/PointCloud2
/perception/fused_obstacle     quadruped_interfaces/FusedObstacle
/traversal/guidance            quadruped_interfaces/TraversalGuidance
/traversal/approach_pose       geometry_msgs/PoseStamped
/diagnostics                   diagnostic_msgs/DiagnosticArray
```

高分辨率 RGB-D 原始云会先用 `transform_max_points` 做覆盖全幅的确定性等间隔采样，再
执行 TF 和前向 ROI 分析；这不会改变话题合同，可显著降低 Gazebo 全栈或 RK3588 上的
内存与矩阵运算压力。在线默认以 5 Hz 处理最新帧、前视 2.5 m；设采样上限为 `0` 可在
离线标定时关闭限制做精度对照。

`/terrain/features` 字段：

```text
[ground_z, high_z, obstacle_height, valid_points, slope_pitch_tan, roughness,
 frontal_obstacle_height, lookahead, traversability, pit_depth, slope_roll,
obstacle_type, confidence, width, clearance_height]
```

启用 OpenCV 时，决策层优先消费 `/perception/fused_obstacle`：它在一个带 Header 的消息中
同时携带几何/视觉确认位、点数、粗糙度、坡度和时间差，避免不同帧字段被拼成一次决策。
同步器会在有界小队列内寻找全局时间差最小的一对消息，能处理常见的回调乱序；零时间戳、
重复旧帧和时间差超过 `0.10 s` 的观测不会融合。融合层还会二次校验类别、NaN/Inf、
归一化视觉框、前向通道相交关系和连续量范围；决策层拒绝超龄/未来时间戳。几何未确认、置信度不足或点数
不足时保持停车。若相机断流，融合器等待 `0.25 s` 同步窗口后继续发布
`vision_confirmed=false` 的纯点云几何，避免辅助相机成为安全链单点故障。关闭
OpenCV 后自动回到 `/terrain/features` 兼容路径，便于只有 3D 雷达的真机继续使用。

判定默认值：高度 `0.07 m` 起分类为 `STEP`，`0.18 m` 起分类为 `CLIMB`，`0.32 m` 起
标记为必须重规划；当前三种情况都会停车。阈值必须依据机器狗的实际腿长、质心、步态
能力和相机安装误差重新标定。

兼容字段仍使用纵向低分位地面包络；强类型输出另将点云压成 XY 高度栅格，从占多数的
高度层迭代 MAD 剔除离群格并拟合 `z=ax+by+c` 主地面，再计算俯仰/横滚坡度、坑深、墙面
垂直跨度、横杆净空和立柱宽度。高处/低处异常不仅必须形成可配置的八邻域连通区域，还要
达到最小原始回波数，分散或相邻的少量飞点都不会组成障碍。横杆净空只用高于地面的物体
回波计算，避免同一切片的地面点把约 0.30 m 悬空杆误判成墙。坑洞必须看到真实低处回波，
单纯无点按未知处理，避免把盲区误判成坑。连通域严格复用建格时的向下取整规则，避免
栅格边界附近的细障碍格被四舍五入合并。
`frontal_obstacle_height`
只统计中央通道，`lookahead` 是最近成片障碍的实际 x 距离，不再是固定 ROI 长度。

算法先用拟合平面计算 ROI 中每点的相对地面高度，只将凸起降采样发布为
`/perception/obstacle_points`，因此 10°/14° 坡面不会随前向距离增加而被误标成墙；
低台阶即便位于机身 `base_link` 下方、绝对 z 为负，也不会被 Nav2 的高度过滤漏掉。
经真实低回波与连通域确认的坑洞会被投影为贴近局部地面的虚拟障碍点写入代价地图；
无回波盲区仍按未知处理，不能凭空制造坑洞。
Nav2 local costmap 以该 `PointCloud2` 进行 marking，2D 雷达继续负责 marking + clearing。点云层不主动
clearing，防止短暂深度空洞错误清除障碍；激光清障和滚动窗口会移除离开视野的旧区域。

### rosbag 离线标定与准确率报告

完整采集和回放建议直接使用：

```bash
./scripts/record_bag.sh                    # 默认写入 bags/时间戳目录
./scripts/replay_bag.sh bags/某次记录 0.5  # 以 /clock 回放，初始暂停
```

工具直接读取 `/vision/obstacle_evidence` 和 `/terrain/features`，不需要启动实时节点。先生成
稀疏标注模板，人工填写 `vision_label`（`none/poles/height_bar/wall/colored_obstacle`）和
`terrain_label`（`WALK/STEP/CLIMB/STOP`），再评估：

```bash
ros2 run quadruped_tools perception_bag_evaluator BAG目录 \
  --write-label-template labels.csv --sample-period 0.5

ros2 run quadruped_tools perception_bag_evaluator BAG目录 \
  --labels labels.csv --report perception_report.json \
  --suggestions calibration_suggestions.yaml
```

报告包含混淆矩阵、accuracy 及其 95% Wilson 区间、macro-F1、每类 precision/recall/F1、
匹配数量和时间对齐误差。默认至少需要每类链路 20 个已匹配样本；达到数量后按时间将最新
30% 留作验证集，只在较早 70% 上搜索 `vision_min_confidence` 与高度阈值，避免用同一批
数据调参又验收造成指标虚高。建议值写入独立 YAML，不会自动覆盖正式配置。

## 10. 速度与失效安全

速度链路固定为：

```text
/cmd_vel_nav -> /cmd_vel_smoothed -> /cmd_vel_terrain_safe -> /cmd_vel
```

- Nav2 controller 只发布 `/cmd_vel_nav`。
- Velocity Smoother 限制加速度并发布 `/cmd_vel_smoothed`。
- `navigation_speed_gate` 应用 `/terrain/speed_limit`，同时检查命令、评估和
  `/navigation/healthy` 心跳。
- Collision Monitor 读取 `/scan`，并作为 `/cmd_vel` 唯一发布者。

Jazzy 1.3.12 的 Collision Monitor 在全栈 Ctrl-C 时可能让进程信号清理与最后一个回调
并发，表现为 `get_subscription_count()` 处 SIGSEGV。项目的
`collision_monitor_supervisor` 不修改官方算法或任何 ROS 接口：运行时仍是原版
`/collision_monitor`；退出时先让上游停发，再结束独立会话里的无持久状态子进程，由
操作系统回收 DDS 和文件描述符，不再让该 Jazzy 版本进入有缺陷的 SIGINT/SIGTERM 清理
路径。不要绕过 `slam.launch.py` 单独启动系统可执行文件，否则不会获得此退出保护。

规划命令或地形决策心跳任意一项超时，速度门都会发布零速度。这只是导航软件层的失效
停车，不替代未来真机必须具备的硬件急停、驱动失能、姿态/关节保护和底层看门狗。

融合模式采用非对称防抖：紧急 STOP 立即生效；STEP/CLIMB 需要连续几帧几何证据；向
更安全等级恢复时要求更多连续安全帧。这样既不延迟紧急停车，也减少飞点和阈值抖动。
已确认的台阶、坑洞、墙和横杆在 0.90 m 以外保留 0.25 倍低速窗口，让 Nav2 到达入口
并对正；进入交接区立即归零并发布 READY。立柱属于绕杆导航物体，保持 0.35 倍速度由
代价地图避碰。坡面会形成越障引导候选，但真机没有运动控制器时仍只能在交接处停车。
视觉细分类也不能无条件覆盖点云：横杆必须同时具有米制离地净空，立柱必须满足点云窄宽度；
视觉与几何冲突时保留几何类别并降低置信度，等待后续同步帧确认。
高度、坡度、粗糙度、点数、消息采样时刻和超时等运行参数也在节点入口及纯决策函数处
进行合法性防御；NaN、Inf、旧帧、未来帧、退化里程计四元数、越量程雷达回波或乱序高度
阈值都不能被解释为可通行。

已加入规则针对性合成回归：橙色双杆、蓝色横杆、约 0.30 m 悬空限高杆、0.30 m 高墙、
真实低回波坑洞及 14° 坡面；同时覆盖黑场、白场、阴影、运动模糊、镜面高光、点云飞点、
相机/点云乱序和旧时间戳。合成测试只能防止代码回退，真机阶段仍必须按比赛场地录包验收。
新增回归还覆盖相机断流历史清除、纯点云降级、栅格边界细障碍及里程计跳变锁存恢复。

核心 Python、launch、参数 YAML、行为树、ROS 消息和运维脚本均已补充中文设计注释；
`/terrain/features` 的下标已集中为具名常量。维护时不要为逐行翻译代码而增加注释，应优先
记录数据流、坐标系、单位、算法假设、安全边界以及更换传感器后必须重新标定的内容。

## 11. Robocon 比赛逻辑

| 待实现内容 | 前置条件 |
|---|---|
| 210 秒计时、计分、障碍顺序和返回起点 | 正式规则与场地坐标冻结 |
| 障碍完成、失败、取消和有限重试 | 真机越障控制器能返回可信结果 |
| 足端接触限制和台阶计分 | 真实足端力/接触检测完成 |
| Nav2 与越障控制切换 | 基础步态、急停和全身控制通过台架验收 |

当前已完成不依赖场地坐标的自主前沿探索、入口接近、Action 交接、完成障碍去重和继续探索。
仓库仍不发布 `/competition/*` 话题，也没有正式顺序、计时、计分、返回起点或裁判状态机，
避免在缺少真机反馈时把通用探索流程误认为完整比赛能力。

## 12. 配置文件索引

| 配置 | 内容 |
|---|---|
| `quadruped_interfaces/msg/`、`action/` | 地形、视觉、融合消息与越障 Action 合同 |
| `quadruped_description/urdf/` | 未标定外形、关节和传感器占位坐标系 |
| `quadruped_gazebo/worlds/robocon_obstacle_field.sdf` | 规则障碍尺寸、颜色和集中式参考布局 |
| `quadruped_gazebo/launch/robocon_field.launch.py` | 独立 Gazebo/传感器桥入口，不加载算法 |
| `quadruped_gazebo/launch/sim_traversal_controller.launch.py` | 可选仿真 Action 替身；不启动场地或算法 |
| `quadruped_perception/config/vision.yaml` | HSV、Canny、多帧确认、图像资源限制 |
| `quadruped_perception/config/terrain.yaml` | 点云话题、ROI、采样和地形阈值 |
| `quadruped_planning/config/terrain_navigation.yaml` | 地形分类阈值、视觉辅助和速度门超时 |
| `quadruped_planning/config/autonomous_mission.yaml` | 前沿选择、入口锁定、Action 超时和完成去重参数 |
| `quadruped_tools/perception_bag_evaluator.py` | rosbag 标签、指标和参数搜索 |
| `.github/workflows/ros2-ci.yaml` | Ubuntu 24.04 + ROS 2 Jazzy 自动构建测试 |
| `slam/config/slam.yaml` | SLAM Toolbox |
| `slam/config/nav2.yaml` | Nav2、代价地图、速度平滑和碰撞监控 |
| `slam/behavior_trees/navigate_to_pose_wakula.xml` | 清图、等待、小退和小角度旋转恢复树 |
| `slam/config/sensor_profiles.yaml` | 常见雷达/相机话题 profile，可直接扩展 |
| `slam/launch/slam.launch.py` | 只启动 SLAM、Nav2、OpenCV、点云和 RViz，不创建自主任务 |
| `slam/launch/autonomous_navigation.launch.py` | 独立自主探索/越障编排；启动即运行，Ctrl-C 即停止 |

核心算法公共启动只有一份：`quadruped_bringup/launch/bringup.launch.py`；
`slam/launch/slam.launch.py` 在其上增加 SLAM、Nav2 与 RViz，避免重复维护节点。
`slam/launch/sensor_compat.launch.py` 仅作为旧命令兼容别名，新的启动命令统一使用
`slam.launch.py`；已删除重复的 `all_in_one.launch.py`。Gazebo 和 Xbox 分别保留独立 launch，
且不会被核心算法入口隐式启动。

## 13. 实机接入清单

1. 用实测值替换 URDF 尺寸、质量和惯性。
2. 新建厂商 SDK 适配或 `ros2_control` 硬件接口；当前仓库没有模拟实现可直接替换。
3. 标定关节零位、IMU、雷达、RGB/深度相机内外参和时间同步。
4. 检查 `/odom`、`odom -> base_link`、传感器 TF 的方向、频率和协方差。
5. 录制 rosbag，在离线数据上标定 HSV、点云 ROI、高度和坡度阈值。
6. 检查 local costmap 中激光与 `/perception/obstacle_points` 是否准确重合。
7. 空载验证断相机、断雷达、断里程计和决策超时；真机安全层完成后再验证急停与恢复。
8. 先低速单障碍测试，再接入真实跨越动作和足端接触反馈。

目前的 OpenCV 是带光照归一化、几何配对和时序一致性校验的可解释规则识别，适合起步、
比赛固定障碍和 RK3588 低负载运行，但准确率仍取决于视角、光照与标定。需要更强泛化时，
应先采集误检/漏检数据，再决定是否增加学习模型，而不是直接让视觉模型控制机器狗动作。

项目根目录维护四份互补文档：

- `README.md`：安装、启动、总体架构和使用入口。
- `instruction.txt`：各模块作用、算法原理和现场调试顺序。
- `connect.txt`：全部节点、话题、消息类型、字段、TF、QoS 和超时合同。
- `quickstart.txt`：跨电脑完整复现、启动命令、当前成果、常见问题和后续开发路线。

后续每次完成修改都同步检查这四份文档：接口变化更新 `connect.txt`，算法或职责变化
更新 `instruction.txt`，启动和使用方式变化更新 `README.md`，完成进度与后续计划变化
更新 `quickstart.txt`。验证通过后直接提交并推送 `main`，不使用强制推送。
