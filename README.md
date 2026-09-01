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
| `instruction.txt` | 各模块作用、算法原理、真机 rosbag 标定和调试步骤 | 修改 SLAM、Nav2、OpenCV 或越障算法 |
| `connect.txt` | 节点作用、关键输入输出、接口字段和真机通信约定 | 接入相机、雷达、真机或排查通信 |
| `AGENTS.md` | 仓库边界、参数归属、注释规范和验证命令 | 让另一台电脑的 Codex/代码助手继续开发 |

### 真机调参从哪里开始

所有真实生效的参数仍保存在原有 YAML 中，没有再复制一套容易失配的“调参参数”。以下
文件顶部均有统一的“现象 → 参数 → 调整方向 → 副作用”索引，接入真机时先读顶部注释，
再修改同一文件中的实际值：

| 现场问题 | 唯一参数入口 |
|---|---|
| 地图跟不上、重影、回环失败、SLAM CPU 高 | `src/slam/config/slam.yaml` |
| 擦碰、窄通道无路径、速度/振荡、代价地图异常 | `src/slam/config/nav2.yaml` |
| 相机话题、雷达话题或厂家命名不同 | `src/slam/config/sensor_profiles.yaml` |
| 光照、颜色、模糊、横杆/立柱视觉误检 | `src/quadruped_perception/config/vision.yaml` |
| 台阶、坡、坑、墙、横杆点云误检或 RK3588 延迟 | `src/quadruped_perception/config/terrain.yaml` |
| 停车距离、限速、对正和 READY 交接 | `src/quadruped_planning/config/terrain_navigation.yaml` |
| 5 秒卡死恢复、重复目标、成功判定和探索覆盖 | `src/quadruped_planning/config/autonomous_mission.yaml` |

调参前先修复时间戳、TF、外参和 `/odom`；这些基础数据错误不能通过放宽算法阈值解决。
每次只修改一个参数组，保存 YAML 版本、rosbag、设备/安装位姿、测试场景和指标。机身
`footprint/robot_radius`、两处 `inflation_radius`、入口停车距离、Action 交接距离是一组
联动参数，禁止只改某一个让机器人“先动起来”。

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
| 强化学习训练环境 | ⬜ 未实现，等待真机 | 在独立 NVIDIA GPU 工作站上基于 Isaac Lab（运行于 Isaac Sim）搭建并行训练环境，设计观测、动作、奖励、课程学习、终止条件和 Sim-to-Real 随机化 |
| 复杂地形运动控制 | ⬜ 未实现，等待真机 | 通过地形随机化和扰动训练，使强化学习策略适应平地、斜坡、台阶、坑洼、连续障碍及非结构化地形，实现速度跟踪、姿态稳定和足端协调 |
| 环境感知 | 🟡 软件雏形完成 | 真机标定雷达/相机，完成 RGB-深度同步、障碍跟踪、高程图、坡面和可落脚区域识别 |
| SLAM 与自主导航 | 🟡 软件雏形完成 | 使用真实 `/scan`、`/odom` 和 TF 调参，验证重定位、动态避障、狭窄通道及失效恢复 |
| 任务与比赛逻辑 | 🟡 通用自主任务雏形完成 | 已能维护已完成/待完成清单，优先回访已知未越障目标，并逐个完成识别、对正、入口和 Action 交接，最后导航到终点；仍需接入真实越障反馈、正式顺序/计时/裁判接口 |
| 整机安全 | ⬜ 仅有导航速度超时门 | 实现并验证硬件急停、驱动失能、过流/过温/欠压及真实姿态/关节保护 |
| 仿真与测试 | 🟡 已有独立 Gazebo 参考场地 | 已按官方 2026 年第二十五届 ROBOCON 仿生足式挑战赛 V2.0 复现已公布尺寸/颜色并提供传感器测试载体；正式坐标、真机动力学、Isaac、SIL/HIL 仍待后续完成 |
| 真机联调与工程化 | 🟡 CI 与 rosbag 评估工具已有 | 架空→保护绳→低速→单障碍→整场测试，完成部署服务、日志策略和维护流程 |

当前代码完成的是环境感知、SLAM/Nav2、传感器通用 profile、导航健康检查、保守地形
决策、速度超时门、未知地图前沿探索、Nav2 越障入口接近、`TraverseObstacle` Action 编排、
Xbox 手柄适配、独立比赛场地、强类型真机对接合同、rosbag 离线评估和全栈长时间回归工具：
9 个 ROS 2 包已完成全量构建；当前 `colcon test-result --all --verbose` 基线为
**550 tests、0 errors、0 failures、0 skipped**，并提供一键启动、独立停止、对接检查和 CI。URDF 只用于 RViz 外形与
传感器 TF 占位，
不能视为运动学或整机控制已完成。
详细清单与开发顺序见 `quickstart.txt`。

感知、规划及 SLAM/Nav2 健康节点会在创建通信实体之前校验原始 ROS 参数，包括话题格式、
ROI 次序、HSV 范围、点数/频率、同步窗口、高度阈值、APPROACH/HANDOFF 距离、任务投票窗口
以及速度门输入/输出回环。非法 YAML 会让对应节点启动失败，并在一条 `ValueError` 中列出同组
全部错误，不再把错误值静默修改后继续运行。真实 ROS 节点回归还覆盖 DDS 感知配对、相机断流后
纯点云降级和速度命令超时归零；感知、规划及 SLAM Python 核心的语句覆盖率约为 61%。

2026-09-01 稳定性回归进一步收紧三道安全合同：`vision_confirmed` 只表示视觉类别与有效
点云几何一致，不再表示“画面中存在任意候选”；自主任务进入 Action 前必须拿到新鲜、有效且
与障碍专名一致的 `NavigationSafety`，T 字台阶同时兼容完整顶部、局部踏面、多级阶梯趋势
以及严格的近场 PIT 轮廓；
最终速度门把 Twist 六个分量作为一个原子命令检查，任一 NaN/Inf 都整条归零。HSV 参数校验
允许 Hue 跨越 OpenCV 的 179→0 边界，但 S/V 上下界仍必须有序。无闪现 Gazebo 逐障碍复测
共取得 317/317 正确专名与 100% 导航健康；动态 T 台正常到达 READY 和控制器等待。未接真实
`/traverse_obstacle` 服务时仍不计物理越障成功。几何新鲜窗口现由
`autonomous_mission.yaml` 的 `safety_geometry_stale_seconds` 唯一配置。

同日第二轮鲁棒性整理继续修复无需真机即可确定的问题：点云在同帧多障碍中按通道内最近的
有效异常连通域决策，并用正/负高度异常原始回波数拒绝稀疏飞点；OpenCV 以相邻帧运动和尺度
变化跟踪平滑接近的目标，发布框始终与当前 Image Header 同帧；冲突视觉只撤销
`vision_confirmed`，不再压低权威点云置信度。导航健康同时验证最新 `map -> base_link` 动态
TF 的源时间，旧 TF 冻结不能继续维持健康。Nav2 与 TraverseObstacle 的 goal/cancel/result
异步通信均有可调看门狗和 generation 隔离；服务未就绪不会消耗补扫、恢复或回访次数，
NaN/Inf Guidance、TF、里程计及目标全部 fail-closed。Action 通信超时意味着远端控制权未知，
任务进入 `ACTION_COMMUNICATION_FAULT` 并持续锁住自主 Twist。该锁不能停止失联控制器的
关节动作：Nav2 故障可停/重启 Nav2 或核心栈，Traverse 故障必须处理外部越障控制器，
归属不明时两者都处理并使用硬件急停；不能只重启自主任务进程或只重启核心栈。

本轮隔离 ROS 域在线复验同时覆盖原始与确定性链路。纯场地、无传送的一轮矩形轨迹中，
`/navigation/healthy` 测量期 252/252 为 true，map 起终误差为 0.0047 m/0.0168 rad；完整
白名单核心进程峰值约 1228 MiB、2.29 个 CPU 核。terrain/fused Header age p95 分别为
0.34/0.44 s，fused 超过 0.35 s 软预算，保留为真机/RK3588 复测项。独立 Gazebo 传送替身
流程在 48 秒完成 8/8、pending=0、回到起点并进入 `COMPLETED`，Ctrl-C 后 `/cmd_vel` 为零。
后者只证明任务编排和接口闭环，不代表真实物理越障。

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
   - 工作：在独立 NVIDIA GPU 工作站建立 Isaac Lab/Isaac Sim 项目，将真实机器人 URDF/USD、质量、惯量、碰撞、摩擦、关节阻尼、执行器响应、PD 参数和控制频率映射到并行仿真环境。
   - 工作：冻结训练与真机共用的观测/动作合同；观测包括关节位置与速度、机身角速度、重力方向、速度指令和足端接触，动作优先定义为带安全限幅的关节目标。
   - 工作：按平地站立与速度跟踪 → 坡面 → 台阶/坑洼 → 连续比赛障碍逐级训练，并随机化质量、摩擦、地形、传感器噪声、控制延迟、外力和执行器强度。
   - 工作：构建基础状态估计接口，为训练和真机推理提供字段、单位、坐标系、顺序、归一化和时间戳完全一致的观测数据。
   - 工作：固化策略导出、离线回放和 RK3588 推理性能测试流程；部署端只加载通过限幅、超时停车和保护状态机验证的策略，不在 RK3588 上承担训练。
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
| `quadruped_gazebo` | 独立比赛场地/传感器；可选一次性传送越障替身，不启动任何算法 | `robocon_field.launch.py`、`robocon_field_teleport.launch.py` |
| `quadruped_bringup` | 感知、地形决策、速度门和占位模型公共入口 | `bringup.launch.py` |
| `quadruped_interfaces` | 带时间戳的感知、导航与越障交接合同 | 五个 `msg/` + `TraverseObstacle.action` |
| `quadruped_perception` | OpenCV、栅格地面分割、几何分类、时间同步融合 | 三个感知节点 |
| `quadruped_planning` | 地形风险分类、入口引导、速度安全门和未知地图自主任务 | 四个导航/任务节点 |
| `quadruped_teleop` | Xbox 按键安全状态机、独立 Twist 候选和自主任务进程开关 | `xbox_teleop` |
| `quadruped_tools` | rosbag 标注评估、SLAM/Nav2 长测与资源报告 | `perception_bag_evaluator`、`stack_regression` |
| `slam` | 核心 SLAM/Nav2/OpenCV 入口，以及需显式启动的独立自主导航入口 | 两个互不 include 的 launch |

主要节点：

- `vision_obstacle_detector`：OpenCV 双光照 HSV + Canny 轮廓识别，并用原始曝光、清晰度、
  高光抑制、Hue 0/179 环绕、类别投票、相邻帧位移/尺度和目标框 IoU 做多帧确认；平滑接近
  或转向不会再被整个窗口的累计变化误判为跳变，输出框与当前 Image Header 保持同帧。同时发布带 ROI、候选框、
  稳定类别和图像质量的 `/vision/annotated_image`，供 RViz 直接观察。蓝白相间限高杆会将
  至少三段水平对齐、尺寸相近且间距规律的蓝色短段合并，避免一排蓝色杂物误报；前向单根
  细长色柱也可提示立柱。无颜色的单条横边或
  闭合大框不能独立判为横杆/墙，避免地平线、台阶正面和场地边界误报。候选相机还会先检查
  frame、时间戳、宽高、步长、数据长度和 CvBridge 编码；同一来源的重复/乱序帧不会刷新
  心跳或贡献多帧投票，Image Header 间隔超时即使回调突发到达也会清空旧投票。
  未来时间戳最多容忍 0.10 s，无效、转换失败或过期主话题不会阻止备用源。
- `terrain_analyzer`：将点云转换到 `base_link`，以稳健高度栅格、连通域和异常原始回波支撑量
  估计台阶、坡度、坑洞、墙、悬空横杆和立柱；近场地面锚定避免宽台阶/桥面反报为坑，
  平面残差区分连续坡面与离散踏面；同帧出现多个结构时先选择前向通道内最近的有效正/负
  高度域，不能由远处大墙遮住近杆，也不能由近处少量飞点遮住真实障碍。送入 Nav2 前按拟合地面移除平地/坡面。候选点云只有在
  frame、时间戳、缓冲区以及浮点 XYZ 字段合同有效，并且采样时刻 TF 变换成功后，才能刷新
  active source 的健康租约。解码失败、全 NaN 或缺 TF 的来源进入短暂 cooldown 并释放所有权，
  健康备用点云可接管。切源、rosbag/Gazebo 时钟回拨、地面先验过期或连续整体高度冲突都会
  清除旧先验。正前方中央通道没有足够地面回波时输出 `UNKNOWN/invalid` 并停车：无回波既
  不是坑洞证据，也不是可通行证据。`CLEAR` 的每个纵向切片必须同时具有中心、左侧和右侧
  地面支撑，并满足最大缺口和横向覆盖率；即使已看到远处 STEP/PIT/WALL/BAR/POLE，每帧
  也会在默认 0.80 m 滚动安全前视内
  检查到 `min(0.80 m, 障碍前缘前一格)` 的接近走廊。远处盲带会在机器人接近并进入该窗口时
  触发 UNKNOWN 停车，不能被远墙掩盖。点云和相机来源同样只容忍 0.10 s 未来时间。
- `perception_fusion`：在小队列中全局寻找时间戳最接近的相机/点云对；相机断流时在
  0.25 s 后退化为纯点云结果；视觉框还必须与前向通道相交，点云始终掌握尺度权限。
  当前没有 CameraInfo/外参投影关联，因此 `vision_confirmed=true` 只在视觉类别与点云类别
  **完全相同**时产生；任何跨类提示都保持 false，且不改变点云类别或置信度。STEP 不会仅凭
  画面中的横杆/立柱提示被改成 BAR/POLE。
- `terrain_safety_assessor`：优先读取按时间戳配对的融合观测，原子发布地形模式、Nav2
  速度上限、稳定英文 `semantic_id`、有效性与几何摘要，并在终端周期显示正前方障碍中文名称、置信度、距离、
  高度和视觉介入状态，供调试及未来运动团队只读接入。名称优先采用点云量测：可显示
  主斜坡、木桥引坡、T 字形台阶、限高杆、直角绕杆区、坑区和高墙；仅有颜色证据时明确
  标注“点云待分类”，不会把笼统视觉结果冒充具体障碍。
  安全层只接受时间戳严格递增的融合帧；DDS 重发或驱动缓冲旧帧不能冒充新的连续确认或
  延长感知心跳，时钟回拨时名称和风险迟滞从 fail-closed 状态重新累计。机器控制只消费
  同一分类结果中的英文 ID；中文文本只是 UI，修改显示措辞不会改变任务语义。
- `traversal_guidance`：把已确认的赛道障碍转换为 `APPROACH → ALIGN → READY`，发布
  `base_link` 中位于障碍前方的相对入口位姿；Safety 与 Guidance 原样共享观测 Header 和
  `semantic_id`。它不调用 Nav2 Action，也不生成抬腿、关节
  或足端命令。同类目标的距离和横向中心经过低通，READY 需要连续 3 帧确认并具有独立的
  距离/角度退出迟滞；输入失效立即撤销历史。`autonomous_mission` 据此向 Nav2 下发入口
  目标，READY 后再交给运动控制器。
- `autonomous_mission`：从 `/map` 的已知—未知边界提取真实自由前沿，在 Nav2 处于活动状态
  后逐个探索；连续确认比赛障碍时冻结其 `map` 坐标，先到达入口，再调用
  `/traverse_obstacle`。只有 ROS Action 终态为 `SUCCEEDED`、控制器返回 `success=true`、实时
  TF 证明机体已越过所观测的入口平面且落地后连续稳定，才记录为已完成并选择下一前沿；
  任一证据失败均保留在待完成清单。自主任务终端每 5 秒以及清单变化时直接显示“已越过/
  未越过”；Nav2 连续 5 秒没有至少 0.04 m 平移或 0.06 rad 旋转进展时会取消当前目标。
  普通探索目标加入临时黑名单；若新鲜地形引导证明前沿被正前方障碍阻挡，或障碍入口
  的交接守卫未通过，则交替转向 90°，随后尝试 0.80 m 的普通 Nav2 安全移动来改变观察站，
  不再原地循环。障碍入口仍保留接近前确认的语义，并以实时空间、粗类型、距离、横偏和
  航向复核；停滞恢复允许在同一入口 2.35 m 安全包络内移交，全部严格证据已经满足时可在
  1.45 m 内直接移交越障控制器。
  远场已多帧确认的比赛语义会写入地图账本；近场只剩通用 STEP/WALL 时，仅在实时粗类型
  兼容且障碍 map 位置相距不超过 0.90 m 时恢复同一 ID，避免砂石坑在入口处反复换视角。
  每次回访按 8/16/32/64 秒退避；到达观察位但没重新识别也不清零。越障控制器 5 秒未就绪
  同样保留待办并继续探索。整场预算默认 300 秒：240 秒工作截止时取消未完成的
  非返程 Nav2/Traverse 目标并开始返程；300 秒硬截止时锁住自主速度、取消所有仍在
  运行的 Action，并保留未完成清单，绝不伪报 `COMPLETED`。无新目标时最多原地补扫两圈。
  Action Goal 由同一个非零 Header 下的 Guidance 姿态和 Safety 几何组成不可变快照，不会拼接
  其他时刻的字段。快照包含 header、obstacle_type/obstacle_id/entry_stage、confidence、
  distance/lateral_offset/heading_error、obstacle_height/pit_depth、slope_pitch/slope_roll、
  roughness/width、structure_heading 及其置信度、clearance_height 和 valid_points。
  Guidance `READY` 是 1.20 m 的任务交接信号，不等于已经进入抬腿窗口；
  只有距离≤0.45 m、横偏≤0.10 m、航向误差≤0.08 rad 才发 `ENTRY_READY`，其他合法交接
  发 `ENTRY_PREPARING`，由服务端先闭环最后一段低速接近/对正。任务侧最宽交接包络是
  2.35 m/0.35 m/0.22 rad，独立仿真服务端的接收上限是 2.50 m/0.35 m/0.22 rad。
  服务端必须按 `PREPARING → TRAVERSING → STABILIZING` 发布有限、单调的全局
  `progress`，并最终到达 1.0；5 秒无状态或进度实质变化会请求取消，单次越障总上限为
  45 秒。只有完整反馈序列到达 `STABILIZING/progress=1.0`、ROS Action 终态成功且
  `Result.success=true`，才会进入任务层越过入口/落地后验。
  Action 的发送响应、取消到最终结果均有有界看门狗；晚到回调用 generation 隔离。若超时后
  无法证明旧运动所有权已释放，则锁存 `ACTION_COMMUNICATION_FAULT` 和自主 Twist，不会盲目
  重试第二个目标；外部越障关节运动仍须由服务端取消或硬件急停。Nav2 暂未 ready 时不会
  提前消耗补扫、恢复或回访状态。任务使用观测时间戳查询历史 TF，并严格配对 Safety/
  Guidance 的 frame、stamp、粗类型和 `semantic_id`；不会把相邻时刻或中文名称拼成一个
  Action。`/navigation/healthy` 为 false 或超时会锁住并取消 Nav2，但保留原目标且不增加
  失败/冷却次数，只有健康连续稳定后才恢复。自主所有权通过易失的强类型
  `AutonomyLease(session_id, sequence, active, motion_allowed)` 发送；进程崩溃或 DDS 断开会让
  速度门锁住自动分支，而不会永久封住键盘/手柄人工分支。
  lease 状态为 UNOWNED/ACTIVE/EXPIRED：新 session 首帧只确认所有权且强制停车；只有同一
  session 严格递增的后续帧才能以 `motion_allowed=true` 放行已接受且可监控结果的 Nav2
  目标，或以 `active=false` 清洁释放。旧进程的迟到 release/停车 false、重复/回退序号和异
  session 消息均不能解锁。一旦断流进入 EXPIRED，确认旧所有者已停后必须重启
  navigation_speed_gate/核心栈清除锁存。`/navigation/autonomy_stop=true` 仍是额外停车否决；
  匿名 false 不再解锁，唯一放行依据是同 session 有序的 `motion_allowed` 。
  硬截止后若 Nav2/Traverse 所有权可确认释放，终态为 `INCOMPLETE_STOP`；若响应或
  取消结果超时而无法确认，终态为 `INCOMPLETE_STOP_OWNERSHIP_FAULT`。两者都不计完成；
  后者必须处理外部控制器并使用硬件急停。`/navigation/autonomy_stop` 只锁自主速度分支，
  不是整机急停，也不得无条件封锁独立人工分支。
  算法不读取 Gazebo 模型名或 world 坐标。
- `navigation_health_monitor`：运行期检查 `/scan`、`/odom`、TF、扫描结构、frame、位置/航向
  协方差以及里程计位置/航向突跳；每个健康周期同时重算 DDS 接收龄和 scan/odom Header
  源时间龄，驱动重发缓存旧帧不会给心跳续期。最新组合 `map -> base_link` 的动态源时间
  也必须新鲜，TF 发布者冻结后不会因缓存仍可查询而继续判健康。跳变会锁存到连续稳定
  样本确认恢复，输入断流则重新发布 false。
- `nav2_readiness_monitor`：复用同一数据合同，等待有效 `/scan`、`/odom` 和定位 TF 后激活 Nav2；
  生命周期服务名、传感器话题、frame、超时和扫描合同会在建立服务客户端前统一校验。
  STARTUP/GetState/ChangeState 请求具有墙钟截止、generation 隔离和迟到回调拒绝，服务失联不会永久卡住激活/恢复标志。
- `navigation_speed_gate`：检查 Nav2 命令、地形评估和导航健康心跳，任一失效立即输出零速；
  直行要求前/后雷达扇区具备连续有效覆盖，任何转向按近 360° 机身扫掠检查。当前只接受
  `linear.x + angular.z`，启用全向侧移前必须按真实 footprint 重做方向安全门。
- `xbox_teleop`：将 `/joy` 转换为带 LB 使能、B 急停和断流归零的 `/cmd_vel_joy`；十字键
  上/下可单独启动或 Ctrl-C 由该节点创建的自主导航 launch，不改变 Gazebo/SLAM 生命周期。
- `perception_bag_evaluator`：将 rosbag 预测与人工标签对齐，统计准确率、召回率和混淆矩阵。
- `stack_regression`：在明确允许仿真运动后交替执行矩形异路闭合和连续正反旋转/前进/倒退，
  再测试多目标、1 m 绕杆窄通道和不可达目标恢复；记录每轮 map/odom 起终一致性、数据断流、
  越障阶段安全合同、完整核心进程 CPU/RSS 与感知 Header 年龄。该指标不能证明 SLAM Toolbox
  已执行回环图优化；真正的回环验收仍需带可控 odom 漂移的 rosbag 做关闭/开启回环 A/B。

## 3. SLAM、Nav2、OpenCV 与点云如何协同

导航采用 Nav2 标准的“全局规划器 + 局部规划器”两层框架：`NavFnPlanner` 在 SLAM 地图和
全局代价地图上生成整段路径，`DWBLocalPlanner` 在局部代价地图上结合实时 `/scan` 与
去地面点云跟踪路径、避开新障碍。OpenCV/点云为规划层补充类别、地形与安全约束，不替代
这两个规划器。

当前技术选型保持如下：

- **强化学习统一采用 Isaac Lab。** 仓库当前还没有真实质量、惯量、执行器和关节参数，
  因此只冻结后续技术路线，不在本阶段安装或伪造训练环境。Isaac Lab 训练放在带 NVIDIA
  GPU 的开发机上；RK3588 继续负责 ROS 2 导航、观测适配、安全约束和后续策略推理。
- **全局规划暂时保留 NavFn。** 当前 14 m x 6 m 比赛场、1 Hz 行为树重规划和 2 Hz
  规划频率下，全局搜索不是已测瓶颈；动态变化由代价地图、周期重规划和 DWB 局部控制处理。
  Nav2 官方插件列表没有 D* Lite，直接切换意味着自行开发、测试和长期维护插件。拿到真机
  外形与运动约束后，优先用 rosbag 对比官方 `SmacPlanner2D`/`SmacPlannerLattice`；只有
  性能数据证明增量全局搜索确有收益时，才评估自研 D* Lite。

后续 Isaac Lab 开发按以下顺序推进，且保持与当前 SLAM/Nav2/Gazebo 工程解耦：

1. 真机尺寸和执行器确定后，建立可校验的 URDF/USD、惯量、碰撞和执行器模型。
2. 定义训练端与真机端完全一致的观测/动作接口，并为每个字段固定单位、坐标系和顺序。
3. 先训练站立与速度跟踪，再通过课程学习加入坡面、台阶、坑洼和连续障碍。
4. 使用质量、摩擦、延迟、噪声、外力和执行器强度随机化缩小 Sim-to-Real 差距。
5. 导出策略后先做离线回放、台架和保护绳测试，再在 RK3588 上测量推理延迟与资源占用；
   策略只负责运动能力，不替代本仓库的 SLAM、Nav2、OpenCV、任务编排和安全停车链。

```text
2D 雷达 /scan ──> SLAM Toolbox ──> /map + map→odom
       │                                  │
       └──────────────────────────────────> Nav2 全局/局部规划

RGB 相机 ──> OpenCV ──> /vision/obstacle_stamped ─┐
                                                  ├─> 时间同步融合
深度点云 ──> 地面分割 ──> /terrain/features_stamped┘       │
                          └─> /terrain/features（仅显式 legacy 回放）│
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
     ─> /cmd_vel ─> 未来真机底盘接口

Xbox /joy ─> xbox_teleop ─> /cmd_vel_joy ─> 未来 twist_mux/底盘仲裁

/scan + /odom + TF ─> navigation health + Nav2 readiness ─> autonomous mission
```

职责边界如下：

1. **SLAM** 用 `/scan` 在陌生环境生成地图和定位，不负责跨越动作。
2. **Nav2** 根据地图规划自由空间路线。普通环境障碍继续正常避碰；赛道越障目标不把终点
   直接设在实体后方，而是先使用 `/traversal/approach_pose` 到达并对正入口。激光和“高于
   局部拟合地面的深度点”仍写入代价地图，防止运动控制器接管前误闯障碍。
3. **OpenCV** 识别杆、限高横杆、墙面和大面积有色障碍，用于同类复核、提前减速和提示；
   未完成相机内参、外参和像素—点云投影前，不得跨类别改写点云结论。
4. **深度点云** 测量障碍高度、坡度、横向偏移和粗糙度，是 `STEP/CLIMB/STOP` 及入口
   对正的几何依据；当前任务层会完成入口导航和 Action 交接，但不生成真实跨越动作。
5. **navigation_speed_gate** 是自动导航 `/cmd_vel` 的最终发布者：应用地形限速、检查导航
   心跳，并用 `/scan` 对运动方向执行极近距离急停兜底；它不把可越障目标改成绕行目标。
6. **Xbox 手柄节点** 默认独立发布 `/cmd_vel_joy`，不加入主导航 launch，也不绕过
   未来底盘安全层；真机阶段通过 `twist_mux` 与 Nav2 速度仲裁。
7. **自主任务节点** 只使用 `/map`、TF 和感知输出决定“去哪里、何时交接”，不包含步态；
   停止服务会立即锁住 Nav2 Twist 并请求取消当前 Nav2/越障目标。真实越障控制器必须自行
   响应取消；取消结果未确认时还要使用硬件急停或停服，不能直接人工接管。

这里把“成功”分成两个层级。单项障碍成功是：控制器完成该类障碍的足端/接触/姿态闭环，
无取消、超时或安全故障；任务层随后独立确认机器人确实从入口侧移动到另一侧，并稳定
`0.75 s`。整场任务成功则还要求八项障碍分别完成、清单无漏项且最终到达终点。仅看到障碍、
Nav2 到达入口、Action 被接受或动作序列播放完，都不算越障成功。

OpenCV 不估计真实距离，也不能独立触发抬腿或跳跃。当前只有视觉与点云类别完全相同才
记为视觉确认；点云缺失、无回波、无效或超时默认 `STOP`。未来若需要视觉跨类细分，必须
先加入 CameraInfo、相机—点云外参、按 Header 查询的历史 TF、像素投影重叠/深度门以及独立
rosbag 验证，不能通过放宽同步窗口或图像中央走廊代替标定。

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
   在目标障碍的完整路径结束、关节和驱动无故障、预期足端接触成立且真实姿态闭环确认
   稳定落地后才能返回 `success=true`。任务层收到后还会进入
   `VERIFYING_TRAVERSAL_RESULT`，核对 map 位移、越过入口平面和落地稳定；它不会直接计分。
4. 真机自身发布 URDF/TF 时使用 `robot_model:=false`，避免两个
   `robot_state_publisher` 同时发布传感器固定 TF。
5. 接入时先运行 `check_integration.sh --inputs-only`，确认每个传感器确实在出数据、消息
   带有效 `frame_id` 且 TF 可达；全栈启动后再运行默认完整检查，核验 `map` TF、算法输出、
   `/cmd_vel` 消费者和可选越障 Action。所有项通过后才算完成上层—真机最小对接。

```bash
ros2 launch slam slam.launch.py robot_model:=false sensor_profile:=ros_default
./scripts/check_integration.sh --inputs-only --image /camera/image_raw --points /camera/depth/points
./scripts/check_integration.sh --image /camera/image_raw --points /camera/depth/points
```

接口的消息类型、字段、TF、超时与设备替换规则以 `connect.txt` 为准。未来可以在独立包中
新增厂家 SDK、状态估计和运动控制，但不应反向让核心感知算法依赖厂家类型。

这里的“可移植”是接口可移植，不是免标定：另一台机器仍要提供标准坐标方向和时间戳，测量
传感器外参，并用实机 rosbag 重调高度、坡度、颜色和安全距离阈值。核心节点不读取 Gazebo
模型名、比赛 world 坐标或厂家消息，因此仿真场、现实场和不同品牌驱动可使用同一套源码。

### 3.2 移植到已有机器人代码仓库

本项目确定采用**完整移植**：目标机器人已有驱动、状态估计和运动控制，但没有 SLAM、Nav2
及本项目的 OpenCV/点云感知。因此必须复制以下六个源码包，不能省略 `slam`：

| 必须迁移 | 作用 |
|---|---|
| `quadruped_interfaces` | 障碍、安全、引导和越障 Action 接口 |
| `quadruped_description` | 启动依赖和调试占位模型；真机运行时不发布该占位模型 |
| `quadruped_perception` | OpenCV、点云几何、融合与地形安全评估 |
| `quadruped_planning` | 障碍入口引导、任务编排和速度安全门 |
| `quadruped_bringup` | 感知与规划公共启动入口 |
| `slam` | SLAM Toolbox、Nav2 参数/行为树及完整启动入口 |

不要复制整个工作空间的 `build/`、`install/`、`log/`，也不要迁移
`quadruped_gazebo`、`quadruped_teleop` 和 `quadruped_tools`。
运行期只需要上述六包；若还要使用本文的对接检查和录包命令，请额外把根目录
`scripts/check_integration.sh`、`scripts/record_bag.sh` 复制到目标工作空间的 `scripts/`，
或保留一个 Wakula checkout 从其中执行。它们是开发工具，不是 ROS 运行节点。

推荐将源码包复制到目标工作空间 `src/` 后由 `rosdep` 解析依赖，先单独构建接口包，再构建
其余包。已有机器人继续拥有传感器驱动、`/odom`、TF、底盘安全和关节控制；本算法不接管
这些模块。若目标已有 `/cmd_vel` 仲裁器，启动感知链时必须设置 `speed_gate:=false`；此时
Wakula 不发布 `/cmd_vel`，目标仲裁器必须把 `/cmd_vel_smoothed` 接成“自动速度候选”，同时
消费 `/terrain/navigation_safety`、`/navigation/healthy`、强类型 `/navigation/autonomy_lease`、
`/navigation/autonomy_stop` 和 `/teleop/emergency_stop`，实现速度限制、断流、雷达与
硬件急停后再唯一发布 `/cmd_vel`。目标仲裁器必须复制 session/sequence/motion_allowed 状态机；
匿名 `autonomy_stop=false` 只是诊断兼容信号，不得用于放行。lease 只封锁自动候选，不能覆盖人工
分支；teleop 软件停车请求覆盖全部候选，但仍不替代实体急停。若采用本项目速度门，则 Nav2 必须按
`/cmd_vel_nav → /cmd_vel_smoothed → /cmd_vel` 串联，禁止两个节点同时发布最终 `/cmd_vel`。

完整复制命令、真机启动方式、Action 对接和验收清单见 `instruction.txt` 第十节；所有
话题、消息和所有权冲突见 `connect.txt` 第八节。

真机调试必须按“原始输入与 TF → 被动感知/SLAM → SLAM 回环 → OpenCV/点云 rosbag →
Nav2 空载规划 → 低速运动 → 单障碍 Action → 自主任务与故障长测”逐级进行。完整命令、
每级通过标准和故障定位方法见 `instruction.txt` 第十节第 6 项；不要首次接入就运行自主任务。

## 4. 默认与可替换传感器接口

### 4.1 真机传感器选型与预留安装位置

当前 URDF 只是约 520 × 240 × 120 mm 的通用机身，`base_link` 名义离地约 440 mm。
在整机尺寸尚未冻结时，可先按下表预留长孔、线束和保护罩；坐标采用 ROS 约定：
`base_link` 的 `+x` 向前、`+y` 向左、`+z` 向上。数值是初始范围，不是最终标定值。

| 传感器 | 建议类型 | 大概位置（相对 `base_link`） | 安装姿态与用途 |
|---|---|---|---|
| 主雷达（性价比方案） | 360° 2D ToF；暗色目标有效距离至少 8 m、10 Hz 以上、角分辨率不大于 0.5°、精度约 3 cm，直接发布 `LaserScan`。推荐 RPLIDAR S2E 规格档 | 机身顶部中心或略靠前：`x=0～+0.08 m`、`y≈0`、`z=+0.10～+0.15 m` | 扫描面水平，正方向对齐 `+x`；优先选 Ethernet、IP65 及有 Jazzy/aarch64 驱动的版本 |
| 主相机 | 主动双目 RGB-D；深度和 RGB 均优先全局快门，深度近端不大于 0.25 m、有效距离至少 3 m、水平 FOV 不小于 80°、最低 640×400@15 Hz、USB 3、支持 RGB/Depth 同步和 `PointCloud2`。推荐 Orbbec Gemini 2 L 规格档 | 前脸中央：`x=+0.28～+0.32 m`、`y≈0`、`z=0～+0.06 m` | 光轴朝前并向下俯 `10°～15°`；真机先用 640×400/480@15～30 Hz，算法仍限频 5 Hz |
| 可选 3D 雷达（增强方案） | 360° 小型 3D ToF；近盲区不大于 0.2 m、垂直 FOV 至少 45°、10 Hz、点频至少 100k/s、Ethernet/PTP、IP65 以上。推荐 Livox Mid-360 规格档 | 顶部中心，尽量接近 2D 雷达位置并高于遮挡物 | 输出 `PointCloud2`，另经 `pointcloud_to_laserscan` 生成 `/scan`；适合强光、远距三维和 RGB-D 失效冗余 |

三类设备职责不要混淆：2D 雷达主要负责平面 SLAM、定位和 Nav2 代价地图；RGB-D 或 3D
雷达负责台阶高度、坑深、坡度、粗糙度和限高净空。单独一台 2D 雷达无法量测这些三维
指标。RGB 图像当前只对点云已给出的同一类别做辅助确认；若未来要把图像类别投影到点云
并跨类细分，必须完成 CameraInfo 内参、相机—点云外参、时间同步和历史 TF 标定。

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

采购优先级建议如下：预算和开发周期优先时采用 **RPLIDAR S2E + Orbbec Gemini 2 L**。
S2E 官方规格档为 360°、10 Hz、32 ksample/s、0.1125°、Ethernet、IP65；比赛场地只有
14 m × 6 m，其暗色目标 10 m 量程已经够用。Gemini 2 L 为主动双目，RGB/IR 全局快门，
深度 0.2～10 m、H91°×V66°、1280×800@30 Hz、USB、144 g，适合运动中的近距离地形。
若场地存在强日光、RGB-D 深度有效率不足，或希望一个雷达同时承担三维地形冗余，升级为
**Livox Mid-360 + Gemini 2 L**：Mid-360 为 H360°×V59°、0.1 m 近盲区、40 m@10%反射率、
200 kpoint/s@10 Hz、Ethernet/PTPv2、IP67、6.5 W、265 g。RK3588 不直接全量重复处理；驱动
发布 10 Hz 后由现有节点限到 5 Hz/40000 点并体素降采样，同时从水平高度带生成 `/scan`。

主动红外 RGB-D 在阳光下仍必须实测；相机深度失效时，本算法会降级为纯点云，但不能用
室内标称距离替代比赛现场 rosbag。采购前还要在 Ubuntu 24.04、ROS 2 Jazzy、aarch64 真机
编译厂商驱动，并在阳光、黑色、反光材质和机身振动下验证有效回波率，确认 Header 时间戳、
`frame_id`、硬件同步和连续运行温度后再定型。参考资料：
[SLAMTEC RPLIDAR S2/S2E 官方规格](https://www.slamtec.com/en/s2/)、
[Orbbec Gemini 2 L 官方规格](https://www.orbbec.com/products/stereo-vision-camera/gemini-2l/)、
[Livox Mid-360 官方规格](https://www.livoxtech.com/cn/mid-360/specs)；Mid-360 的官方
[`livox_ros_driver2`](https://github.com/Livox-SDK/livox_ros_driver2) 已列出
Ubuntu 24.04/ROS 2 Jazzy 构建入口。

### 4.2 默认通信接口

算法内部始终保持 ROS 2 标准合同，不写死厂商品牌：

| 数据 | 内部默认 | 消息类型 | 一键入口覆盖参数 |
|---|---|---|---|
| 2D 激光 | `/scan` | `sensor_msgs/msg/LaserScan` | `scan_topic` |
| 里程计 | `/odom` | `nav_msgs/msg/Odometry` | `odom_topic` |
| RGB | 自动选择 | `sensor_msgs/msg/Image` | `camera_topic` |
| 深度/3D 点云 | 自动选择 | `sensor_msgs/msg/PointCloud2` | `point_cloud_topic` |

`slam.launch.py` 是唯一推荐的一键入口，内部直接读取 profile，并将雷达/里程计 remap
作用到 SLAM、Nav2、最终速度门、就绪监视器和 RViz；图像/点云参数同时传给
OpenCV 与地形节点。整个过程不复制高带宽数据，也不用修改算法源码。预置 profile：

```text
2D 雷达：ros_default、rplidar、ydlidar、ldlidar、hokuyo
RGB-D：  realsense_d400、orbbec_gemini2、zed2、oak_d
3D 雷达：velodyne、ouster、livox、hesai、robosense、lslidar
```

profile 是常见驱动命名的起点，不绑定具体驱动版本；实际名称不同时用四个参数覆盖。
配置集中在 `slam/config/sensor_profiles.yaml`，以后新增型号只需复制一个 YAML 段。
profile 和命令行覆盖统一要求使用 `/` 开头的绝对话题，并在启动阶段拒绝空 namespace、
空白字符及 substitution。这样错误配置会立即报出，而不会让节点在私有命名空间静默等数据。

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

全新 Ubuntu 24.04 电脑的 ROS 2 软件源、完整 apt/rosdep 依赖、VS Code 安装与八个推荐
插件、Git 克隆、硬件驱动、双机 DDS 网络和逐项验收说明见 `quickstart.txt` 第二节。
仓库的 `.vscode/settings.json` 使用 `${workspaceFolder}` 相对路径，可放在任意用户目录。

```bash
cd ~/wakula
source /opt/ros/jazzy/setup.bash
./scripts/bootstrap.sh
./scripts/build.sh
source install/setup.bash
```

`bootstrap.sh` 会直接确认 colcon 的 `ament_python` 构建扩展可导入，再让
rosdep 解析全部运行依赖。`ament_python` 只是 `package.xml` 的构建类型，
不是可由 `ros2 pkg prefix` 查找的包；脚本不使用 `-r` 或 `--skip-keys` 掩盖任何未知依赖。

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
该轮 700 个导航健康样本全部为 true。当时旧采样器只统计了部分核心进程，约
861 MiB/1.55 个 CPU 核仅作历史参考；当前完整白名单口径见后文 2026-09-01 回归。两者都不是 RK3588
温升、功耗或真机精度结论。

同日新增越障引导时序回归：两轮完整联合测试中 `/traversal/guidance` 收到 665 帧、最大
间隔 0.753 s，错误 READY 合同为 0；针对阈值边界的复测实际进入 READY 6 帧，
`ALIGN ↔ READY` 小于 0.35 s 的快速往返为 0。该轮最大位置闭环误差 0.0047 m，说明连续帧
确认和退出迟滞没有阻塞正常交接。入口 Pose 仅在存在有效越障目标时发布，避免消费者误用
无障碍状态下的零位姿。

OpenCV 合成矩阵已覆盖正常光、约 62% 暗光、局部阴影、全白过曝和 31 像素运动模糊：
暗光/阴影保留有效杆体候选，过曝帧被质量门拒绝，模糊帧的质量与置信度均下降。点云含噪
阈值扫测覆盖约 0.08 m 低台阶、0.09 m 坑、11.3°/14° 坡面和约 0.30 m 限高杆；这些是
确定性软件回归，换镜头、曝光、雷达和安装角度后必须用真机 rosbag 重做统计。

2026-08-31 的数据顺序回归进一步验证：Image、PointCloud2 和融合障碍只允许同一数据源的
Header 时间严格递增，重复/乱序帧不能重复计票或刷新安全心跳；仿真/rosbag 时钟回拨会清除
旧视觉投票、地面先验、障碍名称和风险迟滞。蓝白限高杆还增加蓝段尺寸与间距规律性门，
不规则同排蓝色杂物不会再仅凭“水平对齐”确认成横杆。加上本轮地图边缘守卫回归后，
该轮历史基线为 288 项测试通过；当前数量以本页顶部为准。

2026-08-31 又完成一轮**不启动传送/闪现 Action**的逐障碍回归。八类规则障碍各取一个
能看到关键结构的固定观察位，每类连续 40 帧，共 320/320 帧输出正确比赛名称，几何确认与
导航健康也均为 320/320；砂砾坑必须看到坑底低回波，正对护边而看不到坑底时保守输出通用
台阶属于预期行为。分开的动态入口试验中 8/8 障碍均完成 Nav2 接近/对正并进入安全交接或
控制器等待，高墙不再因自身遮挡令障碍中心贴着 SLAM 地图边缘而永久重试。该结果只证明
Gazebo 传感器条件下的识别和**入口导航**，未启动 `/traverse_obstacle` 服务时物理越障成功
数仍为 0/8；真机准确率、步态跨越和返回全程必须在真实 rosbag 与运动控制器接入后另行验收。

同日完成语义分类可读性与非法数据防护整理：分类入口把高度、坑深、粗糙度、宽度、净空
统一标为米，坡度统一转换为度，再按视觉辅助、点云粗类型、规则结构和保守回退的顺序判断。
这次重构不改变既有 320 帧基线和任何通信接口；新增防御要求全部关键几何字段均为有限数，
即使上游错误地把 NaN/Inf 消息标成 valid，也会立即输出“感知数据无效”，不能累计比赛
专名或被视觉提示重新包装成有效目标。

2026-09-01 的第二轮软件回归新增三类故障注入：近处稀疏高度噪点与远处真实障碍同帧、
OpenCV 目标平滑接近/转向与真实瞬移、Nav2/TraverseObstacle 响应或取消结果永不返回。
前两类现在分别由异常原始回波支撑门和相邻帧跟踪门区分；Action 通信不确定时则锁存自主
Twist，避免旧 Nav2 目标与新目标同时拥有速度权；外部关节控制仍靠服务端取消和硬件急停。
导航监控还验证动态 TF 的 Header 年龄，即使
`/scan`、`/odom` 继续到达，冻结的 `map -> base_link` 也会令 `/navigation/healthy=false`。
`stack_regression` 现按明确 executable 白名单统计完整核心进程，报告 terrain/fused 消息的
Header age p50/p95/max、进程启停和每轮闭合数据完整性；延迟预算仅产生软告警，RK3588
硬指标仍须在目标板上按温度、RMW 和是否启动 RViz 单独验收。

同日的独立反例复审又锁定了“无回波带后的远墙”、Action 截止与 4 Hz timer 之间的
回调竞态、旧任务迟到 release 释放新 owner、里程计/Header 缓存续命、生命周期 Future 永不完成
以及 Gazebo `PREPARING` 阻塞自身 `/odom` 回调等边界。现在每帧滚动安全前视中的障碍接近
走廊也必须连续可见；
Action 响应/结果回调自身复核墙钟截止；自主所有权与运动许可改为同一有序强类型消息；
SLAM 周期性重算源 Header 年龄并对生命周期服务使用有界代次。

可复现长测（会让 Gazebo 测试狗运动，真机不得运行）：

```bash
ros2 run quadruped_tools stack_regression --allow-motion --cycles 5 \
  --pipeline-latency-budget 0.35 --report reports/stack_regression.json
```

该工具在 Gazebo 下默认使用 `/clock`；回放系统时间数据时显式加 `--no-use-sim-time`。
报告中的 `closed_path_pose_consistency` 要求每个请求周期都有有限 map 指标，但只表示命令
轨迹结束后的位姿一致性，永久带有
`proves_slam_toolbox_loop_closure_optimization=false`，不得改写成“已证明回环优化”。

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
| `speed_gate` | `true` | 是否由 Wakula 最终速度门发布 `/cmd_vel`；关闭后目标仲裁器须接 `/cmd_vel_smoothed` 及安全/停车心跳 |
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
扫描角度/样本数及里程计数值有效，再确认 `map -> base_link` TF 存在且源时间新鲜后才会自动激活；因此没有连接传感器时可
安全打开和关闭调试环境，不会在等待 TF 的生命周期切换中崩溃。地形节点仍会等待相机
外参，这是正常安全行为。若只检查参数、不希望自动激活，可使用：

```bash
ros2 launch slam slam.launch.py rviz:=false nav2_autostart:=false
```

### 6.1 自主探索与逐障碍越障编排

正常联调只运行下面三个命令，每个终端一个命令，职责互不包含：

```bash
ros2 launch quadruped_gazebo robocon_field_teleport.launch.py
ros2 launch slam slam_sim.launch.py
ros2 launch slam autonomous_navigation.launch.py
```

第一条只提供独立 Gazebo 场地、测试载体、标准传感器/运动接口和仿真传送 Action；第二条只运行核心
SLAM、Nav2、OpenCV、点云和 RViz，且默认没有自主任务；第三条才启动自主探索与越障编排。
真机联调时用真实驱动替换第一条，后两条保持不变。

第一条在 GUI 模式下会自动打开标题为 `Wakula Simulation Keyboard` 的键盘窗口，不需要
第四条命令；点击该窗口后使用 `i/k` 前后、`j/l` 原地转向、任意其他键停车。键盘仅发布
仿真专用 `/cmd_vel_teleop`，不属于 SLAM 或自主任务。

启动该 launch 就立即执行，回到该终端按 `Ctrl-C` 就停止并取消任务；核心 SLAM、Nav2、
OpenCV、RViz 和地图继续运行。退出时先通过 `/navigation/autonomy_stop` 令 Nav2 Twist 归零，
再请求取消 Nav2/越障 Action。Gazebo 替身或已确认取消的真机控制器释放所有权后，键盘/手柄
才可人工接管；失联的真实越障控制器可能仍在驱动关节，必须用硬件急停或停服确认停止，不能
把该 ROS 速度锁当作整机急停。它不 include `slam.launch.py`，也不读取 Gazebo world。真机
最终仲裁器仍须保证任一时刻只有一个运动所有者。

运行逻辑为：选择未知地图前沿 → Nav2 探索 → 连续确认障碍 → 主动对正并导航至入口 →
调用 `/traverse_obstacle` → 成功后登记该障碍 → 优先回访已知但未完成项 → 再探索未知项。
每次只锁定一个障碍；历史记录只负责回到曾经安全的观察位，真正越障前仍要重新通过实时
点云、视觉、距离、横偏和航向门。前沿目标来自 `/map`，代码不读比赛 world 坐标；正式
坐标改变不需要修改任务算法。真机必须由运动控制团队实现
`quadruped_interfaces/action/TraverseObstacle` 服务端。核心第三个命令永远不启动仿真后端；
仿真传送服务完全归第一条 Gazebo 组合入口所有。任务确认障碍并把航向误差收敛到 0.22 rad
以内后，替身只调用一次 Gazebo SetEntityPose，把测试狗放到按实时入口距离和规则结构长度
计算的出口并返回成功；不模拟碰撞、步态或中间轨迹。

入口导航若在已确认比赛障碍的膨胀边界中止，任务层只在最新点云仍指向同一 `map`
位置且置信度、距离、横偏全部满足守卫时交给 Action，避免永久重试，也不会把普通规划
失败当成越障条件。Action 成功后继续选择下一前沿。全局代价地图采用 16 m × 8 m 滚动
窗口；任务层只容忍 0.30 m 的 SLAM 栅格发布滞后，真正离图时仍等待地图恢复。

任务清单可随时查看，输出是包含稳定英文 ID、中文名称和数量的 JSON：

```bash
ros2 topic echo /autonomy/completed_obstacles
ros2 topic echo /autonomy/pending_obstacles
ros2 topic echo /autonomy/progress
```

第三条自主任务命令所在终端还会每 5 秒直接输出一行中文清单，并在完成状态改变时立即输出，
同时显示已用时间和 300 秒剩余预算，无需另开 `topic echo`。`/autonomy/progress` 也包含
`elapsed_seconds` 和 `budget_remaining_seconds`。Nav2 任一目标连续 5 秒没有达到最小平移或
旋转进展时会取消并重规划；
到达入口但 `/traverse_obstacle` 服务端未就绪时也只等待 5 秒，该障碍仍留在未越过清单中。
若前沿、覆盖目标和可回访障碍均耗尽，任务分 8 次转满两圈复查，仍无新证据便返回起点。

默认任务总预算为 300 秒，其中最后 60 秒是返程保留窗口。第 240 秒工作截止时会
取消正在执行的非返程 Nav2/Traverse 目标，确认控制权释放后导航到终点。第 300 秒是
绝对硬截止：自主速度锁定、所有已知 Action 请求取消，未完成清单保留，终态为
`INCOMPLETE_STOP`；如果取消/结果超时而不能证明远端已释放控制权，则为
`INCOMPLETE_STOP_OWNERSHIP_FAULT`，必须停服外部控制器并使用硬件急停。Gazebo 无腿 Action 替身使用一次性传送，
没有仿真动作时长；它只验证上层任务是否正确发现、对正、登记并继续探索。
当正前方危险使地形限速为零时，DWB 可能输出约 `0.10 m/s + 0.20 rad/s` 的转向主导命令；
速度门会强制丢弃其中线速度，只放行最大 0.30 rad/s 的 yaw，使机器人能转离障碍继续返程。
导航健康、超时、外部停车和全向 0.22 m 雷达急停仍具有否决权。
普通探索目标仍执行“5 秒无 map 位姿进展即换目标”；`return_home` 单独使用 20 秒窗口，
让 Nav2 行为树有机会执行 Spin、BackUp 和重新规划，避免任务层每 5 秒提前取消唯一返程目标。
自主任务在返航、探索、覆盖、回访、入口接近/对正和观察站转向期间，以 4 Hz 发布
`/navigation/rotation_recovery`。仅当该心跳为 true 且
新鲜时，速度门才可从 DWB 的弧线命令中提取不超过 0.30 rad/s 的纯 yaw；所有 linear 分量
强制为零，心跳断流 0.8 秒自动关闭，导航健康、外部停车与 360° 雷达急停仍可否决。
这用于先把相机从障碍转开，使 `/terrain/speed_limit` 恢复后再由 Nav2 正常平移；它绝不
允许障碍前线速度，也不在 HANDOFF、TRAVERSING 或 `entry_escape` 阶段提供旁路。

当前验收状态必须区分“限时上层流程回归”和“真实感知准确率”：2026-08-31 使用上述三个
独立命令完成了一轮隔离 ROS 域联合测试。闪现组合入口依次把测试载体放到八个参考观察位，
但每一项仍必须经过标准融合消息、地形安全决策、语义多帧确认、`TraverseObstacle` Action、
实际位移/入口平面和稳定落地后验后才能登记。实测 48 秒完成 8/8，未完成列表为 0，物理模型
回到起点，任务进入 `COMPLETED`，得分 1300，距离 180 秒目标还剩 132 秒。

该成绩只证明“识别合同 → 对正/交接 → Action → 成功后验 → 清单 → 下一障碍 → 返航”的
任务闭环能在三分钟内完成，不证明原始 OpenCV/深度点云分类精度、SLAM 回环或真机步态能力。
为使限时流程可重复，`robocon_field_teleport.launch.py` 默认启用 Gazebo 专属参考观察位和
确定性 `/perception/fused_obstacle` 输入，并只在该入口关闭原始深度点云桥，避免两种感知源
竞争；RGB、2D 雷达、里程计和 TF 仍正常发布。原始传感器与算法准确率必须使用
`robocon_field.launch.py` + `slam_sim.launch.py` 单独测试，纯场地入口的点云桥默认保持开启。
闪现流程不会发布任务完成清单；核心仍是唯一记账者。

八项全部完成后自动导航到终点并进入 `COMPLETED`。默认终点就是第三条命令启动时实时
记录的起点，因此仍只需三个命令。如果正式比赛终点不同，可由独立赛务节点提前发布
`/autonomy/finish_pose`（`PoseStamped`、`frame_id=map`）覆盖；这只是标准 ROS 接口，不让
任务算法依赖 Gazebo 或固定场地坐标。

Gazebo 场地与算法完全分开。只测试传感器/建图时使用纯场地入口：

```bash
ros2 launch quadruped_gazebo robocon_field.launch.py
```

需要测试完整自主任务时，第一条改为：

```bash
ros2 launch quadruped_gazebo robocon_field_teleport.launch.py
```

该组合入口仍只属于 Gazebo：它在纯场地基础上增加 `/traverse_obstacle` 一次性传送替身、
限时回归观察位和仿真融合输入，不加载 SLAM、Nav2、OpenCV 或自主任务。完成八项后先把物理
模型放回 world 起点，再通过已有 `/autonomy/finish_pose` 接口同步其当前 `map` 坐标，以免多次
非物理传送造成 SLAM 坐标跳变后反复发送零意义的返航目标。`autonomous_navigation.launch.py`
不检测或启动任何仿真节点；真机由真实控制器提供同名 Action。

启动 `slam.launch.py` 后先看终端摘要。仿真联调必须显示
`simulation_detected=true, use_sim_time=true, robot_model=false`；入口会对 `/clock` 的
非零发布者数量做多次 DDS 发现重试，避免把只有订阅者或残留图信息的话题误判成仿真，
也避免 Gazebo 已运行却因首次查询漏报而错误使用系统时间。若摘要不是这三个值，
不要开启自动任务，可直接使用等价的显式仿真入口 `ros2 launch slam slam_sim.launch.py`。
该显式入口会先打印 `Wakula simulation mode`；随后核心摘要中的
`simulation_detected=false` 只表示无需再次自动探测，实际必须是
`use_sim_time=true, robot_model=false`。

### 6.2 比赛障碍参考场地（独立启动，不属于 slam.launch.py）

2026 年第二十五届 ROBOCON 仿生足式挑战赛规则 V2.0 已公布的 14 m × 6 m 场地、
8 类障碍尺寸和规定颜色位于
`src/quadruped_gazebo/worlds/robocon_obstacle_field.sdf`。规则明确说明障碍排列和安装位置
赛前另行公布，因此当前全局坐标只是图 1 的易修改参考布局；八个坐标集中在 world 顶部的
`REFERENCE LAYOUT` 框架区，取得正式坐标后只改这八个 `layout_*` frame，不修改 SLAM、
Nav2 或 OpenCV。

| 障碍 | world 中锁定的规则数据 |
|---|---|
| 直角绕杆 | 相邻杆 1.00 m；杆高 0.55 m，满足“不低于 0.50 m”；三个必达区距杆 0.40 m、直径约 0.35 m，为红色虚线视觉圆且没有碰撞体；杆体橘色 |
| 砂砾碎木坑 | L 形外包络 2.00 m × 2.00 m、臂宽 1.00 m、深约 0.10 m、护栏/木槛高 0.15 m；护栏橘色 |
| 限高杆 | 蓝白 PVC 横杆长 1.00 m，横杆底部离地 0.30 m |
| 斜坡 | 水平投影长 3.00 m、总宽 2.00 m、坡角 11.3°，升高约 0.599 m；橘色 |
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

场地、传感器和“对准后传送到出口”的仿真 Action（整场上层流程测试推荐）：

```bash
ros2 launch quadruped_gazebo robocon_field_teleport.launch.py
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
360° 雷达位于机身中心且保持水平，并通过 Gazebo 可见掩码忽略测试狗自身外观，避免自遮挡写入地图。RGB-D
光心位于测试狗机头外侧并向下俯视约 14°，相机外观位于光心后方，避免机头遮挡画面或
深度云全部变成无穷远；RGB 图像使用 `camera_optical_frame`。Gazebo 当前生成的 PointCloudPacked 数值轴实际采用
`camera_link` 约定，因此仿真专用 bridge 会覆写点云 frame，避免算法把点云重复旋转。
真机仍应由驱动发布真实 frame 和 TF，不需要这一仿真修正。RViz 中应看到 `/map`、
LaserScan、机器人 TF、Nav2 代价地图，以及 `Camera Detection` 面板中的识别标注画面。
标注图 `/vision/annotated_image` 使用 RELIABLE、小队列 QoS，与 RViz 默认 Image 订阅兼容；
原始相机输入仍使用低延迟传感器 QoS。
默认 RViz 已关闭容易遮挡地图的 TF、网格和实时 LaserScan；需要查原始雷达时再手动勾选
LaserScan。地图中白色是已观测自由区、黑色是占用区、灰色是未知区。开放场地初始地图
会从出生点向可见障碍展开，白色射线边缘是探索范围而不是墙；应低速覆盖通道、在转角
旋转观测并完成回环，再用黑色墙线是否重合评价地图质量。实测闭环已消除旧模型的黑色
放射假墙；地图以 0.25 s（4 Hz）周期发布，扫描匹配最多约 10 Hz，移动 8 cm 或旋转
0.08 rad 即可加入新关键帧，改善倒退和原地旋转时的跟随。RViz 顶视图跟随 `base_link`，但全局固定坐标仍是
`map`。地图整体相对屏幕旋转只代表 `map` 坐标方向，不是几何错误。

算法运行时不要让键盘与自动导航同时直接发布 `/cmd_vel`。Gazebo 场地 launch
现内置仿真专用速度仲裁器：算法保持标准 `/cmd_vel`，键盘走 `/cmd_vel_teleop`，Xbox 走
`/cmd_vel_joy`；有效人工输入拥有最高优先级，唯一输出 `/cmd_vel_gazebo` 再送入模型。
自主任务退出只锁自动导航分支；确认 Nav2/越障控制器已释放所有权后，键盘和手柄仍能人工接管。GUI Gazebo 默认已经自动打开
键盘窗口；仅在使用 `keyboard_teleop:=false` 或需要单独排障时才运行：

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
自主功能仍然默认关闭；只有十字键上出现一次按下边沿时，`xbox_teleop` 才执行
`ros2 launch slam autonomous_navigation.launch.py`。十字键下向这个子进程组发送 SIGINT，
效果等同在自主任务自己的终端按 `Ctrl-C`；它不会关闭 Gazebo、`slam.launch.py`、手柄节点，
也不会搜索或杀死从其他终端手动启动的自主任务。按住十字键不会重复触发。

| 控件 | 当前作用 |
|---|---|
| 左摇杆上下/左右 | 前后移动/横移 |
| 右摇杆左右 | 左右偏航转向 |
| LB | 摇杆回中时按下解锁，持续按住使能；松开立即归零 |
| A / X / Y | 低速档 / 正常档 / 快速档 |
| B | 锁存软件急停 |
| Start | 松开 LB 且摇杆回中时解除软件急停 |
| 十字键上 | 启动独立 `autonomous_navigation.launch.py`，已运行时不重复启动 |
| 十字键下 | Ctrl-C 并结束由本 Xbox 节点启动的自主任务，不影响 Gazebo/SLAM |
| RB、Back、Guide、左右摇杆按下 | 预留，当前不产生动作 |
| LT、RT、十字键左右、右摇杆上下 | 预留，当前不产生动作 |

若带着非零摇杆按下 LB，节点会拒绝解锁；即使随后回中也必须松开并重新按下 LB，避免
手柄放置姿态造成突然起步。若 `/joy` 断流，重连后同样必须先松开再重新按下 LB，避免
沿用断流前的使能状态。调试时查看 `/cmd_vel_joy`、`/teleop/active`、
`/teleop/emergency_stop`、`/teleop/speed_mode` 和 `/teleop/autonomy_process`。若十字键
上下方向相反，把 `xbox.yaml` 的 `dpad_y_direction` 改成 `-1.0`；若某个摇杆方向相反，只需在
`xbox.yaml` 将对应 `*_direction` 改成 `-1.0`。默认不直接发布
`/cmd_vel`，防止与 Nav2 同时控制。Gazebo 最终 mux 已消费 `/teleop/emergency_stop`：B
会覆盖人工与自主候选并清除旧 Twist，仿真 TraverseObstacle 服务端也会拒绝新 Goal 并中止
正在执行的 Goal，不会在停车后返回 success。Start 解锁后必须收到新命令才会再动。真机的最终
`twist_mux`/安全层也必须消费该锁存话题，但它仍只是 ROS 软件停车请求，不能替代实体急停、
驱动失能和底层看门狗。该节点只输出机身速度，仍需运动控制团队把 Twist 转换为四足步态。

## 7. OpenCV 障碍识别

节点同时使用两类轻量特征：

- HSV 橙色/蓝色区域：对比赛场地中颜色明显的杆和横杆优先识别。
- 灰度 Canny 轮廓：用于补充成对立柱的结构一致性；纯边缘横线或大矩形不会独立声称是限高杆/墙。

每帧先在未经增强的原图上评估曝光、动态范围和清晰度，避免 CLAHE 把暗光噪声伪装成
有效纹理；随后合并原图与 CLAHE 图的 HSV 掩膜，在保留正常色相的同时补回阴影中的橙/蓝
区域。接近纯白且低饱和的高光区域会从 Canny 边缘中膨胀剔除，降低场馆灯光、金属反射和
局部过曝形成假障碍的概率。连续蓝色横杆需满足长宽比和画面占比；蓝白交替横杆则要求
至少三个小蓝段在同一水平线上、跨度有限且覆盖率足够，所以接近后落到画面下部也能识别，
又不会把地平线粘成横杆。严重欠曝、过曝或失焦图像在进入历史窗口前被拒绝；最小轮廓
面积同时采用像素下限和图像面积比例，避免切换分辨率后检测尺度突变。
双立柱必须同时满足高度、垂直重叠、间距、宽度和填充率一致性；另一根暂时出画时，仅
前向通道内高宽比足够大的单根着色柱可作为提示。颜色候选与边缘支持共同计算置信度。
最近 5 帧不仅要求至少 3 帧且达到 60% 同类投票，还要求位置、尺寸和目标框
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
黄框是当前帧候选，绿框是多帧确认后的稳定障碍；顶部 `FRONT` 优先显示最新点云融合
几何，`VISION` 显示 OpenCV 类别，`IMAGE QUALITY` 显示输入质量。安全判断仍以点云融合
结果为准，因此终端中的
`[正前方障碍]` 可能比单帧视觉框更保守。纯 Canny 大框不再输出 `WALL`；墙面名称由
点云实测高度和垂直跨度确认。融合结果还会发布中文速查话题：

当 OpenCV 发现目标而点云尚未确认尺度时，终端会明确显示“视觉疑似××（点云未确认，
已限速）”；一旦点云确认台阶、墙等几何类别，中文名称以点云融合结果为准。超大、贴近
整幅画面的纯边缘轮廓会被视为地面/天空边界或近距遮挡，不单独触发视觉限速。

比赛专名采用可移植的传感器证据，不读取 Gazebo 模型名或坐标：约 11.3° 的低横滚坡面显示
“主斜坡”，约 14° 显示“木桥引坡”；宽且约 0.40 m 高的阶梯显示“T 字形台阶”。仅凭
局部单帧无法可靠区分木桥 A/B 或普通踏板时，会如实显示“A/B 待结构确认”或“台阶或木桥
踏板（待结构确认）”。木桥 B 的宽平桥板与 0.40 m 间隙组合、砂砾坑的低护栏/粗糙填料、
限高杆的约 0.32 m 支柱会给出对应的接近阶段名称；证据不足时保留“待确认”，不猜测坐标。
最终比赛专名还需连续 3 帧一致才会切换，恢复“无障碍”需连续 4 帧；这能抑制墙/台阶
阈值附近和近裁剪时的单帧跳变。感知数据无效不经过迟滞，会立即清除旧名称并停车。

```bash
ros2 topic echo /perception/front_obstacle_name
```

## 8. 当前地形决策边界

| 模式/类别 | 当前处理 | 是否执行腿部动作 |
|---|---|---|
| `WALK` | Nav2 速度上限为 1；视觉证据可将上限降至 0.35 | 否 |
| `POLE` | 普通/矮立柱由 Nav2 低速绕行；高度≥0.45 m 且语义确认为直角绕杆赛项时，进入 Action 任务流程 | 否；由 Action 服务端负责 |
| `STEP` / `PIT` / `WALL` / `BAR` | 远处低速接近入口；确认专名后先原地对准，再进入 1.20 m 交接区发布 `READY` 并停车 | 否 |
| 可量测坡面 | 发布坡面越障候选及入口引导；交接区停车 | 否 |
| 数据断流、TF 失败或字段非法 | 发布 `STOP` 和零速度上限 | 否 |

`/traversal/guidance`、`/traversal/phase` 和 `/traversal/approach_pose` 仍只表达
`APPROACH/ALIGN/READY` 与相对入口建议；`autonomous_mission` 消费这些消息并负责
“选择前沿→发送入口目标→READY 交接→等待 Action 结果→继续探索”。仓库已定义
`TraverseObstacle` Action 合同，但没有 SDK 网关、关节轨迹或真实越障服务端；仿真服务端
只用于验证编排，不等价于机器狗跨越能力。真机服务端必须自行实现客户端失联、取消、
内部超时和硬件急停，并以足端接触、机身姿态、驱动无故障及落地连续稳定判定
`Result.success`；仿真传送成功绝不能替代这些真机证据。

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
不足时保持停车。`vision_confirmed` 仅表示视觉类别与有效点云类别完全相同且通过了基础
空间走廊检查；
它不表示“相机看到了某个框”，冲突帧不得借该位影响下游安全判断。若相机断流，融合器等待 `0.25 s` 同步窗口后继续发布
`vision_confirmed=false` 的纯点云几何，避免辅助相机成为安全链单点故障。关闭
OpenCV 后仍消费带 Header 的 `/terrain/features_stamped` 强类型路径，便于只有 3D 雷达的
真机继续使用且不丢失采样时间、frame 和有效性。无 Header 的 `/terrain/features` 只在
显式设置 `legacy_features_enabled=true` 时供旧 bag/旧节点兼容，不是默认安全路径。

判定默认值：高度 `0.07 m` 起分类为 `STEP`，`0.18 m` 起分类为 `CLIMB`，`0.32 m` 起
标记为必须重规划；当前三种情况都会停车。阈值必须依据机器狗的实际腿长、质心、步态
能力和相机安装误差重新标定。

兼容字段仍使用纵向低分位地面包络；强类型输出另将点云压成 XY 高度栅格，优先用机器人
前方最近约 30%（最多 0.65 m）的有效栅格锚定当前地面，再迭代 MAD 剔除离群格并拟合
`z=ax+by+c`。这会阻止宽台阶、木桥或坡面因占据多数而被当成“地面”，继而把真实入口
反报成坑。连续 11.3°/14° 平面还需满足横纵跨度和低拟合残差；离散踏面继续归为 STEP。
随后计算俯仰/横滚坡度、坑深、墙面
垂直跨度、横杆净空和立柱宽度。高处/低处异常不仅必须形成可配置的八邻域连通区域，还要
达到最小原始回波数，分散或相邻的少量飞点都不会组成障碍。横杆净空只用高于地面的物体
回波计算；带落地支柱的限高杆会额外寻找占优水平高度带，避免支柱把净空拉到零。规则
70 mm 立柱允许“少栅格但原始回波充足”进入三维细长结构复核。坑洞必须看到真实低处回波，
单纯无点按未知处理，避免把盲区误判成坑。连通域严格复用建格时的向下取整规则，避免
栅格边界附近的细障碍格被四舍五入合并。
`frontal_obstacle_height`
只统计中央通道，`lookahead` 是最近成片障碍的实际 x 距离，不再是固定 ROI 长度。

本轮 Gazebo 真传感器回归已逐项确认：T 字台阶输出 `STEP / T 字形台阶`，砂砾坑输出
`PIT`（约 0.09 m 深），限高杆输出 `BAR`（约 0.31 m 净空），直角绕杆输出 `POLE`
（约 0.08 m 宽），主斜坡输出约 11.3°，木桥 B 的桥板间隙输出对应专名。算法没有读取
world 坐标或模型名；这些数值只是回归基线，真机仍必须以 rosbag 重新标定。

2026-08-25 的短流程联合回归选取直角绕杆、砂砾坑、高墙和 T 字台阶验证：高墙在斜向
初始视角下连续输出 `WALL / 高墙`，任务先原地修正约 14°，再接近并交接；直角绕杆同样
先修正约 11°。修复了换视角 Nav2 goal 被下一帧 READY 提前取消的问题，并消除了
0.15～0.35 m 剩余入口距离与小角度之间的控制死区。Action 前最终航向要求约 7° 内，
单次预对正最多 30°，转后等待新点云再决定下一步。

算法先用拟合平面计算 ROI 中每点的相对地面高度，只将凸起降采样发布为
`/perception/obstacle_points`，因此 11.3°/14° 坡面不会随前向距离增加而被误标成墙；
低台阶即便位于机身 `base_link` 下方、绝对 z 为负，也不会被 Nav2 的高度过滤漏掉。
经真实低回波与连通域确认的坑洞会被投影为贴近局部地面的虚拟障碍点写入代价地图；
无回波盲区仍按未知处理，不能凭空制造坑洞。
Nav2 local costmap 以该 `PointCloud2` 进行 marking，2D 雷达继续负责 marking + clearing。点云层不主动
clearing，防止短暂深度空洞错误清除障碍；激光清障和滚动窗口会移除离开视野的旧区域。

### rosbag 离线标定与准确率报告

完整采集矩阵、CSV 标签含义、HSV/横杆间距/点云阈值调整顺序、原始话题离线重算和首轮
验收线见 `instruction.txt` 第五节。这里仅保留命令速查。

先固定相机曝光/白平衡，确认 CameraInfo、点云单位、共同时间源和传感器到 `base_link` 的
外参，再采集；基础数据错误不能靠放宽识别阈值解决。标准话题可直接使用：

```bash
./scripts/record_bag.sh --profile ros_default --print-topics
./scripts/record_bag.sh --profile realsense_d400
./scripts/record_bag.sh --scan /front/scan --odom /robot/odometry \
  --image /rgb/image_raw --points /depth/points \
  --camera-info /rgb/camera_info
./scripts/replay_bag.sh bags/某次记录 0.5  # 以 /clock 回放，初始暂停
```

录包脚本会加载工作空间并复用 `sensor_profiles.yaml`；更换设备应选择 profile 或用具名参数
覆盖话题，不要直接修改脚本中的列表。它同时记录原始传感器、CameraInfo、TF、诊断和强类型
中间结果，便于同一 bag 重新跑不同参数。

评估器直接读取 `/vision/obstacle_evidence` 和兼容地形结果，不需要启动实时节点。先生成
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

需要区分两类标定：评估器可在已有低带宽结果上搜索 `vision_min_confidence` 和
`step/climb/stop_threshold`；HSV、ROI、`segmented_bar_max_gap_ratio/max_gap_cv`、地面分割、
坑/墙/杆几何阈值发生变化后，必须按 instruction 第五节重新播放原始 Image/PointCloud2，
重新运行感知节点并录制新结果。旧 bag 中的算法输出不能代表新参数。

## 10. 速度与失效安全

速度链路固定为：

```text
/cmd_vel_nav -> /cmd_vel_smoothed -> /cmd_vel
```

- Nav2 controller 只发布 `/cmd_vel_nav`。
- Velocity Smoother 限制加速度并发布 `/cmd_vel_smoothed`。
- `navigation_speed_gate` 应用 `/terrain/speed_limit`，同时检查命令、评估和
  `/navigation/healthy` 心跳，并读取 `/scan` 做 0.22 m 极近距离运动方向急停。Twist 的三轴
  平移和三轴转动按一个原子命令校验，任一分量为 NaN/Inf 时整条命令归零。
- `navigation_speed_gate` 直接发布标准 `/cmd_vel`，并在同一节点完成限速、心跳和近距离
  扫描兜底，减少速度链上的额外进程和所有权歧义；该扫描不参与全局避障或越障分类。
- 若设置 `speed_gate:=false`，上述最后一段不存在：目标机器人的 twist_mux/安全层必须把
  `/cmd_vel_smoothed` 接成自动候选，同时消费 NavigationSafety、navigation/healthy、
  强类型 autonomy_lease、autonomy_stop 和 teleop emergency_stop，并实现命令超时、雷达/硬件
  急停后唯一发布 `/cmd_vel`。自动候选只能由当前 session 严格递增的
  `active=true,motion_allowed=true` 放行，匿名 autonomy_stop=false 不是解锁证据。lease 只封锁自动候选；
  teleop 软件停车覆盖
  全部候选，但不能替代实体急停。

规划命令或地形决策心跳任意一项超时，速度门都会发布零速度。这只是导航软件层的失效
停车，不替代未来真机必须具备的硬件急停、驱动失能、姿态/关节保护和底层看门狗。

融合模式采用非对称防抖：紧急 STOP 立即生效；STEP/CLIMB 需要连续几帧几何证据；向
更安全等级恢复时要求更多连续安全帧。这样既不延迟紧急停车，也减少飞点和阈值抖动。
已确认的台阶、坑洞、墙和横杆在 1.20 m 以外保留 0.25 倍低速窗口，让 Nav2 到达入口
并对正；进入交接区立即归零并发布 READY。普通/矮立柱保持 0.35 倍速度由代价
地图避碰；只有高度≥0.45 m 且语义确认为直角绕杆赛项时才进入 Action 流程。坡面会形成越障
引导候选，但真机没有运动控制器时仍只能在交接处停车。
视觉不能覆盖或跨类细分点云：当前仅在视觉类别与权威点云类别完全相同时确认；视觉与几何
冲突时保留点云类别及置信度，只撤销视觉确认位，等待后续同步帧。将来若要让图像把 STEP
重分类为 BAR/POLE，必须先接入 CameraInfo、相机—雷达外参、按图像时间查询的历史 TF、
像素投影重叠和深度门，并用真机 rosbag 标定后再修改合同。
高度、坡度、粗糙度、点数、消息采样时刻和超时等运行参数也在节点入口及纯决策函数处
进行合法性防御；NaN、Inf、旧帧、未来帧、退化里程计四元数、越量程雷达回波或乱序高度
阈值都不能被解释为可通行。

已加入规则针对性合成回归：橙色双杆/单杆、连续与蓝白分段横杆、带落地支柱的约 0.30 m
限高杆、70 mm 窄立柱、宽多级台阶、0.30 m 高墙、真实低回波坑洞及平地后的 14° 坡面；
同时覆盖黑场、白场、阴影、运动模糊、镜面高光、点云飞点、
相机/点云乱序和旧时间戳。合成测试只能防止代码回退，真机阶段仍必须按比赛场地录包验收。
新增回归还覆盖相机断流历史清除、纯点云降级、栅格边界细障碍及里程计跳变锁存恢复。

核心 Python、launch、参数 YAML、行为树、ROS 消息和运维脚本均已补充设计注释；
`/terrain/features` 的下标已集中为具名常量。七个核心 YAML 顶部还统一提供真机故障现象、
优先参数、调整方向和副作用。维护时不要为逐行翻译代码而增加注释，应优先记录数据流、
坐标系、单位、算法假设、安全边界以及更换传感器后必须重新标定的内容。代码助手必须先读
根目录 `AGENTS.md`，避免破坏 Gazebo/SLAM/自主任务的独立边界或把仿真坐标写进算法。

## 11. Robocon 比赛逻辑

| 待实现内容 | 前置条件 |
|---|---|
| 正式计时、指定障碍顺序、裁判协议和终点坐标 | 正式规则与场地坐标冻结 |
| 障碍完成、失败、取消和有限重试 | 真机越障控制器能返回可信结果 |
| 足端接触限制和台阶计分 | 真实足端力/接触检测完成 |
| Nav2 与越障控制切换 | 基础步态、急停和全身控制通过台架验收 |

当前已完成不依赖场地坐标的自主前沿/覆盖探索、已知待完成障碍回访、主动对正、入口接近、
Action 交接、完成去重、任务清单和终点导航；默认把实时起点作为终点，也允许标准话题覆盖。
仓库仍不发布 `/competition/*` 话题，也没有正式障碍顺序、裁判计时/计分或裁判状态机，
避免在缺少真机反馈时把通用自主流程误认为完整比赛能力。

## 12. 配置文件索引

| 配置 | 内容 |
|---|---|
| `AGENTS.md` | 供 Codex/代码助手读取的范围边界、参数归属、注释和验证约定 |
| `quadruped_interfaces/msg/`、`action/` | 地形、视觉、融合消息与越障 Action 合同 |
| `quadruped_description/urdf/` | 未标定外形、关节和传感器占位坐标系 |
| `quadruped_gazebo/worlds/robocon_obstacle_field.sdf` | 规则障碍尺寸、颜色和集中式参考布局 |
| `quadruped_gazebo/launch/robocon_field.launch.py` | 纯 Gazebo/传感器桥入口，不加载算法或越障替身 |
| `quadruped_gazebo/launch/robocon_field_teleport.launch.py` | Gazebo 场地 + 对准后一次传送 Action；不加载算法 |
| `quadruped_gazebo/launch/sim_traversal_controller.launch.py` | 仅单独排查仿真 Action，不启动场地或算法 |
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
