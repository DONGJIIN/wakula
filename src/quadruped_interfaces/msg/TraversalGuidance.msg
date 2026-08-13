# Nav2 到未来越障控制器之间的只读引导合同。
# 本消息只描述“接近入口、对正、可交接”，绝不包含关节/足端动作命令。
std_msgs/Header header

uint8 PHASE_INVALID=0
uint8 PHASE_CLEAR=1
uint8 PHASE_APPROACH=2
uint8 PHASE_ALIGN=3
uint8 PHASE_READY=4

uint8 OBSTACLE_UNKNOWN=0
uint8 OBSTACLE_CLEAR=1
uint8 OBSTACLE_STEP=2
uint8 OBSTACLE_PIT=3
uint8 OBSTACLE_WALL=4
uint8 OBSTACLE_BAR=5
uint8 OBSTACLE_POLE=6

uint8 phase                # 当前越障引导阶段
uint8 obstacle_type        # 与 TerrainFeatures/FusedObstacle 完全相同的类别编码
bool perception_valid      # false 时其余字段不得用于导航或控制
bool traversal_required    # true 表示需要未来越障控制器接管；POLE 等导航物体为 false
bool ready_for_handoff     # 已到入口且对正；运动团队未来只在此状态申请接管
float32 confidence
float32 distance           # 当前障碍前缘距离，m
float32 lateral_offset     # 障碍中心横向偏移，左正右负，m
float32 heading_error      # 建议对正角，逆时针为正，rad
float32 approach_x         # base_link 中的相对入口目标 x，m
float32 approach_y         # base_link 中的相对入口目标 y，m
float32 approach_yaw       # base_link 中的相对入口朝向，rad
float32 speed_limit        # 接近/对正阶段建议 Nav2 速度比例，范围 0..1
