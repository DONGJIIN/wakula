# 感知/导航团队交给未来底盘或运动控制团队的只读原子状态。
# 本消息描述环境与 Nav2 约束，不是动作命令；消费者不得据此直接驱动关节。
std_msgs/Header header

uint8 MODE_UNKNOWN=0
uint8 MODE_WALK=1
uint8 MODE_STEP=2
uint8 MODE_CLIMB=3
uint8 MODE_STOP=4

uint8 OBSTACLE_UNKNOWN=0
uint8 OBSTACLE_CLEAR=1
uint8 OBSTACLE_STEP=2
uint8 OBSTACLE_PIT=3
uint8 OBSTACLE_WALL=4
uint8 OBSTACLE_BAR=5
uint8 OBSTACLE_POLE=6

uint8 mode                 # 当前稳定导航模式；未知输入必须转换为 MODE_STOP
float32 speed_limit        # 对 Nav2 Twist 的保留比例，范围 0..1
bool perception_valid      # false 时消费者必须忽略全部几何字段并停车
bool visual_assist_active  # 视觉证据是否正在额外限制 WALK 速度

# 障碍类别常量与 FusedObstacle/TerrainFeatures 完全一致。
uint8 obstacle_type
float32 confidence         # 下列字段是同一时刻的原子只读快照
float32 obstacle_height
float32 pit_depth
float32 slope_pitch
float32 slope_roll
float32 roughness
float32 distance
float32 width
float32 clearance_height
uint32 valid_points
