# 带采样时间和坐标系的地形几何合同。所有长度单位为米，角度单位为弧度。
std_msgs/Header header

# 障碍类别在 TerrainFeatures、FusedObstacle 和 NavigationSafety 中保持同一编码。
uint8 UNKNOWN=0
uint8 CLEAR=1
uint8 STEP=2
uint8 PIT=3
uint8 WALL=4
uint8 BAR=5
uint8 POLE=6

bool valid                 # 本帧是否满足点数、TF 和几何有效性检查
uint8 obstacle_type
float32 confidence         # 启发式几何置信度，范围 0..1，并非统计概率
float32 ground_height      # base_link 下估计地面高度，m
float32 obstacle_height    # 障碍顶部相对地面高度，m
float32 pit_depth          # 低于地面的最大有效深度，m
float32 slope_pitch        # 前后坡角，rad
float32 slope_roll         # 左右坡角，rad
float32 roughness          # 地面拟合残差，m
float32 distance           # 最近有效障碍前向距离，m
float32 lateral_offset     # 障碍连通区域中心相对 base_link 的横向偏移，左正右负，m
float32 width              # 障碍横向连通宽度，m
float32 clearance_height   # 横杆下沿相对地面净空，m
uint32 valid_points        # 当前 ROI 内参与判断的有效原始点数
