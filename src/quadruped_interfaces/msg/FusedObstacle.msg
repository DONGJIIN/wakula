# 时间同步后的相机与点云观测，供规划、记录和离线评估使用。
std_msgs/Header header

# 类别由点云几何主导；视觉只能确认或细分类别，不能单独给出尺度。
uint8 UNKNOWN=0
uint8 CLEAR=1
uint8 STEP=2
uint8 PIT=3
uint8 WALL=4
uint8 BAR=5
uint8 POLE=6

uint8 obstacle_type
float32 confidence         # 融合后的保守置信度，范围 0..1
bool geometry_confirmed   # 点云字段通过完整有效性检查
bool vision_confirmed     # 同步视觉证据通过置信度和类别一致性检查
float32 obstacle_height   # 以下长度单位均为 m，角度单位为 rad
float32 pit_depth
float32 slope_pitch
float32 slope_roll
float32 roughness
float32 distance
float32 lateral_offset     # 障碍中心横向偏移，左正右负，m
float32 width
float32 clearance_height
float32 time_skew          # 本次配对的图像/点云 Header 时间差，s
uint32 valid_points        # 点云 ROI 的有效原始点数
