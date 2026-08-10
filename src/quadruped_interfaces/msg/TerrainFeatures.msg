# 带采样时间和坐标系的地形几何合同。所有长度单位为米，角度单位为弧度。
std_msgs/Header header

uint8 UNKNOWN=0
uint8 CLEAR=1
uint8 STEP=2
uint8 PIT=3
uint8 WALL=4
uint8 BAR=5
uint8 POLE=6

bool valid
uint8 obstacle_type
float32 confidence
float32 ground_height
float32 obstacle_height
float32 pit_depth
float32 slope_pitch
float32 slope_roll
float32 roughness
float32 distance
float32 width
float32 clearance_height
uint32 valid_points
