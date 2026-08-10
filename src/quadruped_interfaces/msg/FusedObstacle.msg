# 时间同步后的相机与点云观测，供规划、记录和离线评估使用。
std_msgs/Header header

uint8 UNKNOWN=0
uint8 CLEAR=1
uint8 STEP=2
uint8 PIT=3
uint8 WALL=4
uint8 BAR=5
uint8 POLE=6

uint8 obstacle_type
float32 confidence
bool geometry_confirmed
bool vision_confirmed
float32 obstacle_height
float32 pit_depth
float32 slope_pitch
float32 slope_roll
float32 roughness
float32 distance
float32 width
float32 clearance_height
float32 time_skew
uint32 valid_points
