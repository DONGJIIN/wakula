# OpenCV 的带时间戳辅助证据；归一化框坐标范围为 0..1。
std_msgs/Header header

uint8 NONE=0
uint8 POLES=1
uint8 HEIGHT_BAR=2
uint8 WALL=3
uint8 COLORED_OBSTACLE=4

uint8 obstacle_type
float32 confidence
float32 center_x
float32 center_y
float32 width
float32 height
