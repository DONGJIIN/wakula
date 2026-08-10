# OpenCV 的带时间戳辅助证据；归一化框坐标范围为 0..1。
std_msgs/Header header

uint8 NONE=0
uint8 POLES=1
uint8 HEIGHT_BAR=2
uint8 WALL=3
uint8 COLORED_OBSTACLE=4

uint8 obstacle_type
float32 confidence   # 多帧启发式置信度，范围 0..1，不代表米制测量可靠性
float32 center_x     # 检测框中心 x / 图像宽度
float32 center_y     # 检测框中心 y / 图像高度
float32 width        # 检测框宽度 / 图像宽度
float32 height       # 检测框高度 / 图像高度
