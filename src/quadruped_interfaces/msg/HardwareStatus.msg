# 厂商 SDK/ros2_control 适配器必须周期发布的硬件健康心跳。
std_msgs/Header header
uint8 DISCONNECTED=0
uint8 STANDBY=1
uint8 ENABLED=2
uint8 FAULT=3
uint8 state
bool command_ready
bool actuators_enabled
uint32 fault_code
string fault_text
