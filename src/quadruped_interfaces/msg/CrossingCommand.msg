# Command sent by the Action gateway to a vendor SDK or gait controller.
uint8 START=1
uint8 CANCEL=2
uint8 STEP=1
uint8 CLIMB=2
uint8 LOW_PROFILE=3
uint8 command
unique_identifier_msgs/UUID goal_id
uint8 mode
float32 obstacle_height
float32 obstacle_distance
float32 speed_scale
float32 timeout
