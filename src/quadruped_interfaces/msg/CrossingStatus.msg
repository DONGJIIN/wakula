# Status returned by a vendor SDK or gait controller.
uint8 RUNNING=0
uint8 SUCCEEDED=1
uint8 FAILED=2
uint8 CANCELED=3
uint8 ACCEPTED=0
uint8 PREPARING=1
uint8 EXECUTING=2
uint8 VERIFYING_CONTACT=3
uint8 RECOVERING=4
unique_identifier_msgs/UUID goal_id
uint8 state
uint8 phase
float32 progress
bool contact_verified
string message
