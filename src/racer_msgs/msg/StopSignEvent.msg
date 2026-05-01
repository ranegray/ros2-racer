std_msgs/Header header

# Detection state
uint8 STATE_NONE     = 0   # no actionable stop sign
uint8 STATE_DETECTED = 1   # seen but too far to act on (informational)
uint8 STATE_APPROACH = 2   # within braking distance — controller should act
uint8 state

float32 distance_m
int32 bbox_x
int32 bbox_y
int32 bbox_w
int32 bbox_h
float32 confidence
