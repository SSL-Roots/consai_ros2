uint32 TARGET_UNKNOWN = 0
uint32 TARGET_BALL = 1
uint32 TARGET_BLUE_ROBOT = 2
uint32 TARGET_YELLOW_ROBOT = 3

uint32 PARAMETER_UNKNOWN = 0
uint32 PARAMETER_X = 1
uint32 PARAMETER_Y = 2
uint32 PARAMETER_THETA = 3
uint32 PARAMETER_LOOK_TO = 4
uint32 PARAMETER_LOOK_FROM = 5

float32 value
uint32[<=1] target  # optional
uint32[<=1] target_id  # optional
uint32[<=1] target_parameter  # optional