
uint8 MODE_NAV = 0  # desired_poseまでNavigation経由で移動する
uint8 MODE_DIRECT = 1  # desired_poseまで直接移動する
uint8 MODE_RAW_VELOCITY = 2  # raw_velocityをロボットに直接指示する

std_msgs/Header header
uint8 mode 0

uint32 robot_id
bool team_is_yellow

State2D desired_pose

State2D raw_velocity

float32 kick_power
bool chip_kick
float32 dribble_power