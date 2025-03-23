
uint8 MODE_NAVI = 0  # desired_poseまでNavigation経由で移動する
uint8 MODE_DIRECT = 1  # desired_poseまで直接移動する
uint8 MODE_RAW_VELOCITY = 2  # raw_velocityをロボットに直接指示する

std_msgs/Header header  # MotionCommandArrayのheaderと同じであること
uint8 mode 0

uint32 robot_id

State2D desired_pose

State2D raw_velocity

float64 kick_power
bool chip_kick
float64 dribble_power