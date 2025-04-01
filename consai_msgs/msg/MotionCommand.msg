
uint8 MODE_NAVI = 0  # desired_poseまでNavigation経由で移動する
uint8 MODE_DIRECT_POSE = 1  # desired_poseまで直接移動する
uint8 MODE_DIRECT_VELOCITY = 2  # desired_velocityをロボットに直接指示する

std_msgs/Header header  # MotionCommandArrayのheaderと同じであること
uint32 robot_id

uint8 mode 0

State2D desired_pose
State2D desired_velocity
NaviOptions navi_options

float64 kick_power
bool chip_kick
float64 dribble_power