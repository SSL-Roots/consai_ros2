# ロボットへの動作指令値

uint32 robot_id
bool team_is_yellow

# m/s
float32 velocity_x
float32 velocity_y
# rad/s
float32 velocity_angular

# 0.0 ~ 1.0
float32 kick_power
bool chip_kick
# 0.0 ~ 1.0
float32 dribble_power