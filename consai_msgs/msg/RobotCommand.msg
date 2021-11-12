# ロボットへの動作指令値

uint32 robot_id
bool team_is_yellow

# m/s
float64 velocity_x
float64 velocity_y
# rad/s
float64 velocity_theta

# 0.0 ~ 1.0
float64 kick_power
bool chip_kick
# 0.0 ~ 1.0
float64 dribble_power