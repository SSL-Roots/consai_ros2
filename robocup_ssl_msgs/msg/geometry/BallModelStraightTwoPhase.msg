# This msg file is a copy of 'messages_robocup_ssl_geometry.proto'
# https://github.com/RoboCup-SSL/ssl-vision/blob/master/src/shared/proto/messages_robocup_ssl_geometry.proto

# Two-Phase model for straight-kicked balls.
# There are two phases with different accelerations during the ball kicks:
# 1. Sliding
# 2. Rolling
# The full model is described in the TDP of ER-Force from 2016, which can be found here:
# https://ssl.robocup.org/wp-content/uploads/2019/01/2016_ETDP_ER-Force.pdf

# Ball sliding acceleration [m/s^2] (should be negative)
float64 acc_slide
# Ball rolling acceleration [m/s^2] (should be negative)
float64 acc_roll
# Fraction of the initial velocity where the ball starts to roll
float64 k_switch