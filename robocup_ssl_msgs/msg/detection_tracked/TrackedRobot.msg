# This msg file is a copy of 'messages_robocup_ssl_detection_tracked.proto'
# https://github.com/RoboCup-SSL/ssl-vision/blob/master/src/shared/proto/messages_robocup_ssl_detection_tracked.proto

# A single tracked robot

RobotId robot_id

# The position [m] in the ssl-vision coordinate system
Vector2 pos
# The orientation [rad] in the ssl-vision coordinate system
float32 orientation

# The velocity [m/s] in the ssl-vision coordinate system
Vector2[<=1] vel  # optional
# The angular velocity [rad/s] in the ssl-vision coordinate system
float32[<=1] vel_angular  # optional

# The visibility of the robot
# A value between 0 (not visible) and 1 (visible)
# The exact implementation depends on the source software
float32[<=1] visibility  # optional