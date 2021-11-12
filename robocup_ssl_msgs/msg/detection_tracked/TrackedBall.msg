# This msg file is a copy of 'messages_robocup_ssl_detection_tracked.proto'
# https://github.com/RoboCup-SSL/ssl-vision/blob/master/src/shared/proto/messages_robocup_ssl_detection_tracked.proto

# A single tracked ball

# The position (x, y, height) [m] in the ssl-vision coordinate system
Vector3 pos

# The velocity [m/s] in the ssl-vision coordinate system
Vector3[<=1] vel  # optional

# The visibility of the ball
# A value between 0 (not visible) and 1 (visible)
# The exact implementation depends on the source software
float32[<=1] visibility  #optional