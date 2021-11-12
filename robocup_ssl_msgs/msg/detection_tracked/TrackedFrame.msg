# This msg file is a copy of 'messages_robocup_ssl_detection_tracked.proto'
# https://github.com/RoboCup-SSL/ssl-vision/blob/master/src/shared/proto/messages_robocup_ssl_detection_tracked.proto

# A frame that contains all currently tracked objects on the field on all cameras

# Capabilities that a source implementation can have
uint32 CAPABILITY_UNKNOWN = 0
uint32 CAPABILITY_DETECT_FLYING_BALLS = 1
uint32 CAPABILITY_DETECT_MULTIPLE_BALLS = 2
uint32 CAPABILITY_DETECT_KICKED_BALLS = 3

# A monotonous increasing frame counter
uint32 frame_number
# The unix timestamp in [s] of the data
float64 timestamp

# The list of detected balls
# The first ball is the primary one
# Sources may add additional balls based on their capabilities
TrackedBall[] balls
# The list of detected robots of both teams
TrackedRobot[] robots

# Information about a kicked ball, if the ball was kicked by a robot and is still moving
# Note: This field is optional. Some source implementations might not set this at any time
KickedBall[<=1] kicked_ball  # optional

# List of capabilities of the source implementation
uint32[] capabilities