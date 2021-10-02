# This msg file is a copy of 'messages_robocup_ssl_detection_tracked.proto'
# https://github.com/RoboCup-SSL/ssl-vision/blob/master/src/shared/proto/messages_robocup_ssl_detection_tracked.proto

# A ball kicked by a robot, including predictions when the ball will come to a stop

# The initial position [m] from which the ball was kicked
Vector2 pos
# The initial velocity [m/s] with which the ball was kicked
Vector3 vel
# The unix timestamp [s] when the kick was performed
float64 start_timestamp

# The predicted unix timestamp [s] when the ball comes to a stop
float64[<=1] stop_timestamp  # optional
# The predicted position [m] at which the ball will come to a stop
Vector2[<=1] stop_pos  # optional

# The robot that kicked the ball
RobotId[<=1] robot_id  # optional