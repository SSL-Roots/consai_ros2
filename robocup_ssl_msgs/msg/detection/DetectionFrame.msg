# This msg file is a copy of 'messages_robocup_ssl_detection.proto'
# https://github.com/RoboCup-SSL/ssl-vision/blob/master/src/shared/proto/messages_robocup_ssl_detection.proto

uint32 frame_number
float64 t_capture
float64 t_sent
uint32 camera_id
DetectionBall[] balls
DetectionRobot[] robots_yellow
DetectionRobot[] robots_blue