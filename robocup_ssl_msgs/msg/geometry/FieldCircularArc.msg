# This msg file is a copy of 'messages_robocup_ssl_geometry.proto'
# https://github.com/RoboCup-SSL/ssl-vision/blob/master/src/shared/proto/messages_robocup_ssl_geometry.proto

uint32 UNDEFINED = 0
uint32 CENTER_CIRCLE = 1
uint32 TOP_TOUCH_LINE = 2
uint32 BOTTOM_TOUCH_LINE = 3
uint32 LEFT_GOAL_LINE = 4
uint32 RIGHT_GOAL_LINE = 5
uint32 HALFWAY_LINE = 6
uint32 CENTER_LINE = 7
uint32 LEFT_PENALTY_STRETCH = 8
uint32 RIGHT_PENALTY_STRETCH = 9
uint32 LEFT_FIELD_LEFT_PENALTY_STRETCH = 10
uint32 LEFT_FIELD_RIGHT_PENALTY_STRETCH = 11
uint32 RIGHT_FIELD_LEFT_PENALTY_STRETCH = 12
uint32 RIGHT_FIELD_RIGHT_PENALTY_STRETCH = 13

# Represents a field marking as a circular arc segment represented by center point, a
# start angle, an end angle, and an arc thickness.
string name
Vector2f center
float32 radius
float32 a1
float32 a2
float32 thickness
uint32[<=1] type  # optional