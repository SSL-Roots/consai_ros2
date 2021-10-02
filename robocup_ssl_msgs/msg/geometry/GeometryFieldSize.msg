# This msg file is a copy of 'messages_robocup_ssl_geometry.proto'
# https://github.com/RoboCup-SSL/ssl-vision/blob/master/src/shared/proto/messages_robocup_ssl_geometry.proto

int32 field_length
int32 field_width
int32 goal_width
int32 goal_depth
int32 boundary_width
FieldLineSegment[] field_lines
FieldCircularArc[] field_arcs
int32[<=1] penalty_area_depth  # optional
int32[<=1] penalty_area_width  # optional