# This msg file is a copy of 'messages_robocup_ssl_geometry.proto'
# https://github.com/RoboCup-SSL/ssl-vision/blob/master/src/shared/proto/messages_robocup_ssl_geometry.proto

uint32 camera_id
float32 focal_length
float32 principal_point_x
float32 principal_point_y
float32 distortion
float32 q0
float32 q1
float32 q2
float32 q3
float32 tx
float32 ty
float32 tz
float32[<=1] derived_camera_world_tx  # optional
float32[<=1] derived_camera_world_ty  # optional
float32[<=1] derived_camera_world_tz  # optional
uint32[<=1] pixel_image_width  # optional
uint32[<=1] pixel_image_height  # optional