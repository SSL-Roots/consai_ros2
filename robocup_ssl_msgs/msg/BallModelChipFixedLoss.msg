# This msg file is a copy of 'messages_robocup_ssl_geometry.proto'
# https://github.com/RoboCup-SSL/ssl-vision/blob/master/src/shared/proto/messages_robocup_ssl_geometry.proto

# Fixed-Loss model for chipped balls.
# Uses fixed damping factors for xy and z direction per hop.

# Chip kick velocity damping factor in XY direction for the first hop
float64 damping_xy_first_hop
# Chip kick velocity damping factor in XY direction for all following hops
float64 damping_xy_other_hops
# Chip kick velocity damping factor in Z direction for all hops
float64 damping_z