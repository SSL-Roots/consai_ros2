# All robots should completely stop moving.
uint32 COMMAND_HALT = 0
# Robots must keep 50 cm from the ball.
uint32 COMMAND_STOP = 1
# A prepared kickoff or penalty may now be taken.
uint32 COMMAND_NORMAL_START = 2
# The ball is dropped and free for either team.
uint32 COMMAND_FORCE_START = 3
# The yellow team may move into kickoff position.
uint32 COMMAND_PREPARE_KICKOFF_YELLOW = 4
# The blue team may move into kickoff position.
uint32 COMMAND_PREPARE_KICKOFF_BLUE = 5
# The yellow team may move into penalty position.
uint32 COMMAND_PREPARE_PENALTY_YELLOW = 6
# The blue team may move into penalty position.
uint32 COMMAND_PREPARE_PENALTY_BLUE = 7
# The yellow team may take a direct free kick.
uint32 COMMAND_DIRECT_FREE_YELLOW = 8
# The blue team may take a direct free kick.
uint32 COMMAND_DIRECT_FREE_BLUE = 9
# The yellow team may take an indirect free kick.
uint32 COMMAND_INDIRECT_FREE_YELLOW = 10
# The blue team may take an indirect free kick.
uint32 COMMAND_INDIRECT_FREE_BLUE = 11
# The yellow team is currently in a timeout.
uint32 COMMAND_TIMEOUT_YELLOW = 12
# The blue team is currently in a timeout.
uint32 COMMAND_TIMEOUT_BLUE = 13
# The yellow team just scored a goal.
# For information only.
# For rules compliance, teams must treat as STOP.
# Deprecated: Use the score field from the team infos instead. That way, you can also detect revoked goals.
uint32 COMMAND_GOAL_YELLOW = 14
# The blue team just scored a goal. See also GOAL_YELLOW.
uint32 COMMAND_GOAL_BLUE = 15
# Equivalent to STOP, but the yellow team must pick up the ball and
# drop it in the Designated Position.
uint32 COMMAND_BALL_PLACEMENT_YELLOW = 16
# Equivalent to STOP, but the blue team must pick up the ball and drop
# it in the Designated Position.
uint32 COMMAND_BALL_PLACEMENT_BLUE = 17

uint32 raw_command
State2D designated_position
bool is_placement
bool is_inplay
