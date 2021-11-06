# This msg file is a copy of 'ssl_gc_referee_message.proto'
# https://github.com/RoboCup-SSL/ssl-game-controller/blob/master/proto/ssl_gc_referee_message.proto

# The first half is about to start.
# A kickoff is called within this stage.
# This stage ends with the NORMAL_START.
uint32 STAGE_NORMAL_FIRST_HALF_PRE = 0
# The first half of the normal game, before half time.
uint32 STAGE_NORMAL_FIRST_HALF = 1
# Half time between first and second halves.
uint32 STAGE_NORMAL_HALF_TIME = 2
# The second half is about to start.
# A kickoff is called within this stage.
# This stage ends with the NORMAL_START.
uint32 STAGE_NORMAL_SECOND_HALF_PRE = 3
# The second half of the normal game, after half time.
uint32 STAGE_NORMAL_SECOND_HALF = 4
# The break before extra time.
uint32 STAGE_EXTRA_TIME_BREAK = 5
# The first half of extra time is about to start.
# A kickoff is called within this stage.
# This stage ends with the NORMAL_START.
uint32 STAGE_EXTRA_FIRST_HALF_PRE = 6
# The first half of extra time.
uint32 STAGE_EXTRA_FIRST_HALF = 7
# Half time between first and second extra halves.
uint32 STAGE_EXTRA_HALF_TIME = 8
# The second half of extra time is about to start.
# A kickoff is called within this stage.
# This stage ends with the NORMAL_START.
uint32 STAGE_EXTRA_SECOND_HALF_PRE = 9
# The second half of extra time.
uint32 STAGE_EXTRA_SECOND_HALF = 10
# The break before penalty shootout.
uint32 STAGE_PENALTY_SHOOTOUT_BREAK = 11
# The penalty shootout.
uint32 STAGE_PENALTY_SHOOTOUT = 12
# The game is over.
uint32 STAGE_POST_GAME = 13

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

uint64 packet_timestamp
uint32 stage
int32[<=1] stage_time_left  # optional
uint32 command
uint32 command_counter
uint64 command_timestamp
TeamInfo yellow
TeamInfo blue
Point[<=1] designated_position  # optional
bool[<=1] blue_team_on_positive_half  # optional
uint32[<=1] next_command  # optional
# GameEvent[] game_events
# GameEventProposalGroup[] game_event_proposals
int32[<=1] current_action_time_remaining  # optional