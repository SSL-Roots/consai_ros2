# This msg file is a copy of 'ssl_gc_referee_message.proto'
# https://github.com/RoboCup-SSL/ssl-game-controller/blob/master/proto/ssl_gc_referee_message.proto

# The team's name (empty string if operator has not typed anything).
string name
# The number of goals scored by the team during normal play and overtime.
uint32 score
# The number of red cards issued to the team since the beginning of the game.
uint32 red_cards
# The amount of time (in microseconds) left on each yellow card issued to the team.
# If no yellow cards are issued, this array has no elements.
# Otherwise, times are ordered from smallest to largest.
uint32[] yellow_card_times
# The total number of yellow cards ever issued to the team.
uint32 yellow_cards
# The number of timeouts this team can still call.
# If in a timeout right now, that timeout is excluded.
uint32 timeouts
# The number of microseconds of timeout this team can use.
uint32 timeout_time
# The pattern number of this team's goalkeeper.
uint32 goalkeeper
# The total number of countable fouls that act towards yellow cards
uint32[<=1] foul_counter  # optional
# The number of consecutive ball placement failures of this team
uint32[<=1] ball_placement_failures  # optional
# Indicate if the team is able and allowed to place the ball
bool[<=1] can_place_ball  # optional
# The maximum number of bots allowed on the field based on division and cards
uint32[<=1] max_allowed_bots  # optional
# The team has submitted an intent to substitute one or more robots at the next chance
bool[<=1] bot_substitution_intent  # optional
# Indicate if the team reached the maximum allowed ball placement failures and is thus not allowed to place the ball anymore
bool[<=1] ball_placement_failures_reached  # optional