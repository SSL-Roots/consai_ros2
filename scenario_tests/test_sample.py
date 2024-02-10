import math
import time

import rcst


# def test_our_kickoff(rcst_comm):
#     rcst_comm.send_empty_world()
#     rcst_comm.send_ball(0, 0)
#     rcst_comm.send_blue_robot(1, -0.5, 0.0, math.radians(0))
#     time.sleep(3)  # Wait for the robots to be placed.

#     rcst_comm.observer.reset()
#     rcst_comm.change_referee_command('STOP', 3.0)
#     rcst_comm.change_referee_command('PREPARE_KICKOFF_BLUE', 3.0)
#     rcst_comm.change_referee_command('NORMAL_START', 5.0)

#     assert rcst_comm.observer.ball_has_been_in_positive_goal() is True

def test_control_disturbance(rcst_comm):
    ball_pos = (4, 0)

    rcst_comm.send_empty_world()
    rcst_comm.send_ball(*ball_pos)
    rcst_comm.send_blue_robot(1, -5, -4, math.radians(0))
    time.sleep(3)  # Wait for the robots to be placed.

    rcst_comm.observer.reset()
    rcst_comm.change_referee_command('FORCE_START', 3.0)

    while True:
        # # invert ball_pos
        # ball_pos = (-ball_pos[0], -ball_pos[1])

        # rcst_comm.send_ball(*ball_pos)
        # time.sleep(3)
        pass