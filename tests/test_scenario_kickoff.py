
import math
import time


def test_our_kickoff(rcst_comm):
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(0, 0)
    rcst_comm.send_blue_robot(1, -0.5, 0.0, math.radians(0))
    time.sleep(3)  # Wait for the robots to be placed.

    rcst_comm.observer.reset()
    rcst_comm.change_referee_command('STOP', 3.0)
    rcst_comm.change_referee_command('PREPARE_KICKOFF_BLUE', 3.0)
    rcst_comm.change_referee_command('NORMAL_START', 5.0)

    assert rcst_comm.observer.ball_has_been_in_positive_goal() is True


def test_their_kickoff(rcst_comm):
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(0, 0)
    rcst_comm.send_blue_robot(0, -5.5, 0.0, math.radians(0))
    rcst_comm.send_yellow_robot(0, 0.1, 0.0, math.radians(180))
    time.sleep(1)  # Wait for the robots to be placed.

    rcst_comm.observer.reset()
    rcst_comm.change_referee_command('STOP', 3.0)
    rcst_comm.change_referee_command('PREPARE_KICKOFF_YELLOW', 3.0)
    rcst_comm.change_referee_command('NORMAL_START', 1.0)

    # Shoot to our goal.
    rcst_comm.send_ball(0, 0, -6.0, 0.5)
    time.sleep(5)

    assert rcst_comm.observer.ball_has_been_in_negative_goal() is False