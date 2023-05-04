import pytest
import math
import consai_examples.geometry_tools as tool

from consai_msgs.msg import State2D

def test_trans_class():
    
    pose1 = State2D(x=0.0, y=0.0)
    pose2 = State2D(x=0.0, y=1.0)
    trans = tool.Trans(pose1, float(math.pi / 2))
    p_trans = trans.transform(pose2) 
    assert math.isclose(p_trans.x, 1, abs_tol=0.01)
    assert math.isclose(p_trans.y, 0, abs_tol=0.01)

    p = trans.inverted_transform(p_trans) 
    assert math.isclose(p.x, 0, abs_tol=0.01)
    assert math.isclose(p.y, 1, abs_tol=0.01)


    pose1 = State2D(x=0.0, y=0.0)
    pose2 = State2D(x=0.0, y=1.0)
    trans = tool.Trans(pose1, float(math.pi / 4))
    p_trans = trans.transform(pose2) 
    assert math.isclose(p_trans.x, 0.7071, abs_tol=0.01)
    assert math.isclose(p_trans.y, 0.7071, abs_tol=0.01)

    p = trans.inverted_transform(p_trans) 
    assert math.isclose(p.x, 0, abs_tol=0.01)
    assert math.isclose(p.y, 1, abs_tol=0.01)

def test_get_diff_xy():

    pose1 = State2D(x=0.0, y=0.0)
    pose2 = State2D(x=1.0, y=1.0)

    diff_pose = tool.get_diff_xy(pose1, pose2)
    assert math.isclose(diff_pose.x, -1.0, abs_tol=0.01)
    assert math.isclose(diff_pose.y, -1.0, abs_tol=0.01)

    pose1 = State2D(x=-2.0, y=0.0)
    pose2 = State2D(x=0.0, y=3.0)

    diff_pose = tool.get_diff_xy(pose1, pose2)
    assert math.isclose(diff_pose.x, -2.0, abs_tol=0.01)
    assert math.isclose(diff_pose.y, -3.0, abs_tol=0.01)

    pose1 = State2D(x=2.0, y=5.0)
    pose2 = State2D(x=0.0, y=2.0)

    diff_pose = tool.get_diff_xy(pose1, pose2)
    assert math.isclose(diff_pose.x, 2.0, abs_tol=0.01)
    assert math.isclose(diff_pose.y, 3.0, abs_tol=0.01)
    
def test_get_distance():

    pose1 = State2D(x=0.0, y=0.0)
    pose2 = State2D(x=1.0, y=1.0)
    distance = tool.get_distance(pose1, pose2)
    assert math.isclose(distance, 1.414, abs_tol=0.01)
    
    pose1 = State2D(x=-1.0, y=2.0)
    pose2 = State2D(x=1.0, y=1.0)
    distance = tool.get_distance(pose1, pose2)
    assert math.isclose(distance, 2.236, abs_tol=0.01)

def test_get_line_parameter():

    pose1 = State2D(x=0.0, y=0.0)
    pose2 = State2D(x=1.0, y=1.0)
    slope, intercept = tool.get_line_parameter(pose1, pose2)
    assert math.isclose(slope, 1, abs_tol=0.01)
    assert math.isclose(intercept, 0, abs_tol=0.01)

    pose1 = State2D(x=-1.0, y=0.0)
    pose2 = State2D(x=1.0, y=1.0)
    slope, intercept = tool.get_line_parameter(pose1, pose2)
    assert math.isclose(slope, 1/2, abs_tol=0.01)
    assert math.isclose(intercept, 1/2, abs_tol=0.01)

    pose1 = State2D(x=1.0, y=0.0)
    pose2 = State2D(x=-1.0, y=2.0)
    slope, intercept = tool.get_line_parameter(pose1, pose2)
    assert math.isclose(slope, -1, abs_tol=0.01)
    assert math.isclose(intercept, 1, abs_tol=0.01)

def test_get_angle():
    from_pose = State2D(x=0.0, y=0.0)
    to_pose = State2D(x=1.0, y=1.0)
    angle = tool.get_angle(from_pose, to_pose)
    assert math.isclose(angle, float(math.pi / 4), abs_tol=0.01)
    
    from_pose = State2D(x=0.0, y=0.0)
    to_pose = State2D(x=-1.0, y=-1.0)
    angle = tool.get_angle(from_pose, to_pose)
    assert math.isclose(angle, float(-math.pi * 3 / 4), abs_tol=0.01)

def test_angle_normalize():
    angle = tool.angle_normalize(math.pi / 2)
    assert math.isclose(angle, float(math.pi / 2), abs_tol=0.01)

    angle = tool.angle_normalize(-math.pi / 2)
    assert math.isclose(angle, float(-math.pi / 2), abs_tol=0.01)

    angle = tool.angle_normalize(3 * math.pi)
    assert math.isclose(angle, float(math.pi), abs_tol=0.01)
    
    angle = tool.angle_normalize(-3 * math.pi)
    assert math.isclose(angle, float(-math.pi), abs_tol=0.01)

