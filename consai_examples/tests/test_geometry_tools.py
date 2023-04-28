import pytest
import math
import consai_examples.geometry_tools as tool

def test_trans_class():
    
    trans = tool.Trans([0, 0], float(math.pi / 2))
    p_trans = trans.transform([0, 1]) 
    assert math.isclose(p_trans[0], 1, abs_tol=0.01)
    assert math.isclose(p_trans[1], 0, abs_tol=0.01)

    p = trans.inverted_transform(p_trans) 
    assert math.isclose(p[0], 0, abs_tol=0.01)
    assert math.isclose(p[1], 1, abs_tol=0.01)

    trans = tool.Trans([0, 0], float(math.pi / 4))
    p_trans = trans.transform([0, 1]) 
    assert math.isclose(p_trans[0], 0.7071, abs_tol=0.01)
    assert math.isclose(p_trans[1], 0.7071, abs_tol=0.01)

    p = trans.inverted_transform(p_trans) 
    assert math.isclose(p[0], 0, abs_tol=0.01)
    assert math.isclose(p[1], 1, abs_tol=0.01)

def test_get_distance():
    distance = tool.get_distance([0, 0], [1, 1])
    assert math.isclose(distance, 1.414, abs_tol=0.01)
    
    distance = tool.get_distance([-1, 2], [1, 1])
    assert math.isclose(distance, 2.236, abs_tol=0.01)

def test_get_angle():
    angle = tool.get_angle([0, 0], [1, 1])
    assert math.isclose(angle, float(math.pi / 4), abs_tol=0.01)
    
    angle = tool.get_angle([0, 0], [-1, -1])
    assert math.isclose(angle, float(-math.pi * 3 / 4), abs_tol=0.01)

