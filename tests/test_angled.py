############
# Standard #
############
import math

###############
# Third Party #
###############
import pytest

##########
# Module #
##########
from detrot  import AngledJoint, Point
from conftest import PseudoMotor


@pytest.fixture(scope='module')
def pseudo_angled():
    angled = AngledJoint(slide  = PseudoMotor(5),
                         lift   = PseudoMotor(10),
                         offset = Point(1,2,3))
    return angled

def test_joint(pseudo_angled):
    #Test Vertical 
    pseudo_angled.alpha = math.pi/2.
    assert pytest.approx(pseudo_angled.joint.x) == 5
    assert pytest.approx(pseudo_angled.joint.y) == 10

    #Test Horizontal
    pseudo_angled.alpha= 0
    assert pseudo_angled.joint.x == 15
    assert pseudo_angled.joint.y == 0

def test_invert(pseudo_angled):
    #Test 45
    pseudo_angled.alpha = math.pi/4.
    assert pytest.approx(pseudo_angled.invert((12.07,7.07))[0]) == pytest.approx(5,0.1)
    assert pytest.approx(pseudo_angled.invert((12.07,7.07))[1]) == pytest.approx(10,0.1)

def test_position(pseudo_angled):
    pseudo_angled.alpha= 0
    assert pseudo_angled.position == (16, 2, 3)
    
    pseudo_angled.alpha = math.pi/2.
    assert pseudo_angled.position.x == pytest.approx(6,0.1)
    assert pseudo_angled.position.y == 12
    assert pseudo_angled.position.z == 3

def test_cmp():
    p1 = PseudoMotor(5)
    p2 = PseudoMotor(10)
    assert AngledJoint(p1,p2) == AngledJoint(p1, p2)
