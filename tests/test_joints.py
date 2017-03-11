############
# Standard #
############
import math

###############
# Third Party #
###############
import ophyd
import pytest

##########
# Module #
##########
from detrot  import ConeJoint, AngledJoint, Point
from conftest import PseudoMotor


@pytest.fixture(scope='module')
def pseudo_cone():
    angled = ConeJoint(slide  = PseudoMotor(5),
                       lift   = PseudoMotor(10),
                       offset = Point(1,2,3))
    return angled

@pytest.fixture(scope='module')
def pseudo_angle():
    angled = AngledJoint(slide  = PseudoMotor(5),
                         lift   = PseudoMotor(10),
                         offset = Point(1,2,3))
    return angled


def test_cone_joint(pseudo_cone):
    #Test Vertical 
    pseudo_cone.alpha = math.pi/2.
    assert pytest.approx(pseudo_cone.joint.x) == 5
    assert pytest.approx(pseudo_cone.joint.y) == 10

    #Test Horizontal
    pseudo_cone.alpha= 0
    assert pseudo_cone.joint.x == 15
    assert pseudo_cone.joint.y == 0

def test_cone_invert(pseudo_cone):
    #Test 45
    pseudo_cone.alpha = math.pi/4.
    assert pseudo_cone.invert((13.07,9.07))[0] == pytest.approx(5,0.1)
    assert pseudo_cone.invert((13.07,9.07))[1] == pytest.approx(10,0.1)

def test_angle_joint(pseudo_angle):
    #Test Vertical
    pseudo_angle.alpha = math.pi/2.
    assert pytest.approx(pseudo_angle.joint.x) == 5
    assert pytest.approx(pseudo_angle.joint.y) == 10
    assert pytest.approx(pseudo_angle.joint.z) == 0
    
    #Test Horizontal
    pseudo_angle.alpha = 0
    assert pytest.approx(pseudo_angle.joint.x) == 5
    assert pytest.approx(pseudo_angle.joint.y) == 0
    assert pytest.approx(pseudo_angle.joint.z) == -10

    #Test no-slide
    p2 = pseudo_angle.slide
    pseudo_angle.slide = None
    assert pytest.approx(pseudo_angle.joint.x) == 0
    assert pytest.approx(pseudo_angle.joint.y) == 0
    assert pytest.approx(pseudo_angle.joint.z) == -10
    pseudo_angle.slide = p2

def test_angle_invert(pseudo_angle):
    #Test Vertical
    pseudo_angle.alpha = math.pi/2.
    assert pseudo_angle.invert((6,12))[0] == pytest.approx(5,0.1)
    assert pseudo_angle.invert((6,12))[1] == pytest.approx(10,0.1)

    #Test no-slide
    p2 = pseudo_angle.slide
    pseudo_angle.slide = None
    assert pseudo_angle.invert((6,12)) == pytest.approx(10,0.1)
    pseudo_angle.slide = p2

def test_position(pseudo_cone):
    pseudo_cone.alpha= 0
    assert pseudo_cone.position == (16, 2, 3)

    pseudo_cone.alpha = math.pi/2.
    assert pseudo_cone.position.x == pytest.approx(6,0.1)
    assert pseudo_cone.position.y == 12
    assert pseudo_cone.position.z == 3

def test_displacement(pseudo_angle):
    assert pseudo_angle.displacement == (5,10)
    p2 = pseudo_angle.slide
    pseudo_angle.slide = None
    assert pseudo_angle.displacement == 10
    pseudo_angle.slide = p2


def test_set_joint(pseudo_angle):
    #Vertical
    pseudo_angle.alpha = math.pi/2.
    pseudo_angle.set_joint((6,12))
    assert pseudo_angle.displacement[0] == pytest.approx(5,0.1)
    assert pseudo_angle.displacement[1] == pytest.approx(10,0.1)
    
    #Test no-slide
    p2 = pseudo_angle.slide
    pseudo_angle.slide = None
    pseudo_angle.set_joint((6,12)) 
    assert pseudo_angle.displacement == pytest.approx(10,0.1) 
    pseudo_angle.slide = p2


def test_model(pseudo_angle, pseudo_cone):
    model = AngledJoint.model(pseudo_angle)
    assert isinstance(model.slide, ophyd.SoftPositioner)
    assert isinstance(model.lift, ophyd.SoftPositioner)
    assert model.displacement == pseudo_angle.displacement

    #Test no slide
    p2 = pseudo_angle.slide
    model = AngledJoint.model(pseudo_angle)
    assert isinstance(model.slide, ophyd.SoftPositioner)
    assert isinstance(model.lift, ophyd.SoftPositioner)
    assert model.displacement == pseudo_angle.displacement
    pseudo_angle.slide = p2

    #Test cone
    model = ConeJoint.model(pseudo_cone)
    assert isinstance(model.slide, ophyd.SoftPositioner)
    assert isinstance(model.lift, ophyd.SoftPositioner)
    assert model.displacement == pseudo_angle.displacement

def test_stop(pseudo_cone):
    pseudo_cone.stop()
    pseudo_cone.slide.stop_call.method.assert_called_with()
    pseudo_cone.lift.stop_call.method.assert_called_with()

def test_cmp():
    p1 = PseudoMotor(5)
    p2 = PseudoMotor(10)
    assert AngledJoint(p1,p2) == AngledJoint(p1, p2)
