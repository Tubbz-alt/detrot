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
from detrot  import ConeJoint, AngledJoint, StandPoint, Point
from conftest import PseudoMotor

@pytest.fixture(scope='function')
def pseudo_cone():
    angled = ConeJoint(slide  = PseudoMotor(5),
                       lift   = PseudoMotor(10),
                       offset = Point(1,2,3))
    return angled

@pytest.fixture(scope='function')
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
    assert pytest.approx(pseudo_angle.joint.z) == 10

    #Test no-slide
    pseudo_angle.slide = None
    assert pytest.approx(pseudo_angle.joint.x) == 0
    assert pytest.approx(pseudo_angle.joint.y) == 0
    assert pytest.approx(pseudo_angle.joint.z) == 10

def test_angle_invert(pseudo_angle):
    #Test Vertical
    pseudo_angle.alpha = math.pi/2.
    assert pseudo_angle.invert((6,12))[0] == pytest.approx(5,0.1)
    assert pseudo_angle.invert((6,12))[1] == pytest.approx(10,0.1)

    #Test no-slide
    pseudo_angle.slide = None
    assert pseudo_angle.invert((6,12)) == pytest.approx(10,0.1)

def test_position(pseudo_cone):
    pseudo_cone.alpha= 0
    assert pseudo_cone.position == (16, 2, 3)

    pseudo_cone.alpha = math.pi/2.
    assert pseudo_cone.position.x == pytest.approx(6,0.1)
    assert pseudo_cone.position.y == 12
    assert pseudo_cone.position.z == 3

def test_displacement(pseudo_angle):
    assert pseudo_angle.displacement == (5,10)
    pseudo_angle.slide = None
    assert pseudo_angle.displacement == 10


def test_set_joint(pseudo_angle):
    #Vertical
    pseudo_angle.alpha = math.pi/2.
    pseudo_angle.set_joint((6,12))
    assert pseudo_angle.displacement[0] == pytest.approx(5,0.1)
    assert pseudo_angle.displacement[1] == pytest.approx(10,0.1)

    #Test no-slide
    pseudo_angle.slide = None
    pseudo_angle.set_joint((6,12)) 
    assert pseudo_angle.displacement == pytest.approx(10,0.1) 

def test_model(pseudo_angle, pseudo_cone):
    model = AngledJoint.model(pseudo_angle)
    assert isinstance(model.slide, ophyd.SoftPositioner)
    assert isinstance(model.lift, ophyd.SoftPositioner)
    assert model.displacement == pseudo_angle.displacement

    #Test no slide
    pseudo_angle.slide = None
    model = AngledJoint.model(pseudo_angle)
    assert model.slide == None
    assert isinstance(model.lift, ophyd.SoftPositioner)
    assert model.displacement == pseudo_angle.displacement

    #Test cone
    model = ConeJoint.model(pseudo_cone)
    assert isinstance(model.slide, ophyd.SoftPositioner)
    assert isinstance(model.lift, ophyd.SoftPositioner)
    assert model.displacement == pseudo_cone.displacement

def test_stop(pseudo_cone):
    pseudo_cone.stop()
    pseudo_cone.slide.stop_call.method.assert_called_with()
    pseudo_cone.lift.stop_call.method.assert_called_with()

def test_cmp():
    p1 = PseudoMotor(5)
    p2 = PseudoMotor(10)
    assert AngledJoint(p1,p2) == AngledJoint(p1, p2)


def test_find_angle(pseudo_stand):
    pseudo_stand.find_angles()
    pseudo_stand.cone.offset = Point(0., 0., 0.)
    pseudo_stand.vee.offset  = Point(10., 0. ,  -20.)
    pseudo_stand.flat.offset = Point(-10.,  0. ,  -20.)
    #No offset should be at rest
    assert pseudo_stand.pitch == 0
    assert pseudo_stand.yaw   == 0
    assert pseudo_stand.roll  == 0

    #15 degree roll
    pseudo_stand.flat.lift.move(-10.)
    pseudo_stand.vee.lift.move(10.)
    pseudo_stand.find_angles()
    assert pseudo_stand.pitch == pytest.approx(0, abs=math.pi/180)
    assert pseudo_stand.yaw   == pytest.approx(0, abs=math.pi/180)
    assert pseudo_stand.roll  == pytest.approx(math.pi/12, abs=math.pi/180)

    #12 degree yaw
    pseudo_stand.vee.slide.move(4.2411312)
    pseudo_stand.vee.lift.move(0)
    pseudo_stand.flat.lift.move(0)
    pseudo_stand.find_angles()
    assert pseudo_stand.pitch == pytest.approx(0, abs=math.pi/180)
    assert pseudo_stand.yaw   == pytest.approx(-math.pi/15, abs=3*math.pi/180)
    assert pseudo_stand.roll  == pytest.approx(0, abs=math.pi/180)

    #10 degree pitch
    pseudo_stand.vee.lift.move(-13.4185)
    pseudo_stand.flat.lift.move(-13.4185)
    pseudo_stand.find_angles()
    assert pseudo_stand.pitch == pytest.approx(-math.pi/18, abs=math.pi/180)
    assert pseudo_stand.yaw   == pytest.approx(-math.pi/15, abs=3*math.pi/180)
    assert pseudo_stand.roll  == pytest.approx(0, abs=math.pi/180)

def test_translate(pseudo_stand):
    pseudo_stand.translate(dx=1,dy=2)
    assert pseudo_stand.cone.position.x  == pytest.approx(2, abs=0.0001)
    assert pseudo_stand.cone.position.y  == pytest.approx(4, abs=0.0001)
    assert pseudo_stand.vee.position.x   == pytest.approx(2, abs=0.0001)
    assert pseudo_stand.vee.position.y   == pytest.approx(4, abs=0.0001)
    assert pseudo_stand.flat.position.y  == pytest.approx(4, abs=0.0001)
    pseudo_stand.translate(dx=4,dy=1)
    assert pseudo_stand.cone.position.x  == pytest.approx(6, abs=0.0001)
    assert pseudo_stand.cone.position.y  == pytest.approx(5, abs=0.0001)
    assert pseudo_stand.vee.position.x   == pytest.approx(6, abs=0.0001)
    assert pseudo_stand.vee.position.y   == pytest.approx(5, abs=0.0001)
    assert pseudo_stand.flat.position.y  == pytest.approx(5, abs=0.0001)
    pseudo_stand.translate(dx=-6,dy=-5)
    assert pseudo_stand.cone.position.x  == pytest.approx(0, abs=0.0001)
    assert pseudo_stand.cone.position.y  == pytest.approx(0, abs=0.0001)
    assert pseudo_stand.vee.position.x   == pytest.approx(0, abs=0.0001)
    assert pseudo_stand.vee.position.y   == pytest.approx(0, abs=0.0001)
    assert pseudo_stand.flat.position.y  == pytest.approx(0, abs=0.0001)

def test_rotate(pseudo_stand):
    pseudo_stand.cone.offset = Point(0, 0, 0)
    pseudo_stand.flat.offset = Point(-10,  0 ,  -20)
    pseudo_stand.vee.offset  = Point(10, 0 ,  -20)
    pseudo_stand.rotate(dpitch=math.pi/180, dyaw=math.pi/60, droll=math.pi/90)
    pseudo_stand.find_angles()
    #Test we found the right angles
    assert pseudo_stand.pitch == pytest.approx(math.pi/180, abs=math.pi/180)
    assert pseudo_stand.yaw   == pytest.approx(math.pi/60,  abs=math.pi/180)
    assert pseudo_stand.roll  == pytest.approx(math.pi/90,  abs=math.pi/180)

    pitch = ((pseudo_stand.flat.position.y
            +pseudo_stand.vee.position.y)/2.
            - pseudo_stand.cone.position.y)/20.

    assert math.tan(pseudo_stand.pitch) == pytest.approx(pitch, abs=0.01)

    pseudo_stand.rotate(dpitch=-math.pi/180, dyaw=-math.pi/60, droll=-math.pi/90)
    pseudo_stand.find_angles()
    assert pseudo_stand.pitch == pytest.approx(0., abs=math.pi/180)
    assert pseudo_stand.yaw   == pytest.approx(0., abs=math.pi/180)
    assert pseudo_stand.roll  == pytest.approx(0., abs=math.pi/180)

def test_align(pseudo_stand):
    #Create offsets
    pseudo_stand.cone.offset = Point(0., 0., 0.)
    pseudo_stand.flat.offset = Point(-10.,  0.,  -20.)
    pseudo_stand.vee.offset  = Point(10., 0.,  -20.)
    pseudo_stand.detector.offset = Point(0., 1., 0.)

    #Points to monitor
    fixed = StandPoint((0.,1.,-1.), pseudo_stand)
    origin = fixed.room_coordinates
    mobile = StandPoint((0.,1.,-150.), pseudo_stand)
    start  = mobile.room_coordinates
    
    pseudo_stand.align(-150.0, dx=2., dy=3., origin=-1.0)
    pseudo_stand.find_angles()
    assert fixed.room_coordinates.x == pytest.approx(origin.x, abs=0.01)
    assert fixed.room_coordinates.y == pytest.approx(origin.y, abs=0.01)
    assert mobile.room_coordinates.x == pytest.approx(start.x+2., abs=0.01)
    assert mobile.room_coordinates.y == pytest.approx(start.y+3., abs=0.01)
    
    pseudo_stand.align(-150.0, dx=-4., dy=6., origin=-1.0)
    pseudo_stand.find_angles()
    assert fixed.room_coordinates.x == pytest.approx(origin.x, abs=0.1)
    assert fixed.room_coordinates.y == pytest.approx(origin.y, abs=0.1)
    assert mobile.room_coordinates.x == pytest.approx(start.x-2., abs=0.1)
    assert mobile.room_coordinates.y == pytest.approx(start.y+9., abs=0.1)

