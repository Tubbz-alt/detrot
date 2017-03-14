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
from detrot import ConeJoint, AngledJoint, StandPoint, Point

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
