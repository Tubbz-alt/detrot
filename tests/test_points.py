############
# Standard #
############
import math
import logging

###############
# Third Party #
###############
import pytest

##########
# Module #
##########
from detrot  import Point, StandPoint


@pytest.fixture(scope='function')
def coord_stand(pseudo_stand):
    pseudo_stand.cone.offset = Point(0,0,3)
    pseudo_stand.cone.set_joint((4,5))
    return StandPoint(Point(1,2,3), pseudo_stand)


def test_stand_coordinate(coord_stand):
    assert coord_stand.stand_coordinates.x == 5
    assert coord_stand.stand_coordinates.y == 7
    assert coord_stand.stand_coordinates.z == 3


def test_room_coordinates(coord_stand):

    #No rotation should match stand coordinates
    coord_stand.pitch = 0
    coord_stand.yaw   = 0
    coord_stand.roll  = 0

    assert coord_stand.room_coordinates.x == 5
    assert coord_stand.room_coordinates.y == 7
    assert coord_stand.room_coordinates.z == 3

    #90 degree pitch
    coord_stand.stand.pitch = math.pi/2.
    coord_stand.stand.yaw   = 0
    coord_stand.stand.roll  = 0

    assert coord_stand.room_coordinates.x == 5
    assert coord_stand.room_coordinates.y == pytest.approx(-3,0.001)
    assert coord_stand.room_coordinates.z == 7

    #90 degree yaw
    coord_stand.stand.pitch = 0
    coord_stand.stand.yaw   = math.pi/2.
    coord_stand.stand.roll  = 0

    assert coord_stand.room_coordinates.x == pytest.approx(3, 0.001)
    assert coord_stand.room_coordinates.y == 7
    assert coord_stand.room_coordinates.z == -5

    #90 degree roll
    coord_stand.stand.pitch = 0
    coord_stand.stand.yaw   = 0
    coord_stand.stand.roll  = math.pi/2.

    assert coord_stand.room_coordinates.x == -7
    assert coord_stand.room_coordinates.y == 5
    assert coord_stand.room_coordinates.z == 3

