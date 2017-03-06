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
from conftest import PseudoStand
from pystand  import Point, StandPoint


@pytest.fixture(scope='module')
def pseudo_stand():
    stnd = PseudoStand(Point(4,5,3))
    return StandPoint(Point(1,2,3), stnd)


def test_stand_coordinate(pseudo_stand):
    assert pseudo_stand.stand_coordinates.x == 5
    assert pseudo_stand.stand_coordinates.y == 7
    assert pseudo_stand.stand_coordinates.z == 3


def test_room_coordinates(pseudo_stand):

    #No rotation should match stand coordinates
    pseudo_stand.pitch = 0
    pseudo_stand.yaw   = 0
    pseudo_stand.roll  = 0

    assert pseudo_stand.room_coordinates.x == 5
    assert pseudo_stand.room_coordinates.y == 7
    assert pseudo_stand.room_coordinates.z == 3


