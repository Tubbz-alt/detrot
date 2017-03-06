############
# Standard #
############

###############
# Third Party #
###############
import pytest

##########
# Module #
##########
from pystand import Point

class PseudoMotor:

    def __init__(self, position):
        self.position = position

class PseudoStand: 

    pitch = 0.
    yaw   = 0.
    roll  = 0.

    def __init__(self, point):
        self.cone = point
