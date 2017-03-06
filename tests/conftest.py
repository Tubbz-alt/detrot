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
from detrot import Point

class PseudoMotor:

    def __init__(self, position):
        self.position = position

class PseudoStand: 


    def __init__(self, point):
        self.cone  = point
        self.pitch = 0.
        self.yaw   = 0.
        self.roll  = 0.
