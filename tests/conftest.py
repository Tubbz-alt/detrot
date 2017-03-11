############
# Standard #
############

###############
# Third Party #
###############
import pytest
from unittest.mock import Mock
##########
# Module #
##########
from detrot import Point

class PseudoMotor:

    stop_call = Mock()

    def __init__(self, position):
        self.position = position
        self.name = 'pseudo'
        self.limits = (0,0)

    def move(self, pos, wait=True):
        self.position = pos
        return True

    def stop(self):
        self.stop_call.method()
        pass

class PseudoStand: 


    def __init__(self, point):
        self.cone  = point
        self.pitch = 0.
        self.yaw   = 0.
        self.roll  = 0.
