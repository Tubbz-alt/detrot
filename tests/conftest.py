############
# Standard #
############
import logging
###############
# Third Party #
###############
import pytest
from unittest.mock import Mock
##########
# Module #
##########
from detrot import Detector, Point, ConeJoint, AngledJoint, Stand

#Setup Logger in DEBUG mode
logging.getLogger('detrot').setLevel(logging.DEBUG)
#logging.basicConfig()

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


@pytest.fixture(scope='function')
def pseudo_stand():
    cone = ConeJoint(slide = PseudoMotor(0),
                     lift   = PseudoMotor(0),
                     offset = Point(1,2,3))

    flat = AngledJoint(lift   = PseudoMotor(0),
                       offset = Point(1,2,3))

    vee = AngledJoint(slide  = PseudoMotor(0),
                      lift    = PseudoMotor(0),
                      offset  = Point(1,2,3))

    det = Detector(slide=PseudoMotor(0),
                   offset = Point(0,1,0))
    return Stand(cone=cone, flat=flat, vee=vee, det=det)
