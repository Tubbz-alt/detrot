"""
"""
############
# Standard #
############
import logging
from math import cos, tan, sin

###############
# Third Party #
###############


##########
# Module #
##########
from .points import Point
logger = logging.getLogger(__name__)


class AngledJoint(object):
    """
    A class representing two angled joint motors as a single axis

    Parameters
    ----------
    slide : ``ophyd.EpicsMotor``
        Horizontal Motor

    lift  : ``ophyd.EpicsMotor``, optional
        Vertical Motor

    offset : `.Point`
        The position of the joint when all motors are at nominal zero

    Attributes
    ----------
    alpha : float
        The angle of the tilted motor in radians
    """
    alpha =0.261799387

    def __init__(self, slide=None, lift=None, offset=None):

        self.slide  = slide
        self.lift   = lift
        self.offset = offset


    @property
    def displacement(self):
        """
        Displacement of the angled motor from the nominal zero
        """
        if not self.lift:
            return Point(self.slide.position, 0., 0.)
        else:
            return Point(self.slide.position, self.lift.position, 0.)


    @property
    def position(self):
        """
        Position of the ball joint in stand coordinates
        """
        return Point(self.joint[0]+self.offset.x,
                     self.joint[1]+self.offset.y,
                     self.offset.z)


    @property
    def joint(self):
        """
        Displacement of the ball joint from nominal zero
        """
        return Point(self.displacement.y*cos(self.alpha) + self.displacement.x,
                     self.displacement.y*sin(self.alpha),
                     0.)


    def invert(self, point):
        """
        Invert the matrix to find the neccesary motor positions to put the
        joint at a specific displacement (x,y) coordinate in rest coordinates

        Parameters
        -----------
        point : tuple or :class:`.Point`
            The desired x,y coordinates of the joint

        Returns
        --------
        position : tuple
            A tuple of the neccesary positions of the form (slide, lift)

        Raises
        ------
        ValueError :
            If the given position is not possible for the joint
        """
        #Convert to Point object
        if not isinstance(point, Point):
            point = Point(*point, 0.)

        #Find displacement
        dis = Point(point.x - self.offset.x,
                    point.y - self.offset.y)

        if not self.lift and point.y != 0.:
            raise ValueError("Unable to reach desired position {},"
                             "because this joint has no lift"
                             "".format(point[1]))

        return (dis.x-dis.y/tan(self.alpha),
                dis.y/sin(self.alpha))


    def __repr__(self):
        return "AngledJoint at {!r}".format(self.joint)


    def __eq__(self, other):
        if not isinstance(other, AngledJoint):
            raise TypeError("Can not compare AngledJoint "
                            "and {}".format(type(other)))
        return self.slide, self.lift == other.slide, other.lift
