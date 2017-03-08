"""
Part of the complexity of rotating the detector stands is manipulating the
three unique joints at each corner. All three have a ``lift`` motor which
pushes the stand upwards and horizontally. In the case of the cone joint at
front of the stand, moving upwards also neccesitates a motion in the positive X
direction, while the back two joints both introduce displacement in negative z  

Finally, both the vee and cone joints have ``slide `` motors. These help adjust
the X motion of the chamber to counter act the parasitic motions of the lift.
In order represent all three of these joints, we have the :class:`.AngledJoint`
and :class:`.ConeJoint` class. Each interperts the position of their motors and
the given ``offset`` to find the rest frame position of the joint. They
also each have an :meth:`.invert` method so the the operation can be reversed.
"""
############
# Standard #
############
import logging
from math import pi, cos, tan, sin

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
    A class representing two angled joint motors as a single axis.

    This class is used for both the vee and flat joints, with the
    former leaving the ``slide`` parameter blank

    Parameters
    ----------
    lift  : ``ophyd.EpicsMotor``
        Vertical Motor

    slide : ``ophyd.EpicsMotor``, optional
        Horizontal Motor

    offset : tuple or :class:`.Point`
        The (x,y,z) position of the joint when all motors are at nominal zero

    Attributes
    ----------
    alpha : float
        The angle of the tilted motor in radians
    """
    alpha = pi/12.

    def __init__(self, lift=None, slide=None, offset=None):

        self.slide  = slide
        self.lift   = lift
        self.offset = offset


    @property
    def displacement(self):
        """
        Displacement of the joint motors from the nominal zero
        """
        if not self.slide:
            return self.lift.position
        else:
            return (self.slide.position, self.lift.position)


    @property
    def position(self):
        """
        Position of the ball joint in rest coordinates as a :class:`.Point`
        """
        return Point(self.joint.x+self.offset.x,
                     self.joint.y+self.offset.y,
                     self.joint.z+self.offset.z)


    @property
    def joint(self):
        """
        Displacement of the ball joint from nominal zero as :class:`.Point`
        """
        if not self.slide:
            slide, lift = 0., self.displacement

        else:
            slide, lift = self.displacement

        return Point(slide, lift*sin(self.alpha), -lift*cos(self.alpha))


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
            point = Point(*point, 0)

        #Find displacement
        dis = Point(point.x - self.offset.x,
                   (point.y - self.offset.y)/sin(self.alpha),
                    0.)

        if not self.slide:
            return dis.y

        else:
            return (dis.x, dis.y)


    def __repr__(self):
        return "AngledJoint at {!r}".format(self.joint)


    def __eq__(self, other):
        if not isinstance(other, AngledJoint):
            raise TypeError("Can not compare AngledJoint "
                            "and {}".format(type(other)))
        return self.slide, self.lift == other.slide, other.lift


class ConeJoint(AngledJoint):
    """
    Class to represent the stand Cone joint
    """
    @property
    def joint(self):
        """
        Displacement of the cone joint from nominal zero as a :class:`.Point`
        """
        return Point(self.displacement[1]*cos(self.alpha) + self.displacement[0],
                     self.displacement[1]*sin(self.alpha),
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
                    point.y - self.offset.y,
                    0.)

        return (dis.x-dis.y/tan(self.alpha),
                dis.y/sin(self.alpha))


