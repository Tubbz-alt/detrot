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

logger = logging.getLogger(__name__)


class AngledJoint(object):
    """
    A class representing two angled joint motors as a single axis

    Parameters
    ----------
    slide : ``ophyd.EpicsMotor``
        Horizontal Motor

    lift  : ``ophyd.EpicsMotor``
        Vertical Motor

    x0

    y0

    Attributes
    ----------
    alpha : float
        The angle of the tilted motor in radians
    """
    alpha =0.261799387

    def __init__(self, slide=None, lift=None,
                 x0=0., y0=0.)
        self.slide = slide
        self.lift  = lift
        self.x0    = x0
        self.y0    = y0


    @property
    def position(self):
        """
        Position of the two motor axes
        """
        return (self.slide.position, self.lift.position)


    @property
    def joint(self):
        """
        Position of the ball joint
        """
        (slide, lift) = self.position
        return (lift*cos(self.alpha) + slide,
                lift*sin(self.alpha))



    def invert(self, coordinate):
        """
        Invert the matrix to find the neccesary motor positions to put the
        joint at a specific (x,y) coordinate

        Parameters
        -----------
        coordinate : tuple
            The desired x,y coordinates of the joint

        Returns
        --------
        position : tuple
            A tuple of the neccesary positions of the form (slide, lift)
        """
        return (coordinate[0]-coordinate[1]/tan(self.alpha),
                coordinate[0]/sin(self.alpha)


    def __repr__(self):
        return "AngledJoint at {}".format(self.joint)


    def __eq__(self, other):
        if not isinstance(other, AngledJoint):
            raise TypeError("Can not compare AngledJoint "
                            "and {}".format(type(other)))
        return self.slide, self.lift == other.slide, other.lift
