"""
Part of the complexity of rotating the detector stands is manipulating the
three unique joints at each corner. All three have a ``lift`` motor which
pushes the stand upwards and horizontally. In the case of the cone joint
underneath the stand, moving upwards also introduces motion in the positive X
direction, while the other two joints both displace the stand in Z.

In addition to the lifting motors, both the vee and cone joints have ``slide``
motors. These help adjust the X motion of the chamber to counter act the
parasitic motions of the lift.  In order represent all three of these joints,
we have the :class:`.AngledJoint` and :class:`.ConeJoint` classes. Each
interprets the position of their motors and the given ``offset`` to find the
rest frame position of the joint. They also each have an :meth:`.invert` method
so the the operation can be reversed.
"""
############
# Standard #
############
import copy
import logging
from math import pi, cos, tan, sin, fabs

###############
# Third Party #
###############
from ophyd import SoftPositioner

##########
# Module #
##########
from .points import StandPoint, Point

logger = logging.getLogger(__name__)

class AngledJoint:
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

        if not isinstance(offset, Point):
            try:
                offset = Point(*offset)
            except TypeError:
                logger.warning("Found invalid offset type {} ..."
                               "".format(type(offset)))
                offset = Point(0,0,0)

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

        return Point(slide, lift*sin(self.alpha), lift*cos(self.alpha))


    def invert(self, point, offset=True):
        """
        Invert the matrix to find the neccesary motor positions to put the
        joint at a specific displacement (x,y) in rest coordinates

        Parameters
        -----------
        point : tuple or :class:`.Point`
            The desired x,y coordinates of the joint
        
        offset : bool, optional
            Subtract the offset before calculating the motor positions. If set
            to False, just the displacement of the motors should be entered

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
        if offset:
            point = Point(point.x - self.offset.x,
                          point.y - self.offset.y,
                          0.)

        if not self.slide:
            return point.y/sin(self.alpha)

        else:
            return (point.x, point.y/sin(self.alpha))


    def set_displacement(self, displacement, relative=False):
        """
        Set the displacements of the lift and/or slide stages of the joint

        Parameters
        ----------
        displacement : float or tuple
            Either a single float if there is no slide, otherwise a tuple of
            (slide, lift)

        relative : bool, optional
            Choice of relative or absolute move

        Returns
        -------
        status : ``ophyd.Status``
            Status of the requested move, or list of both requested moves
        """

        if not self.slide:
            if relative:
                displacement += self.lift.position

            logger.info("Setting motors to {} from nominal "
                        " zero".format(displacement))

            return self.lift.move(displacement, wait=False)

        else:
            if relative:
                displacement = (displacement[0]+self.slide.position,
                                displacement[1]+self.lift.position)
            logger.info("Setting motors to {} from nominal "
                        " zero".format(displacement))
            return [self.slide.move(displacement[0], wait=False),
                    self.lift.move(displacement[1],  wait=False)],


    def set_joint(self, point, offset=True):
        """
        Set the joint to a specific point in rest frame coordinates

        This is the identical to setting the displacement of the motors to the
        result of the :meth:`.invert` method

        Parameters
        ----------
        point : tuple or :class:`.Point`
            The desired x,y coordinates of the joint

        offset : bool, optional
            Subtract the offset before calculating the motor positions. If set
            to False, just the displacement of the motors should be entered

        Returns
        -------
        status : ``ophyd.Status``
            Status of the requested move, or list of both requested moves

        See Also
        --------
        :meth:`.invert`, :meth:`.set_displacement`
        """
        return self.set_displacement(self.invert(point, offset=offset))


    def stop(self):
        """
        Stop all motion
        """
        logger.info("Stopping joint")
        if self.slide:
            self.slide.stop()
        self.lift.stop()


    @classmethod
    def model(cls, joint):
        """
        Create an identical joint but with the actual motors replaced by
        ``SoftPositioner`` instances

        Parameters
        ----------
        joint : :class:`.Joint`
            Joint to be modeled

        Returns
        -------
        joint : :class:`.Joint`
            Copied joint with ``SoftPositioner`` slide and lift
        """
        #Copy stand
        joint = copy.copy(joint)

        #Method to duplicate
        def duplicate(mtr):
            if not mtr:
                return None

            soft = SoftPositioner(name=mtr.name, limits=mtr.limits)
            soft.move(mtr.position)
            return soft

        joint.slide, joint.lift = duplicate(joint.slide), duplicate(joint.lift)
        return joint


    def __copy__(self):
        joint = AngledJoint(lift   = self.lift,
                            slide  = self.slide,
                            offset = self.offset)
        joint.alpha = self.alpha
        return joint


    def __repr__(self):
        return "AngledJoint at {!r} (".format(self.joint)


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

    def invert(self, point, offset=True):
        """
        Invert the matrix to find the neccesary motor positions to put the
        joint at a specific displacement (x,y) coordinate in rest coordinates

        Parameters
        -----------
        point : tuple or :class:`.Point`
            The desired x,y coordinates of the joint

        offset : bool, optional
            Subtract the offset before calculating the motor positions. If set
            to False, just the displacement of the motors should be entered

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
        if offset:
            point = Point(point.x - self.offset.x,
                          point.y - self.offset.y,
                        0.)

        return (point.x-point.y/tan(self.alpha),
                point.y/sin(self.alpha))



    def __repr__(self):
        return "ConeJoint at {!r}".format(self.joint)


    def __copy__(self):
        joint = ConeJoint(lift   = self.lift,
                          slide  = self.slide,
                          offset = self.offset)
        joint.alpha = self.alpha
        return joint


class Detector:
    """
    The internal slide of the Detector

    Parameters
    ----------
    slide : ``ophyd.EpicsMotor``
        The internal Z motion

    offset : :class:`.Point` or tuple
        The distance from the cone to the zero point of the motor
    """
    def __init__(self, slide, offset=None):
        self.slide  = slide
        self.offset = offset

        if not isinstance(offset, Point):
            try:
                offset = Point(*offset)
            except TypeError:
                logger.warning("Found invalid offset type {} ..."
                               "".format(type(offset)))
                offset = Point(0,0,0)

        self.offset = offset


    @property
    def displacement(self):
        """
        Displacement of the detector motor from the nominal zero
        """
        return self.slide.position


    @property
    def position(self):
        """
        Position of the ball joint in rest coordinates as a :class:`.Point`
        """
        return Point(self.offset.x,
                     self.offset.y,
                     self.displacement+self.offset.z)


    def set_displacement(self, pos, relative=False):
        """
        Set the displacement of the Detector motor

        Parameters
        ----------
        displacement : float or tuple
            Desired position or move of the detector

        relative : bool, optional
            Choice of relative or absolute move

        Returns
        -------
        status : ``ophyd.Status``
            Status of the requested move
        """
        if relative:
            displacement += self.slide.position

        logger.info("setting motors to {} from nominal "
                " zero".format(displacement))

        return self.lift.move(displacement, wait=False)


    @classmethod
    def model(cls, det):
        """
        Create a model of the detector stage with a ``ophyd.SoftPositioner`` as
        the slide

        Parameters
        ----------
        det : :class:`.Detector`
            Detector stage to model
        """
        model = copy.copy(det)
        model.slide = SoftPositioner(name   = det.slide.name,
                                     limits = det.slide.limits)
        model.slide.move(det.slide.position)
        return model

    def __copy__(self):
        det = Detector(slide  = self.slide,
                       offset = self.offset)
        return det

    def __repr__(self):
        return "Detector at {!r}".format(self.position)


