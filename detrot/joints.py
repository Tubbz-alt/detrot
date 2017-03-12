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


    def set_displacement(self, displacement):
        """
        Set the displacements of the lift and/or slide stages of the joint

        Parameters
        ----------
        displacement : float or tuple
            Either a single float if there is no slide, otherwise a tuple of
            (slide, lift)

        Returns
        -------
        status : ``ophyd.Status``
            Status of the requested move, or list of both requested moves
        """
        logger.info("Setting motors to {} from nominal "
                    " zero".format(displacement))
        if not self.slide:
            return self.lift.move(displacement, wait=False)

        else:
            return [self.slide.move(displacement[0], wait=False),
                    self.lift.move(displacement[1],  wait=False)],


    def set_joint(self, point, offset=True):
        """
        Set the joint to a specific point in rest frame coordinates

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


class Stand:
    """
    Represent a detector stand

    This object keeps track of all three joints as well as the angle of the
    stand as a whole. Upon initialization, the stand may not have the proper
    angles set, and the :meth:`.find_angles` method may be used to iteratively
    search for the correct set of angles to satisfy the current motor positions

    Parameters
    ----------
    cone : :class:`.ConeJoint`
        The front joint for the detector stand

    flat : :class:`.AngledJoint`
        The rear joint without a slide on the detector stand

    vee : :class:`.AngledJoint`
        The rear joint with a slide on the detector stand


    Attributes
    ----------
    pitch : float
        The rotation about the X axis in radians

    yaw : float
        The rotation about the Y axis in radians

    roll : float
        The rotation about the Z axis in radians

    joint_names : tuple
        Names of joints assoicated with stand
    """
    joint_names = ('cone', 'vee', 'flat')

    def __init__(self, cone=None, flat=None, vee=None):

        #Angles
        self.pitch = 0.
        self.yaw   = 0.
        self.roll  = 0.

        #Load Joints
        self.cone = cone
        self.flat = flat
        self.vee  = vee


    @property
    def cone_ball(self):
        """
        The :class:`.StandPoint` at the top of the cone joint
        """
        return StandPoint(self.cone.position, self)


    @property
    def vee_ball(self):
        """
        The :class:`.StandPoint` at the top of the vee joint
        """
        return StandPoint(self.vee.position, self)


    @property
    def flat_ball(self):
        """
        The :class:`.StandPoint` at the top of the flat joint
        """
        return StandPoint(self.flat.position, self)


    def find_angles(self, precision= 0.001, min_iterations=30):
        """
        Return the value of the pitch, yaw, and roll through an iterative
        process of comparing estimated angles and motor encoder readbacks.

        Previous attempts to invert the transformation matrix proved too
        inaccurate, so instead an iterative method is used. The algorithm
        considers the location of the cone joint and an estimated set of angles
        to provide a predicted location of each joint. From there, we
        can use the geometry of each joint to see where our estimation would
        put the position of the motors. By comparing  these motor positions
        with the actual encoder values on the stand, we can get a sense of how
        far our predicted angles are from reality. By iterating through this
        process a number of times, the true angle of the stand can be
        determined.

        The algorithm looks for one of two conditions to end the iterative
        process, first if the desired precision for our estimate is reached, as
        well as the minimum number of iterations we can stop searching for a
        solution. Secondly, if the number of iterations has gone past twice the
        minimum value, we stop searching for a solution.

        Parameters
        ----------
        precision : float, optional
            The precision required for the estimated motor positions.

        min_iterations : int, optional
            The minimum number of iterations before the function will exit.

        Returns
        -------
        angles : tuple
            Pitch, Yaw and Roll of the detector chamber
        """
        it = 0
        logger.debug("Finding angles of stand ...")

        #Resting stand positions
        flat = StandPoint(self.flat.offset, self)
        vee  = StandPoint(self.vee.offset,  self)

        #Begin iteration
        while True:

            #Find error in predicted flat motor position from current angles
            fl_e =  self.flat.invert(flat.room_coordinates)- self.flat.displacement
            logger.debug("Found an error of {} mm in the prediction "
                         "of the flat slide motor"
                         "".format(fl_e))

            #Find error in predicted vee motor position from current angles
            predictions  = self.vee.invert(vee.room_coordinates)
            (vs_e, vl_e) = [pred - actual for (actual,pred) in
                            zip(self.vee.displacement, predictions)]
            logger.debug("Found a lift error of {} mm and a slide error of "
                         "{} mm for the motors in the vee joint"
                         "".format(vl_e, vs_e))

            #End iteration if loop and precision thresholds have been met
            if (it > min_iterations and precision > max([fabs(fl_e),
                                                         fabs(vl_e),
                                                         fabs(vs_e)])):
                logger.info("Succesfully found stand angles")
                break

            #End iteration if loop has gone twice the minima
            if it > 2*min_iterations:
                logger.warning("Unable to converge on angles for the stand")
                break

            #Adjust the angle predictions 
            logger.debug("Iteration {} ...".format(it))

            self.pitch += (vl_e + vs_e)/(3*self.vee.offset.z)
            self.yaw   +=  vs_e/(-3*self.vee.offset.z)
            self.roll  += (fl_e - vl_e)/(3*(sin(self.vee.alpha)
                                         +self.vee.offset.x
                                         -self.flat.offset.x))

            logger.debug("Pitch, Yaw, and Roll adjusted to {}"
                         "".format((self.pitch, self.yaw, self.roll)))

            it += 1
        return (self.pitch, self.yaw, self.roll)


    def invert(self, offset=True, **kwargs):
        """
        Find the neccesary motor positions for requested joint displacement

        Parameters
        ----------
        cone : tuple or :class:`.Point`
            Position of cone joint

        flat : tuple or :class:`.Point`
            Position of vee joint

        vee : tuple or :class:`.Point`
            Position of vee joint

        offset : bool
            Whether to remove the requested inherit offset or not

        Returns
        -------
        displacement : dict
            Dictionary with each 
        """
        shift = dict.from_keys(self.joint_names)

        for joint in shift.keys():
            if joint in kwargs:
                shift[joint] = getattr(self,joint).invert(kwargs[joint],
                                                          offset=offset)

        return shift


    def translate(self, dx=0., dy=0., wait=False):
        """"
        Translate the entire stand in x,y

        Parameters
        ----------
        dx : float
            Distance in mm to move in horizontal direction

        dy : float
            Distance in mm to move in vertical direction

        wait : bool
            Block thread until move finishes

        timeout : float
            Amount of time in seconds to wait. None disables, such that wait()
            will only return when either the status completes or if interrupted
            by the user.

        Returns
        -------
        statuses : list of status
            List of MoveStatus

        Raises
        ------
        TimeoutError
            If time waited exceeds specified timeout

        RuntimeError
            If the status failed to complete successfully 
        """
        ax, ay, az = self.pitch, self.yaw, self.roll

        logger.debug("Translating stand {} mm horizontally, and {} "
                     "vertically".format(dx,dy))

        #Calculate displacement in room coordinates
        mx = dx*cos(ay)*cos(az) + dy*sin(az)*cos(ay)
        my = (dx*(sin(ax)*sin(ay)*cos(az) - sin(az)*cos(ax))
            + dy*(sin(ax)*sin(ay)*sin(az) + cos(ax)*cos(az)))

        status =  [self.cone.set_joint((mx,my), offset=False),
                   self.flat.set_joint((mx,my), offset=False),
                   self.vee.set_joint((mx,my),  offset=False)]

        if wait:
            #Wait for each movement to finish
            try:
                print("Waiting for motion to be completed ...") 
                [ophyd.status.wait(s) for s in status]

            #Stop all motion if one motor fails
            except:
                print("Stopping motors ...")
                self.cone.stop()
                self.flat.stop()
                self.vee.stop()
                raise 

        return status
