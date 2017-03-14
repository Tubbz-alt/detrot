"""
The :class:`.Stand` is made up of four joints, the cone and vee, both
:class:`.AngledJoint` classes, as well as the :class:`.ConeJoint` and internal
:class:`.Detector` slide. The stand takes care of abstracting the manipulation
of each of these joints into one rigid body which can be both translated and
rotated with their corresponding methods :meth:`.translate` and
:meth:`.rotate`. By combining these two methods, higher level functionality
becomes available such as the :meth:`.align` method. Uses a combination of both
rotation and translation to keep an arbitrary point on the Z axis fixed, while
another is displaced in X and Y, allowing us to align the internal motor of the
detector stand with the beam

Because a few of the processes described use an iterative approach to finding
the correct motor positions, the class makes heavy use of the :meth:`.model`
methods of the joints in order to quickly create a simulated version of the
detector stand. This is useful to explore what requested rotations and
translations will have on the system, without actually moving motors.
You can always work entirely on the model and then use the :meth:`.from_model`
on the actual detector stand object to move the motors to match your simulated
moves
"""
############
# Standard #
############
import logging
from math import cos, sin, tan, fabs
###############
# Third Party #
###############


##########
# Module #
##########
from .points import StandPoint, Point
from .joints import AngledJoint, ConeJoint, Detector

logger = logging.getLogger(__name__)

class Stand:
    """
    Represent a detector stand

    This object keeps track of all three joints as well as the angle of the
    stand as a whole. Upon initialization, the stand may not have the proper
    angles set, use the :meth:`.find_angles` method may be used to iteratively
    search for the correct set of angles to satisfy the current motor positions

    Parameters
    ----------
    cone : :class:`.ConeJoint`
        The apex of the joint for the detector stand. Used as the origin of the
        coordinate system

    flat : :class:`.AngledJoint`
        The angeled joint without a slide on the detector stand. Only one
        degree of freedom that moves in both X and Z simultaneously

    vee : :class:`.AngledJoint`
        The rear joint with a slide on the detector stand. Two degrees of
        freedom , one motion in X and one in both X and Z simultaneously

    det : class:`.Detector`
        The internal stage of the Detector

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

    def __init__(self, cone=None, flat=None, vee=None, det=None):

        #Angles
        self.pitch = 0.
        self.yaw   = 0.
        self.roll  = 0.

        #Load Joints
        self.cone = cone
        self.flat = flat
        self.vee  = vee
        self.detector = det


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

        flat = StandPoint(self.flat.offset, self)
        vee  = StandPoint(self.vee.offset,  self)
        #Begin iteration
        while True:

            #Find error in predicted flat motor position from current angles
            fl_e =  (self.flat.invert(flat.room_coordinates)
                     - self.flat.displacement)
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


    def translate(self, dx=0., dy=0., wait=False, timeout=5.0):
        """"
        Translate the entire stand in x,y

        This calculation is done by using the x and y part of the rotation
        matrix to evaluate the corresponding translation in the rest
        coordinates. From there, the ``invert`` method of each joint is used to
        determine the motor positions that move the joints to the desired
        position

        Parameters
        ----------
        dx : float
            Relative horizontal move in room coordinates (mm)

        dy : float
            Relative vertical move in room coordinates (mm)

        wait : bool
            Block thread until move finishes

        timeout : float
            Amount of time in seconds to wait. None disables, such that wait
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

        return self.set_displacement(cone = self.cone.invert((mx,my), offset=False),
                                     flat = self.flat.invert((mx,my), offset=False),
                                     vee  = self.vee.invert((mx,my),  offset=False),
                                     relative=True, wait=wait, timeout=timeout)


    def rotate(self, dpitch=0., dyaw = 0., droll=0., wait=False, timeout=5.0):
        """
        Rotate the detector stand, while keeping the cone joint stationary

        This is calculated by using the :meth:`.model` of the stand and
        manually adjusting the angles to meet the requested move. From there we
        can find the position of each joint by using the
        :attr:`.room_coordinates` of each of the joint balls. Once we have
        these the ``invert`` method of each joint gives us the requested motor
        positions that correspond to the change in rotation

        Parameters
        ----------
        dpitch : float
            Desired change in pitch (radians)

        dyaw : float
            Desired change in yaw (radians)

        droll : float
            Desired change in roll (radians)

        wait : bool
            Block thread until move finishes

        timeout : float
            Amount of time in seconds to wait. None disables, such that wait
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
        #Setup model with requested angles
        model = Stand.model(self)
        model.pitch += dpitch
        model.yaw   += dyaw
        model.roll  += droll

        #See where joint positions lie in the new model
        model_vee  = StandPoint(self.vee.offset,  model)
        model_flat = StandPoint(self.flat.offset, model)

        #Make motor agree with model
        return self.set_displacement(flat = self.flat.invert(model_flat.room_coordinates),
                                     vee  = self.vee.invert(model_vee.room_coordinates),
                                     wait=wait, timeout=timeout)


    def align(self, z, origin=0., dx=0., dy=0.,
              retries=2, wait=False, timeout=5.0):
        """
        Align the internal detector axis by keeping one point fixed and rotating
        another.

        The alignment procedure is done by calculating an initial rotation of
        the stand using the requested displacement and a small angle
        approximation. This should produce the desired change in the floating
        point, but depending on the distance of the fixed point from the Z
        axis, it will also have moved. This requires a small correction
        translation to adjust the fixed point back to its original position.
        This process is done iteratively as to not move to drastically and
        affecting the results of our original rotation.

        Because these processes are done iteratively, and require exploring the
        results of different moves of the stand, the :meth:`.model` is used to
        view the result ahead of time.

        Parameters
        ----------
        z : float
            The floating point on the detector axis to rotate

        dx : float, optional
            The requested change in the horizontal position in room coordinates

        dy : float, optional
            The requested change in the vertical position in room coordinates

        origin : float, optional
            The fixed point on the detector axis

        retries : int, optional
            The algorithm will do a number of retry translations to move the
            fixed point back to to its original position after rotation. An
            increased number of retries will fix this point more accurately,
            but risks moving the rotated point

        wait : bool
            Block thread until move finishes

        timeout : float
            Amount of time in seconds to wait. None disables, such that wait
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
        logger.debug("Rotating point {} on the detector axis about point {}"
                     "".format(z, origin))
        #Setup model
        model       = Stand.model(self)
        model_fixed = StandPoint((model.detector.position.x,
                                  model.detector.position.y,
                                  origin),
                                  model)

        #Save initial position for reference
        initial = model_fixed.room_coordinates

        #Small angle approximate
        dpitch = dy/(origin - z)
        dyaw   = dx/(z - origin)

        logger.debug("Small angle approximation requests a change in pitch "
                     "and yaw of {}, {} respectively"
                     "".format(dpitch, dyaw))

        #Rotate model to move desired point
        model.rotate(dpitch=dpitch, dyaw=dyaw)
        model.find_angles()

        #Correction on fixed origin
        its = 0
        while its < retries:
            #Find change from initial origin
            dx, dy, dz = [m -r for (m,r) in zip(model_fixed.room_coordinates,
                                                initial)]
            logger.info("Fixed point has error x,y,z -> ({},{},{})"
                        "".format(dx,dy,dz))

            #Approximate neccesary translation
            xslope = dz * (model.pitch*model.roll + model.yaw)
            yslope = dz * (model.pitch + model.yaw*model.roll)

            #Translate 
            model.translate(-dx + xslope, -dy - yslope)

            #Next iteration
            its += 1

        return self.from_model(model, wait=wait, timeout=timeout)


    @classmethod
    def model(cls, stand):
        """
        Create an identical stand but with the actual motors replaced by
        ``SoftPositioner`` instances

        Parameters
        ----------
        stand : :class:`.Stand`
            Stand to be modeled

        Returns
        -------
        model : :class:`.Stand`
            Copied model with ``SoftPositioner`` joints
        """
        model = Stand(cone = ConeJoint.model(stand.cone),
                      flat = AngledJoint.model(stand.flat),
                      vee  = AngledJoint.model(stand.vee),
                      det  = Detector.model(stand.detector))

        model.pitch, model.yaw, model.roll = stand.pitch, stand.yaw, stand.roll
        return model


    def from_model(self, model, wait=False, timeout=5):
        """
        Move the detector stand to be in aggreance with a model

        Parameters
        ----------
        model : class:`.Stand`
            The model to copy

        wait : bool
            Block thread until move finishes

        timeout : float
            Amount of time in seconds to wait. None disables, such that wait
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
        return self.set_displacement(cone = model.cone.displacement,
                                     flat = model.flat.displacement,
                                     vee  = model.vee.displacement,
                                     wait=wait, timeout=timeout)

    def set_displacement(self,  wait=False, timeout=5.0, relative=False,  **kwargs):
        """
        Set the displacement of the joints

        Parameters
        ----------
        cone : tuple
            Desired position or move of the cone joint (slide, lift)

        vee : tuple
            Desired position or move of the vee joint (slide, lift)

        flat : float
            Desired position or move of the vee joint

        relative : bool, optional
            Choice of relative or absolute move

        wait : bool
            Block thread until move finishes

        timeout : float
            Amount of time in seconds to wait. None disables, such that wait
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
        status = []

        #Find requested joints
        for joint in self.joint_names:
            if joint in kwargs:
                #Order move
                st = getattr(self, joint).set_displacement(kwargs[joint],
                                                           relative=relative)
                #Add status to pile
                status.append(st)

        #Perform motion
        if wait:
            #Wait for each movement to finish
            try:
                print("Waiting for motion to be completed ...") 
                [ophyd.status.wait(s, timeout=timeout) for s in status]

            #Stop all motion if one motor fails
            except:
                print("Exception raised, stopping motors ...")
                self.cone.stop()
                self.flat.stop()
                self.vee.stop()
                raise

        return status
