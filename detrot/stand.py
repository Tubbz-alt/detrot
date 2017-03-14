"""
Module Docstring
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

        Parameters
        ----------
        dx : float
            Position in mm to move in horizontal direction

        dy : float
            Position in mm to move in vertical direction

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

        return self.set_displacement(cone = self.cone.invert((mx,my), offset=False),
                                     flat = self.flat.invert((mx,my), offset=False),
                                     vee  = self.vee.invert((mx,my),  offset=False),
                                     relative=True, wait=wait, timeout=timeout)


    def rotate(self, dpitch=0., dyaw = 0., droll=0., wait=False, timeout=5.0):
        """
        Rotate the detector stand, while keeping the cone joint stationary

        Parameters
        ----------
        dpitch : float
            Desired change in pitch (radians)

        dyaw : float
            Desired change in yaw (radians)

        droll : float
            Desired change in roll (radians)
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


    def align(self, z, dx=0., dy=0., origin= 0.,
              retries=2, wait=False, timeout=5.0):
        """
        Parameters
        ----------

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
        dpitch = -dy/(z - origin)
        dyaw   = -dx/(origin  - z)

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
            logger.debug("Fixed point has error x,y,z -> ({},{},{})"
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
        model = Stand(cone = ConeJoint.model(stand.cone),
                      flat = AngledJoint.model(stand.flat),
                      vee  = AngledJoint.model(stand.vee),
                      det  = Detector.model(stand.detector))

        model.pitch, model.yaw, model.roll = stand.pitch, stand.yaw, stand.roll
        return model


    def from_model(self, model, wait=False, timeout=5):
        return self.set_displacement(cone = model.cone.displacement,
                                     flat = model.flat.displacement,
                                     vee  = model.vee.displacement,
                                     wait=wait, timeout=timeout)

    def set_displacement(self,  wait=False, timeout=5.0, relative=False,  **kwargs):
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
