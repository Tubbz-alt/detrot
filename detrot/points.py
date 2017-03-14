"""
The ``detrot`` module takes advantage of three different reference frames to
view the Detector stand, the rest frame, the stand frame and the room frame.
The most intuitive starting point is the rest frame, this ignores any rotation
and translation of the stand, with the y axis perpindicular to the detector
base and all joints pointing vertically upwards. From here we can translate to
the frame of the surrounding stand by looking at the position of the cone joint
i.e the apex of the three joints that manipulate the stand. We use this as the
center of rotation for all our rigid body calculations. After accounting for
the x,y displacement of the cone, we can then do the more complex calculation
into the room frame. This takes into account the pitch, yaw and roll of the stand
and the fact that the rest frame axes are not colinear with the surrounding
room.

Transforming through these different frames is done through the
:class:`.StandPoint`. Simply enter a point in the rest frame using the cone
joint as the origin, then check either the :attr:`.stand_coordinates` or
:attr:`.room_coordinates` to see how the point moves as the angles of the stand
change.
"""
############
# Standard #
############
import logging
from collections import namedtuple
from math import cos, sin, tan
###############
# Third Party #
###############


##########
# Module #
##########

logger = logging.getLogger(__name__)

class Point(namedtuple('PointBase', ['x','y','z'])):
    """
    Generic Point Class

    This is a reimplementation of the ``collections.namedtuple`` class. This
    allows both index and attribute based access to the position variables.

    Parameters
    ----------
    x : float

    y : float

    z : float

    Example
    -------
    .. ipython:: python

        from detrot import Point

        pnt = Point(1,2,3)

        print(pnt.x)

        pnt.y == 2

        pnt[2] == 3

        pnt == (1,2,3)
    """
    def __repr__(self):
        return "Point (x,y,z -> {},{},{})".format(self.x,
                                                  self.y,
                                                  self.z)

class StandPoint:
    """
    A class to handle the transformations between various frame of references
    involved with stand rotation

    Parameters
    ----------
    offset : tuple or :class:`.Point`
        Location of the point in the rest frame

    stand : class:`.Stand`
        Stand object
    """
    def __init__(self, offset, stand):

        if not isinstance(offset, Point):
            offset = Point(*offset)

        self.offset = offset
        self.stand  = stand


    @property
    def stand_coordinates(self):
        """
        The coordinates of the point in the stand reference frame
        """
        return Point(self.offset.x + self.stand.cone.joint.x,
                     self.offset.y + self.stand.cone.joint.y,
                     self.offset.z)


    @property
    def room_coordinates(self):
        """
        The coordinates of the point in the reference frame of the room
        """
        #Stand Position
        x,y,z    = self.stand_coordinates
        ax,ay,az = self.stand.pitch, self.stand.yaw, self.stand.roll

        #####
        # X #
        #####
        rm_x = (x*cos(ay)*cos(az)
             +  y*(sin(ax)*sin(ay)*cos(az)-sin(az)*cos(ax))
             +  z*(sin(ay)*cos(ax)*cos(az)+sin(ax)*sin(az)))

        #####
        # Y #
        #####
        rm_y =  (x*sin(az)*cos(ay)
              +  y*(sin(ax)*sin(ay)*sin(az)+cos(ax)*cos(az))
              +  z*(sin(ay)*cos(ax)*sin(az)-sin(ax)*cos(az)))

        #####
        # Z #
        #####
        rm_z =  (-x*sin(ay)
                + y*sin(ax)*cos(ay)
                + z*cos(ax)*cos(ay))

        return Point(rm_x, rm_y, rm_z)
