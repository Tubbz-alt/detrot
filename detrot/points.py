"""
Utilities for pystand
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
