"""
The detrot module uses three separate reference frames to keep track of various
positions on the detector stand, the rest frame, the stand frame and the room
frame. For the purposes of this application we consider the rest frame to be
when the cone joint at the front of the stand is perfectly vertical, and all of
the other joint motors are zeroed.

From the points in this frame, it is trivial to transform into the stand frame.
Simply adjust the value in x,y based on the current position of the cone joint.
However, this ignores any rotation of the stand. Finally, a more complex
transformation is neccesary to reach the room frame. This matrix takes into
account not only the cone joint position but also the pitch, yaw, and roll
introduced by offsets in the other joints.

The :class:`.StandPoint makes adjusting between these points simple. Simply
enter the rest coordinates of the position and use the attributes
:attr:`.StandPoint.frame_coordinates`, and :attr:`.StandPoint.room_coordinates`
to move between coordinate systems.
"""
############
# Standard #
############
from math        import cos, sin, tan
from collections import namedtuple

###############
# Third Party #
###############


##########
# Module #
##########

class Point(namedtuple('PointBase', ['x','y','z'])):
    """
    Generic Point Class

    A reimplementation of ``collections.namedtuple`` that allows for attribute
    access of the different coordinates.

    Example
    -------
    .. ipython:: python

        pnt = Point(0,1,2)

        pnt == (0,1,2)

        pnt.x == pnt[0]

        print(pnt.x, pnt.y, pnt.z)
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
        Location of the point in the rest frame. All three coordinates must be
        specifed

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
        The coordinates of the point in the stand reference frame as a
        :class:`.Point`
        """
        return Point(self.offset.x + self.stand.cone.x,
                     self.offset.y + self.stand.cone.y,
                     self.offset.z)


    @property
    def room_coordinates(self):
        """
        The coordinates of the point in the reference frame of the room as a
        :class:`.Point`
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
