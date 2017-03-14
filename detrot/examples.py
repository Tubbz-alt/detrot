############
# Standard #
############

###############
# Third Party #
###############


##########
# Module #
##########
from .stand  import Stand
from .points import Point
from .joints import Detector, AngledJoint, ConeJoint, Point

#######
# DS1 #
#######
ds1 = Stand(cone = ConeJoint(lift   = 'CXI:DS1:MMS:01',
                             slide  = 'CXI:DS1:MMS:02',
                             offset = Point(0.,0.,0.)),

            flat = AngledJoint(lift   = 'CXI:DS1:MMS:03',
                               offset =  Point(-342.9, 361.404, -609.6508)),

            vee  = AngledJoint(lift   = 'CXI:DS1:MMS:04',
                               slide  = 'CXI:DS1:MMS:05',
                               offset =  Point(342.9, 361.404, -609.6508)),

            det = Detector('CXI:DS1:MMS:06',
                            offset = Point(0,480.0346,-914.5524)))

#######
# DS2 #
#######
ds2 = Stand(cone = ConeJoint(lift   = 'CXI:DS2:MMS:01',
                             slide  = 'CXI:DS2:MMS:02',
                             offset = Point(0.,0.,0.)),

            flat = AngledJoint(lift   = 'CXI:DS2:MMS:03',
                               offset =  Point(-342.9, 361.404, -609.6508)),

            vee  = AngledJoint(lift   = 'CXI:DS2:MMS:04',
                               slide  = 'CXI:DS2:MMS:05',
                               offset =  Point(342.9, 361.404, -609.6508)),

            det = Detector('CXI:DS2:MMS:06',
                            offset = Point(0,480.0346,-914.5524)))
