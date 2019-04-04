import struct
import sys
from math import pi
sysname = sys.platform
if sysname == 'linux' or sysname == 'win32':
    # Mocks for non-pyboard use
    import time
    def sleep_ms(x): return time.sleep(x/1e3)
elif sysname == 'pyboard':
    from utime import sleep_ms

import ax


RAD_TO_ANGLE = 180./pi

# Send neutral standing positions to all servos.
def g8Stand(gait, axbus, leg_ids):

    gait.s11pos, gait.s12pos, gait.s13pos, gait.s14pos = \
            gait.leg1.get_pos_from_angle(gait.leg1.s1_center_angle, -45, 110, 0)
    gait.s21pos, gait.s22pos, gait.s23pos, gait.s24pos = \
            gait.leg2.get_pos_from_angle(gait.leg2.s1_center_angle, -45, 110, 0)
    gait.s31pos, gait.s32pos, gait.s33pos, gait.s34pos = \
            gait.leg3.get_pos_from_angle(gait.leg3.s1_center_angle, -45, 110, 0)
    gait.s41pos, gait.s42pos, gait.s43pos, gait.s44pos = \
            gait.leg4.get_pos_from_angle(gait.leg4.s1_center_angle, -45, 110, 0)

    axbus.sync_write(leg_ids, ax.GOAL_POSITION,
            [struct.pack('<H', int(pos)) for pos in
                       (gait.s11pos, gait.s21pos, gait.s31pos, gait.s41pos,
                        gait.s12pos, gait.s22pos, gait.s32pos, gait.s42pos,
                        gait.s13pos, gait.s23pos, gait.s33pos, gait.s43pos,
                        gait.s14pos, gait.s24pos, gait.s34pos, gait.s44pos)])
#    sleep_ms(2000)

    # stop IK and Gait from processing, whichever was active...
    #walk = False
    return False

# Send standing positions to all servos. BUT don't rotate legs to center position
def g8FeetDown(gait, axbus, leg_ids):
    # Don't seg positions to coax servos
    my_leg_ids = leg_ids[4:]
    _, gait.s12pos, gait.s13pos, gait.s14pos = \
            gait.leg1.get_pos_from_angle(gait.leg1.s1_center_angle, -45, 110, 0)
    _, gait.s22pos, gait.s23pos, gait.s24pos = \
            gait.leg2.get_pos_from_angle(gait.leg2.s1_center_angle, -45, 110, 0)
    _, gait.s32pos, gait.s33pos, gait.s34pos = \
            gait.leg3.get_pos_from_angle(gait.leg3.s1_center_angle, -45, 110, 0)
    _, gait.s42pos, gait.s43pos, gait.s44pos = \
            gait.leg4.get_pos_from_angle(gait.leg4.s1_center_angle, -45, 110, 0)
    axbus.sync_write(my_leg_ids, ax.GOAL_POSITION,
            [struct.pack('<H', int(pos)) for pos in
                       (gait.s12pos, gait.s22pos, gait.s32pos, gait.s42pos,
                        gait.s13pos, gait.s23pos, gait.s33pos, gait.s43pos,
                        gait.s14pos, gait.s24pos, gait.s34pos, gait.s44pos)])

    # TODO
    #sleep_ms(200) #probably too short, but a long wait is scary, too.

    # stop IK and Gait from processing, whichever was active...
    #walk = False
    return False

# Send standing positions to all servos.
def g8Flop(gait, axbus, leg_ids):
    # TODO unused

    #s11pos = 639 + 20
    #s21pos = 383 - 20
    #s31pos = 639
    #s41pos = 383

    #s12pos = 358 - 105
    #s22pos = 664 + 105
    #s32pos = 358 - 105
    #s42pos = 664 + 105

    #s13pos = 358 - 130
    #s23pos = 664 + 130
    #s33pos = 358 - 130
    #s43pos = 664 + 130

    #s14pos = 511 #409+39)
    #s24pos = 511 #613-39)
    #s34pos = 511 #409+39)
    #s44pos = 511 #613-39)
    # TODO I didn't change these yet
    gait.s11pos, gait.s12pos, gait.s13pos, gait.s14pos = \
            gait.leg1.get_pos_from_angle(gait.leg1.s1_center_angle, -45, 110, 0)
    gait.s21pos, gait.s22pos, gait.s23pos, gait.s24pos = \
            gait.leg2.get_pos_from_angle(gait.leg2.s1_center_angle, -45, 110, 0)
    gait.s31pos, gait.s32pos, gait.s33pos, gait.s34pos = \
            gait.leg3.get_pos_from_angle(gait.leg3.s1_center_angle, -45, 110, 0)
    gait.s41pos, gait.s42pos, gait.s43pos, gait.s44pos = \
            gait.leg4.get_pos_from_angle(gait.leg4.s1_center_angle, -45, 110, 0)

    axbus.sync_write(leg_ids, ax.GOAL_POSITION,
            [struct.pack('<H', int(pos)) for pos in
                       (gait.s11pos, gait.s21pos, gait.s31pos, gait.s41pos,
                        gait.s12pos, gait.s22pos, gait.s32pos, gait.s42pos,
                        gait.s13pos, gait.s23pos, gait.s33pos, gait.s43pos,
                        gait.s14pos, gait.s24pos, gait.s34pos, gait.s44pos)])
    # TODO
    #sleep_ms(2000) #probably too short, but a long wait is scary, too.

    # stop IK and Gait from processing, whichever was active...
    #walk = False
    return False

# Lower feet to ground regardless of shoulder servo position, then cut torque to prevent overheating
def g8Crouch(gait, axbus, leg_ids):
    my_leg_ids = leg_ids[4:]

    # Don't set positions to coax servos
    my_leg_ids = leg_ids[4:]
    _, gait.s12pos, gait.s13pos, gait.s14pos = \
            gait.leg1.get_pos_from_angle(gait.leg1.s1_center_angle, -90, 110, 0)
    _, gait.s22pos, gait.s23pos, gait.s24pos = \
            gait.leg2.get_pos_from_angle(gait.leg2.s1_center_angle, -90, 110, 0)
    _, gait.s32pos, gait.s33pos, gait.s34pos = \
            gait.leg3.get_pos_from_angle(gait.leg3.s1_center_angle, -90, 110, 0)
    _, gait.s42pos, gait.s43pos, gait.s44pos = \
            gait.leg4.get_pos_from_angle(gait.leg4.s1_center_angle, -90, 110, 0)

#    sleep_ms(2000) #probably too short, but a long wait is scary, too.

    # stop IK and Gait from processing, whichever was active...
    #walk = False

    # ??
    axbus.sync_write(my_leg_ids, ax.GOAL_POSITION,
            [struct.pack('<H', int(pos)) for pos in
                       (gait.s12pos, gait.s22pos, gait.s32pos, gait.s42pos,
                        gait.s13pos, gait.s23pos, gait.s33pos, gait.s43pos,
                        gait.s14pos, gait.s24pos, gait.s34pos, gait.s44pos)])

    sleep_ms(200)

    # Disable torque to 2nd servo of each leg
    axbus.sync_write(leg_ids[4:8], ax.TORQUE_ENABLE, [bytearray([0]) for __ in range(4)])

    return

#  a: default leg position (fixed)
#  b: walk vector (from controller)
#  c: trav offset vector (dynamic for each leg)
# Add a + c to get leg vector and determine leg length
#
#        c      _ b   
#       /       /|    
#      a\      /    / 
#        \    /    /  
#         \ _____ /   
#          |     |    
#          |numa |    counter clockwise is positive direction
#          |_____|    
#         /       \   
#        /         \  
#       /           \ 

#                          //\3                           
#          3-4-2 = alph2  // \\            (3)            
#          3-2-4 = alph3 //   \\                          
#                        4\__  \\__     4                 
#                            \__\==      \                
#               3               2      ___\2              
#              /\                     H  alph1            
#             /  \                                        
#            /    \          alph2 +/- alph1 = 3-4-horiz  
#           /      \                                      
#          /        \                                     
#         /          \                                    
#        /            \                                   
#     4 /              \                                  
#      |                \        |                        
#      |                 \_______|1        ___            
#      |                  2      |_______    |            
#      |                                     |- bodyH 
#      |5                                  __|            
#
#
#      |___________legLen________|

def gen_numa2_legs():
# 4\ __^__ /3
#   |     | 
#   |numa2| 
#   |_____| 
# 1/       \2
    stance = 0
    offsets_dict = {
            "aoffset1": 45.0,
            "aoffset2": -31.54,
            "aoffset3": -31.54,
            "a1stance": stance,
            "a1stance_rear": stance,
            "L0": 110, # mm - pretty close to actual...
            "L12": 58,
            "L23": 63,
            "L34": 67,
            "L45": 57,  # This is fake right?
            }
    leg_model = LegGeom(offsets_dict)

    # leg_geom, s1_sign, s2_sign, s3_sign, s4_sign=None, front_leg=True):
    leg1 = LegDef(leg_model, offsets_dict,  1,  1, -1)
    leg2 = LegDef(leg_model, offsets_dict, -1, -1,  1)
    leg3 = LegDef(leg_model, offsets_dict,  1,  1, -1, front_leg=True)
    leg4 = LegDef(leg_model, offsets_dict, -1, -1,  1, front_leg=True)

    return leg_model, leg1, leg2, leg3, leg4

# offset1: Varies per leg + or - 45, + or -
# offset2: -31.54
# offset3: -31.54
# offset4: N/A
class LegGeom(object):
    """

    Conventions:
    - See ascii drawings in IK.py
    - Can support 3 or 4 servos per leg
    - Can optionally specify servo type. Default is AX-12A

    Each servo has an offset defined by

    """

    #def __init__(self, aoffset1, aoffset2, aoffset3, aoffset4=None):
    def __init__(self, offsets_dict):

        self.L0 =  offsets_dict.pop("L0")
        self.L12 = offsets_dict.pop("L12")
        self.L23 = offsets_dict.pop("L23")
        self.L34 = offsets_dict.pop("L34")
        self.L45 = offsets_dict.pop("L45")

        # Offsets are specified in degrees, then converted to radians
        # TODO validate direction is +/- 1 # ?????
        self.aoffset1 = pi / 180. * offsets_dict.pop("aoffset1")
        self.aoffset2 = pi / 180. * offsets_dict.pop("aoffset2")
        self.aoffset3 = pi / 180. * offsets_dict.pop("aoffset3")
        self.aoffset4 = pi / 180. * offsets_dict.pop("aoffset4", 0)

        # Stance is offset from default 45 degree leg direction. Positive stance puts
        # forward legs more forward and rear legs more rearward
        self.a1stance = offsets_dict.pop("a1stance")
        # Optional rear stance lets front legs and back legs have separate stance angle
        self.a1stance_rear = offsets_dict.pop("a1stance_rear", self.a1stance)

        self.pos_lookup = {"ax12": self.ax12pos,
                           "ax12a": self.ax12pos,
        }


    def ax12pos(self, angle):
        """Return an angle converted from degrees into integer position values for the servo
        
        Note: Generally combined with an offset representing the servo's center position
        """
        #return 511 + angle/150.0 * 511 # Degrees
        return int(angle/150.0 * 512) # Degrees
        #return 511 + angle/(5/12. * pi) * 511 # TODO verify; # Radians


class LegDef(object):
    """A representation of the servos in a leg that can be generically given
    a set of joint angles and will generate corresponding servo positions,
    accounting for orientation, etc.
    """

    def __init__(self, leg_geom, offsets_dict, s1_sign, s2_sign, s3_sign, s4_sign=None, front_leg=True):
        """
        offsets_dict : dict
            Dictionary with keys 'servoX_type'.
        """
        self.leg_geom = leg_geom
        self.s1_sign = s1_sign 
        self.s2_sign = s2_sign
        self.s3_sign = s3_sign
        self.s4_sign = s4_sign

        # degrees
        a1_stance_offset = self.leg_geom.a1stance if front_leg else self.leg_geom.a1stance_rear
        #a1_mount_offset = self.leg_geom.aoffset1

        # Set per-joint position functions based on servo type
        self.pos1 = leg_geom.pos_lookup[offsets_dict.pop("servo1_type", "ax12")]
        self.pos2 = leg_geom.pos_lookup[offsets_dict.pop("servo2_type", "ax12")]
        self.pos3 = leg_geom.pos_lookup[offsets_dict.pop("servo3_type", "ax12")]
        self.pos4 = leg_geom.pos_lookup[offsets_dict.pop("servo4_type", "ax12")]

        # Convert offsets in degrees to servo values
        self.s1_center_angle = self.s1_sign * (leg_geom.aoffset1 + a1_stance_offset)
        self.s1_center = 512 + self.pos1(self.s1_center_angle)
        self.s2_center = 512 + self.pos2(self.s2_sign * leg_geom.aoffset2)
        self.s3_center = 512 + self.pos3(self.s3_sign * leg_geom.aoffset3)
        if self.s4_sign:
            self.s4_center = 512 + self.pos4(self.s4_sign * leg_geom.aoffset4)
        else:
            self.s4_center = 512

    def get_pos_from_angle(self, a1, a2, a3, a4=None):
        # degrees
        positions = [
               self.s1_center + self.pos1(a1),
               self.s2_center + self.pos2(self.s2_sign * a2),
               self.s3_center + self.pos3(self.s3_sign * a3),
        ]

        if a4:
            positions.append(self.leg_geom.pos4(self.s4_sign * (a4 + self.leg_geom.aoffset4)))
        else:
            positions.append(512)

        # TODO check limits

        return positions

    def get_pos_from_radians(self, a1, a2, a3, a4=None):
        # Current convention is we convert all angles from radians to degrees
        positions = [
               self.s1_center + self.pos1(RAD_TO_ANGLE * a1),
               self.s2_center + self.pos2(RAD_TO_ANGLE * self.s2_sign * a2),
               self.s3_center + self.pos3(RAD_TO_ANGLE * self.s3_sign * a3),
                ]

        if a4 and self.s4_sign:
            positions.append(self.s4_center + self.pos4(RAD_TO_ANGLE * self.s4_sign * a4))
        else:
            positions.append(self.s4_center)

        # TODO check/enforce limits

        return positions
