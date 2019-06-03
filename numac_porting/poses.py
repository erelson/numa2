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
    a2 = 45
    a3 = -125
    a4 = 0

    gait.s11pos, gait.s12pos, gait.s13pos, gait.s14pos = \
            gait.leg1.get_pos_from_angle(0, a2, a3, a4)
    gait.s21pos, gait.s22pos, gait.s23pos, gait.s24pos = \
            gait.leg2.get_pos_from_angle(0, a2, a3, a4)
    gait.s31pos, gait.s32pos, gait.s33pos, gait.s34pos = \
            gait.leg3.get_pos_from_angle(0, a2, a3, a4)
    gait.s41pos, gait.s42pos, gait.s43pos, gait.s44pos = \
            gait.leg4.get_pos_from_angle(0, a2, a3, a4)

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
    a2 = 45
    a3 = -125
    a4 = 0
    # Don't send positions to coax servos
    my_leg_ids = leg_ids[4:]
    _, gait.s12pos, gait.s13pos, gait.s14pos = \
            gait.leg1.get_pos_from_angle(0, a2, a3, a4)
    _, gait.s22pos, gait.s23pos, gait.s24pos = \
            gait.leg2.get_pos_from_angle(0, a2, a3, a4)
    _, gait.s32pos, gait.s33pos, gait.s34pos = \
            gait.leg3.get_pos_from_angle(0, a2, a3, a4)
    _, gait.s42pos, gait.s43pos, gait.s44pos = \
            gait.leg4.get_pos_from_angle(0, a2, a3, a4)
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

    a2 = 45
    a3 = -125
    a4 = 0
    # TODO I didn't change these yet
    gait.s11pos, gait.s12pos, gait.s13pos, gait.s14pos = \
            gait.leg1.get_pos_from_angle(0, a2, a3, a4)
    gait.s21pos, gait.s22pos, gait.s23pos, gait.s24pos = \
            gait.leg2.get_pos_from_angle(0, a2, a3, a4)
    gait.s31pos, gait.s32pos, gait.s33pos, gait.s34pos = \
            gait.leg3.get_pos_from_angle(0, a2, a3, a4)
    gait.s41pos, gait.s42pos, gait.s43pos, gait.s44pos = \
            gait.leg4.get_pos_from_angle(0, a2, a3, a4)

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

    # OLD: These should be same as g8Stand
    # angles are leg angles
    a2 = 90
    a3 = -155
    a4 = 0
    # Don't set positions to coax servos
    my_leg_ids = leg_ids[4:]
    _, gait.s12pos, gait.s13pos, gait.s14pos = \
            gait.leg1.get_pos_from_angle(0, a2, a3, a4)
    _, gait.s22pos, gait.s23pos, gait.s24pos = \
            gait.leg2.get_pos_from_angle(0, a2, a3, a4)
    _, gait.s32pos, gait.s33pos, gait.s34pos = \
            gait.leg3.get_pos_from_angle(0, a2, a3, a4)
    _, gait.s42pos, gait.s43pos, gait.s44pos = \
            gait.leg4.get_pos_from_angle(0, a2, a3, a4)

#    sleep_ms(2000) #probably too short, but a long wait is scary, too.

    # stop IK and Gait from processing, whichever was active...
    #walk = False

    # ??
    axbus.sync_write(my_leg_ids, ax.GOAL_POSITION,
            [struct.pack('<H', int(pos)) for pos in
                       (gait.s12pos, gait.s22pos, gait.s32pos, gait.s42pos,
                        gait.s13pos, gait.s23pos, gait.s33pos, gait.s43pos,
                        gait.s14pos, gait.s24pos, gait.s34pos, gait.s44pos)])

    # Let the servos move
    sleep_ms(400)

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
#      |5      TODO mirror this            __|            
#
#
#      |___________legLen________|

def gen_numa2_legs():
# 4\ __^__ /3
#   |     | 
#   |numa2| 
#   |_____| 
# 1/       \2
    stance = 5
    offsets_dict = {
            # Offsets are in degrees
            "aoffset1": 45.0, # this one is special
            "aoffset2": 31.54,
            "aoffset3": 31.54 - 5.63, # off_b - off_h
            "a1stance": stance,
            "a1stance_rear": -10,#5, # degrees
            "L0": 130,
            "L12": 58,
            "L23": 65, #63,
            "L34": 130, #67,
            "L45": 5,  # This is fake right?
            # mins/max are in degrees from actual servo center (not joint center!)
            "max1": 95,
            "min1": -10,
            "max2": 100,
            "min2": -68,
            "max3": 10, #90,
            "min3": -140, #-20,
            #
            "joint2sign": -1,
            "joint3sign": 1,
            }
    leg_model = LegGeom(offsets_dict)

    # leg_geom, s1_sign, s2_sign, s3_sign, s4_sign=None, front_leg=True):
    leg1 = LegDef(leg_model, offsets_dict,  1, -1,  1)
    leg2 = LegDef(leg_model, offsets_dict, -1,  1, -1)
    leg3 = LegDef(leg_model, offsets_dict,  1, -1,  1, front_leg=True)
    leg4 = LegDef(leg_model, offsets_dict, -1,  1, -1, front_leg=True)

    return leg_model, leg1, leg2, leg3, leg4


class LegGeom(object):
    """

    Conventions:
    - See ascii drawings in IK.py
    - Can support 3 or 4 servos per leg
    - Can optionally specify servo type. Default is AX-12A

    Each servo has an offset defined by

    """

    def __init__(self, offsets_dict):

        self.L0 =  offsets_dict.pop("L0")
        self.L12 = offsets_dict.pop("L12")
        self.L23 = offsets_dict.pop("L23")
        self.L34 = offsets_dict.pop("L34")
        self.L45 = offsets_dict.pop("L45")

        # Offsets are specified in degrees
        self.aoffset1 = offsets_dict.pop("aoffset1")
        self.aoffset2 = offsets_dict.pop("aoffset2")
        self.aoffset3 = offsets_dict.pop("aoffset3")
        self.aoffset4 = offsets_dict.pop("aoffset4", 0)

        # +1 if servo is on non-moving side of joint, -1 if servo is on moving side.
        self.joint2sign = offsets_dict.pop("joint2sign", 1)
        self.joint3sign = offsets_dict.pop("joint3sign", 1)
        self.joint4sign = offsets_dict.pop("joint4sign", 1)

        # Stance is offset from default 45 degree leg direction. Positive stance puts
        # forward legs more forward and rear legs more rearward
        self.a1stance = offsets_dict.pop("a1stance")
        # Optional rear stance lets front legs and back legs have separate stance angle
        self.a1stance_rear = offsets_dict.pop("a1stance_rear", self.a1stance)

        # Joint max/min angles
        # TODO(enhancement): genericize for other servo types
        self.max_angle = {}
        self.min_angle = {}
        for n in range(1,5):
            self.max_angle[n] = offsets_dict.pop("max{0}".format(n), 150)
            self.min_angle[n] = offsets_dict.pop("min{0}".format(n), -150)

        self.pos_lookup = {"ax12": self.ax12pos,
                           "ax12a": self.ax12pos,
        }

    def ax12pos(self, angle):
        """Return an angle converted from degrees into integer position values for the servo
        
        Note: Generally combined with an offset representing the servo's center position
        """
        return int(angle/150.0 * 512) # Degrees -> servo position


class LegDef(object):
    """A representation of the servos in a leg that can be generically given
    a set of joint angles and will generate corresponding servo positions,
    accounting for orientation, etc.
    """

    def __init__(self, leg_geom, offsets_dict, s1_sign, s2_sign, s3_sign, s4_sign=1, front_leg=True):
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

        # Set per-joint position functions based on servo type
        self.pos1 = leg_geom.pos_lookup[offsets_dict.pop("servo1_type", "ax12")]
        self.pos2 = leg_geom.pos_lookup[offsets_dict.pop("servo2_type", "ax12")]
        self.pos3 = leg_geom.pos_lookup[offsets_dict.pop("servo3_type", "ax12")]
        self.pos4 = leg_geom.pos_lookup[offsets_dict.pop("servo4_type", "ax12")]

        # This is so tedious, elegance would be cool.
        # TODO non-AX-12 specific
        s1lims = [511 + self.s1_sign * self.pos1(leg_geom.max_angle[1]), 511 + self.s1_sign * self.pos1(leg_geom.min_angle[1])]
        s2lims = [511 + self.s2_sign * self.pos2(leg_geom.max_angle[2]), 511 + self.s2_sign * self.pos2(leg_geom.min_angle[2])]
        s3lims = [511 + self.s3_sign * self.pos3(leg_geom.max_angle[3]), 511 + self.s3_sign * self.pos3(leg_geom.min_angle[3])]
        s4lims = [511 + self.s4_sign * self.pos4(leg_geom.max_angle[4]), 511 + self.s4_sign * self.pos4(leg_geom.min_angle[4])]
        s1lims.sort()
        s2lims.sort()
        s3lims.sort()
        s4lims.sort()
        self.s1min, self.s1max = s1lims
        self.s2min, self.s2max = s2lims
        self.s3min, self.s3max = s3lims
        self.s4min, self.s4max = s4lims
        # Note: 512 is the real center position per dynamixel wizard, but +/- 512
        # will take us out of bounds...
        self.s1min = self.s1min if self.s1min >= 0 else 0
        self.s2min = self.s2min if self.s2min >= 0 else 0
        self.s3min = self.s3min if self.s3min >= 0 else 0
        self.s4min = self.s4min if self.s4min >= 0 else 0

        # Convert offsets in degrees to servo values
        self.s1_center_angle = self.s1_sign * (leg_geom.aoffset1 + a1_stance_offset)
        self.s1_center_radians = self.s1_center_angle / RAD_TO_ANGLE
        self.s1_center = 512 + self.pos1(self.s1_center_angle)
        self.s2_center = 512 + self.pos2(self.s2_sign * leg_geom.aoffset2)
        self.s3_center = 512 + self.pos3(self.s3_sign * leg_geom.aoffset3)
        if leg_geom.aoffset4:
            self.s4_center = 512 + self.pos4(self.s4_sign * leg_geom.aoffset4)
        else:
            self.s4_center = 512

    def get_pos_from_angle(self, a1, a2, a3, a4=None):
        # Angles are in degrees. Returns list of servo positions
        positions = [
                self.s1_center + self.pos1(a1),  # Remember to supply offset from center, not absolute angle
                self.s2_center + self.pos2(self.s2_sign * self.leg_geom.joint2sign * a2),
                self.s3_center + self.pos3(self.s3_sign * self.leg_geom.joint3sign * a3),
        ]

        if a4 and self.s4_sign:
            positions.append(self.s4_center + self.pos4(
                self.s4_sign * self.leg_geom.joint4sign * a4))
        else:
            positions.append(self.s4_center)

        # TODO check limits

        return positions

    def get_pos_from_radians(self, a1, a2, a3, a4=None):
        # Current convention is we convert all angles from radians to degrees
        positions = [
                self.s1_center + self.pos1(RAD_TO_ANGLE * a1),
                self.s2_center + self.pos2(RAD_TO_ANGLE * self.s2_sign * self.leg_geom.joint2sign * a2),
                self.s3_center + self.pos3(RAD_TO_ANGLE * self.s3_sign * self.leg_geom.joint3sign * a3),
        ]

        if a4 and self.leg_geom.aoffset4:
            positions.append(self.s4_center + self.pos4(
                RAD_TO_ANGLE * self.s4_sign * self.leg_geom.joint4sign * a4))
        else:
            positions.append(self.s4_center)

        # TODO check/enforce limits

        return positions
