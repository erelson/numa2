import sys
from math import cos, sin, acos, pi, sqrt, atan2, copysign

PRINT_DEBUG_IK = False

if (sys.version_info > (3, 0)):
    # Python 3 code in this block
    from array import array
else:
    # Python 2 code in this block
    from numpy import array as _array
    # This is a hack
    def array(_atype, data):
        return _array(data)


#uint8_t calc_foot_h(int16_t now, uint16_t foot_h_max, float time_down_frac,
#    int16_t half_loopLength, float transition_frac, float height_frac:
def calc_foot_h(now, foot_h_max, time_down_frac, half_loopLength, transition_frac, height_frac):
    """"""
    # Important note, `now` must be in the range from 0 to 2 * half_loopLength

    # Foot going down slowly
    if now >= (2.0 - time_down_frac) * half_loopLength:
        footH = int(height_frac * foot_h_max * (2.0 * half_loopLength - now) /
            (time_down_frac * half_loopLength))
    # Foot going down quickly
    elif now >= (2.0 - transition_frac) * half_loopLength:
        footH = int((height_frac * foot_h_max + (1.0 - height_frac) * foot_h_max) *
            ((2.0 - time_down_frac) * half_loopLength - now) /
            ((transition_frac - time_down_frac) * half_loopLength))
    # Steady at max height
    elif now >= (1 + transition_frac) * half_loopLength:
        footH = foot_h_max
    # Foot going up quickly
    elif now >= half_loopLength * (1.0 + time_down_frac):
        footH = int(height_frac * foot_h_max + (1.0 - height_frac) * foot_h_max *
            ((now - half_loopLength * (1.0 + time_down_frac)) /
            ((transition_frac - time_down_frac) * half_loopLength)))
    # Foot going up slowly
    elif now >= half_loopLength:
        footH = int(height_frac * foot_h_max*(now - half_loopLength) /
            (time_down_frac * half_loopLength))
    # Foot on ground
    else:
        footH = 0

    return footH


# Two functions defined here:
# - walk_code: takes params defining sawtooth (?) time wave and point on wave; returns ...
# - turnCode

TURN_ANGLE = 25 # angle sweep of legs when turning??
TURN_INCR = 10 # UNUSED?
FH = 25  # maximum foot height, (mm)
FH_FRAC = 0.07  # fraction of time over which foot height slowly increases from zero
FH_TURN = 25
FH_FRAC_TURN = 0.09
ALL_FEET_DOWN_TIME_FRAC =0.12 # (one half of fraction of time both feet are on ground
                                    # simultaneously during IK walking
ALL_FEET_DOWN_TIME_FRAC_TURNING = 0.12 # (one half of fraction of time both feet are on ground
                                    # simultaneously during IK walking
#Transition fraction is total fraction of time between down and full up.
TRANSITION_FRAC = ALL_FEET_DOWN_TIME_FRAC + 0.28
TRANSITION_FRAC_TURNING = ALL_FEET_DOWN_TIME_FRAC_TURNING + 0.28

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
#          |numa |    
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
#         /          \                 Coax servo        
#        /            \              /                    
#     4 /              \            /                     
#      |                \        | L                      
#      |                 \_______|1        ___            
#      |                  2      |_______    |            
#      |                                     |- bodyH 
#      |5                                  __|            
#
#
#      |___________legLen________|

# Leg constants for Numa1
#defaultbodyH = 150 #130 + 20 # mm  # Numa
defaultbodyH = 88 # Numa2

#defaultL0 = 105 + 5 # mm - pretty close to actual...
#defaultL12 = 58
#defaultL23 = 63
#defaultL34 = 67
#defaultL45 = 57

#Leg ServoPos Offsets
#  this is in units of 10bit position of 300 deg range
#  currently not used... manually entered values are in
#offsetServo1 = 0
#offsetServo2 = -237 # 70
#offsetServo3 = -70
#offsetServo4 = 0*20 #39 per CAD with original feet.

class Gaits():
    """Class for generating servo angles for walking and turning.

    Scope is over-large, as it include a generic leg IK solver.
    """
    def __init__(self,
                 #L0=defaultL0,
                 #L12=defaultL12,
                 #L23=defaultL23,
                 #L34=defaultL34,
                 #L45=defaultL45,
                 leg_geom,
                 leg1, leg2, leg3, leg4,
                 bodyH=defaultbodyH,  # Leave this one open to being dynamic
                ):
        self.L0 = leg_geom.L0
        self.L12 = leg_geom.L12
        self.L23 = leg_geom.L23
        self.L34 = leg_geom.L34
        self.L45 = leg_geom.L45
        self.bodyH = bodyH
        self.v12 = [self.L12, 0]  # constant (until we implement per-servo body height)
        self.v23 = [0, 0]
        self.v34 = [0, 0]

        # Leg constants
        #sqL12 = L12 * L12  # unused
        self.sqL23 = self.L23 * self.L23
        self.sqL34 = self.L34 * self.L34
        #sqL45 = L45 * L45

        self.leg_geom = leg_geom
        self.leg1 = leg1
        self.leg2 = leg2
        self.leg3 = leg3
        self.leg4 = leg4

        # TODO safe initial values?
        self.s11pos = 511
        self.s21pos = 511
        self.s31pos = 511
        self.s41pos = 511
        self.s12pos = 511
        self.s22pos = 511
        self.s32pos = 511
        self.s42pos = 511
        self.s13pos = 0
        self.s23pos = 0
        self.s33pos = 0
        self.s43pos = 0
        self.s14pos = 511
        self.s24pos = 511
        self.s34pos = 511
        self.s44pos = 511

        self.initTrig()

    def initTrig(self):
        # We store these sines/cosines of the shoulder servo center angles
        self.cos_servo11Ang = cos(self.leg1.s1_center_angle * pi / 180.)
        self.sin_servo11Ang = sin(self.leg1.s1_center_angle * pi / 180.)
        self.cos_servo21Ang = cos(self.leg2.s1_center_angle * pi / 180.)
        self.sin_servo21Ang = sin(self.leg2.s1_center_angle * pi / 180.)
        self.cos_servo31Ang = cos(self.leg3.s1_center_angle * pi / 180.)
        self.sin_servo31Ang = sin(self.leg3.s1_center_angle * pi / 180.)
        self.cos_servo41Ang = cos(self.leg4.s1_center_angle * pi / 180.)
        self.sin_servo41Ang = sin(self.leg4.s1_center_angle * pi / 180.)
        return


    def walk_code(self, loopLength, half_loopLength, travRate, double_travRate, now1, now2, now3, now4, ang_dir, curve_dir=0):
        # OPT: self added for jupyter
        # Get height of each pair of feet
        self.footH13 = calc_foot_h(now2, FH, ALL_FEET_DOWN_TIME_FRAC, half_loopLength, TRANSITION_FRAC, FH_FRAC)
        self.footH24 = calc_foot_h(now3, FH, ALL_FEET_DOWN_TIME_FRAC, half_loopLength, TRANSITION_FRAC, FH_FRAC)

        # Now change the now variables so that they track a triangle wave instead of a sawtooth wave
        if now2 >= half_loopLength:           # Goes from 0ms...5000ms
            hnow2 = loopLength - now2       # then 5000ms...0ms
        if now3 >= half_loopLength:
            hnow3 = loopLength - now3
        if now1 >= half_loopLength:
            hnow1 = loopLength - now1
        if now4 >= half_loopLength:
            hnow4 = loopLength - now4

        #/ ///////////////////
        #/ Now we use the travel direction to calculate leg lengths and coax angles.
        #/ ///////////////////

        direction = ang_dir * pi / 180 # radians
        cdir = cos(direction)
        sdir = sin(direction)

        # trav values are calculated at each time step. Result is range from -travRate to +travRate
        # represent offset of foot position in walking direction from standing point of foot.
        # travN is vector length in walking direction to offset default foot position by.
        # it's magnitude ranges between +/- travRate
        # TODO we throw some -1 offsets onto legs 1 and 4... derive why this is...
        self.trav2 = ( double_travRate * (now2 / half_loopLength) ) - travRate
        self.trav3 = ( double_travRate * (now3 / half_loopLength) ) - travRate
        self.trav4 = -1 * ( (double_travRate * (now4 / half_loopLength) ) - travRate )
        self.trav1 = -1 * ( (double_travRate * (now1 / half_loopLength) ) - travRate )

        # curve_dir is -1, 0 or 1; Corresponds with sign of walkH from joystick
        if curve_dir < 0:
            self.trav2 *= 0.15
            self.trav3 *= 0.15
        elif curve_dir > 0:
            self.trav1 *= 0.15
            self.trav4 *= 0.15

        # Compute sines and cosines of trav#...
        # These are x and y components for offsetting foot position from default foot position.
        trav_cdir1 = self.trav1 * cdir
        trav_sdir1 = self.trav1 * sdir

        trav_cdir2 = self.trav2 * cdir
        trav_sdir2 = self.trav2 * sdir

        trav_cdir3 = self.trav3 * cdir
        trav_sdir3 = self.trav3 * sdir

        trav_cdir4 = self.trav4 * cdir
        trav_sdir4 = self.trav4 * sdir

    #def doLegKinem(self, trav_cdir, trav_sdir,
    #         footH, cos_s1Ang=None, sin_s1Ang=None, debug=0):
        s12ang, s13ang, s14ang = self.doLegKinem(trav_cdir1, trav_sdir1, self.footH13, self.cos_servo11Ang, self.sin_servo11Ang)
        s22ang, s23ang, s24ang = self.doLegKinem(trav_cdir2, trav_sdir2, self.footH24, self.cos_servo21Ang, self.sin_servo21Ang)
        s32ang, s33ang, s34ang = self.doLegKinem(trav_cdir3, trav_sdir3, self.footH13, self.cos_servo31Ang, self.sin_servo31Ang)
        s42ang, s43ang, s44ang = self.doLegKinem(trav_cdir4, trav_sdir4, self.footH24, self.cos_servo41Ang, self.sin_servo41Ang, debug=1)

        # Calculate coax servo positions as combination of trav vector and default leg position vector
        # Note trav_sdir/cdir -> trav_[sdir|cdir]2 and trav_sdir/cdir[3|4]->trav_[sdir|cdir]3
        s11ang = atan2(self.L0 * self.sin_servo11Ang + trav_cdir1,
                          self.L0 * self.cos_servo11Ang + trav_sdir1) - self.leg1.s1_center_radians
        s21ang = atan2(self.L0 * self.sin_servo21Ang + trav_cdir2,
                          self.L0 * self.cos_servo21Ang + trav_sdir2) - self.leg2.s1_center_radians
        s31ang = atan2(self.L0 * self.sin_servo31Ang + trav_cdir3,
                          self.L0 * self.cos_servo31Ang + trav_sdir3) - self.leg3.s1_center_radians
        s41ang = atan2(self.L0 * self.sin_servo41Ang + trav_cdir4,
                          self.L0 * self.cos_servo41Ang + trav_sdir4) - self.leg4.s1_center_radians

        self.s11pos, self.s12pos, self.s13pos, self.s14pos = \
                self.leg1.get_pos_from_radians(s11ang, s12ang, s13ang, s14ang)
        self.s21pos, self.s22pos, self.s23pos, self.s24pos = \
                self.leg2.get_pos_from_radians(s21ang, s22ang, s23ang, s24ang)
        self.s31pos, self.s32pos, self.s33pos, self.s34pos = \
                self.leg3.get_pos_from_radians(s31ang, s32ang, s33ang, s34ang)
        self.s41pos, self.s42pos, self.s43pos, self.s44pos = \
                self.leg4.get_pos_from_radians(s41ang, s42ang, s43ang, s44ang)


    def turn_code(self, turn_dir, loopLength, half_loopLength, now1, now2, now3, now4):
        """Implement turning by modifying foot heights and turning coax servo"""

        # TODO verify calc_foot_h() takes sawtooth wave and not triangle wave...
        self.footH13 = calc_foot_h(now1, FH_TURN, ALL_FEET_DOWN_TIME_FRAC_TURNING, half_loopLength, TRANSITION_FRAC_TURNING, FH_FRAC_TURN)
        self.footH24 = calc_foot_h(now4, FH_TURN, ALL_FEET_DOWN_TIME_FRAC_TURNING, half_loopLength, TRANSITION_FRAC_TURNING, FH_FRAC_TURN)

        # Now change the now2 and now3 so that they track a triangle wave instead of a sawtooth wave
        if now2 >= half_loopLength:         # Goes from 0ms...5000ms
            now2 = loopLength - now2     # then 5000ms...0ms
        if now3 >= half_loopLength:         # Goes from 0ms...5000ms
            now3 = loopLength - now3     # then 5000ms...0ms
        # now1 and now4 are identical to now3 and now2 respectively so we just use those
        turn_angle13 = pi/180 * TURN_ANGLE*(2*now3/loopLength - 0.5)
        turn_angle24 = pi/180 * TURN_ANGLE*(2*now2/loopLength - 0.5)

        print("\nfootH13: %d" % (self.footH13), "footH24: %d" % (self.footH24), "now3:", now3, "loopLength:", loopLength, turn_angle13, TURN_ANGLE)

    #   self.doLegKinem(cos_s1Ang, sin_s1Ang, trav_cdir, trav_sdir, footH, debug)
        s12ang, s13ang, s14ang = self.doLegKinem(0, 0, self.footH13)
        s22ang, s23ang, s24ang = self.doLegKinem(0, 0, self.footH24)
        s32ang, s33ang, s34ang = self.doLegKinem(0, 0, self.footH13)
        s42ang, s43ang, s44ang = self.doLegKinem(0, 0, self.footH24, debug=1)

        s11ang = turn_dir * turn_angle13
        s21ang = turn_dir * turn_angle24
        s31ang = turn_dir * turn_angle13
        s41ang = turn_dir * turn_angle24

        self.s11pos, self.s12pos, self.s13pos, self.s14pos = \
                self.leg1.get_pos_from_radians(s11ang, s12ang, s13ang, s14ang)
        self.s21pos, self.s22pos, self.s23pos, self.s24pos = \
                self.leg2.get_pos_from_radians(s21ang, s22ang, s23ang, s24ang)
        self.s31pos, self.s32pos, self.s33pos, self.s34pos = \
                self.leg3.get_pos_from_radians(s31ang, s32ang, s33ang, s34ang)
        self.s41pos, self.s42pos, self.s43pos, self.s44pos = \
                self.leg4.get_pos_from_radians(s41ang, s42ang, s43ang, s44ang)

    # Method does a few things depending on whether walking or turning, then invokes rest of IK
    # TODO seems I could split out the calculation of leg length from the calculation of
    # individual segment's relative angles? So far I made the vectors class level for access
    def doLegKinem(self, trav_cdir, trav_sdir,
             footH, cos_s1Ang=None, sin_s1Ang=None, debug=0):

        # Turning (see turn_code())
        if cos_s1Ang is None and sin_s1Ang is None:
            legLen = 1.00 * self.L0  # Historicially, I tried scaled this value for turning gait.
        else:
            # Top down x-y plane; lenx and leny form the vector of the leg, from which we get length
            # Note: cos_s1Ang and sin_s1Ang do not change.
            # Yes, trav_sdir goes to lenx and etc. (per measuring angle from 0 deg == forward, CCW)
            lenx = self.L0 * cos_s1Ang + trav_sdir
            leny = self.L0 * sin_s1Ang + trav_cdir

            # LEG LENGTH (length of projection of leg onto floor plane)
            legLen = sqrt( lenx * lenx + leny * leny ) # current length of leg

        return self.genericLegKinem(footH, legLen, debug=0)

    # Method does IK calculations for the non-coax parts of a leg. returns joint angles in radians
    def genericLegKinem(self, footH, legLen, debug=0):
        """
        """
        # TODO(enhancment) to support per-servo bodyH, will need to calculate v12 differently
        # x y are side view of leg
        len24x = legLen - self.L12
        len24y = self.bodyH - (self.L45 + footH) # body height minus height of point 4
        L24 = sqrt(len24x * len24x + len24y * len24y) # Length between points 2 and 4 in leg
        sqL24 = L24*L24

        ## Angles from which we find joint 3's x,y location and thus vectors 23 and 34
        # float angle between horizontal and hypotenuse (L24)
        alph1 = acos(len24x/L24)
        # We use len24y's sign because angle 3-4-horizontal is either alph2 + alph1 or alph2 - alph1
        # depending on whether point 4 is below or above point 2, respectively.
        # We also do this with alph3 but end up subtracting alph1
        alph1 = copysign(alph1, len24y)

        # To get angles of 2-3-4 triangle... Law of cosines: cos(C) = (a a + b b - c c) / (2 a b)
        # alph2 is the angle of 3-4-2
        alph2 = acos( (self.sqL34 + sqL24 - self.sqL23) / ( 2 * self.L34 * L24 ) )
        # alph3 is the angle of 3-2-4
        alph3 = acos( (self.sqL23 + sqL24 - self.sqL34) / ( 2 * self.L23 * L24 ) )

        # DEBUG: Copy to class variables so we can print them elsewhere
        self.alph1 = alph1
        self.alph2 = alph2
        self.alph3 = alph3

        # Sine and cosine of angle between 4-3 and horizontal
        c_alph1plus2 = cos(alph2 + alph1)
        s_alph1plus2 = sin(alph2 + alph1)
        #
        c_alph3minus1 = cos(alph3 - alph1)
        s_alph3minus1 = sin(alph3 - alph1)

        ## Leg vectors from which we'll calculate joint angles
        v23 = [self.L23 * c_alph3minus1,
               self.L23 * s_alph3minus1]
        v34 = [self.L34 * c_alph1plus2,
               -self.L34 * s_alph1plus2]  # TODO bad assumption if we're raising the leg into the air lol
        self.v23 = v23
        self.v34 = v34

        # TODO refactor if we use this. It probably doesn't make sense right now.
        # To avoid problems with sign of angle being incorrectly handled by v2D_Angle
        # ... we instead compare a near right angle and later subtract Pi/2 radians
        v45 = [-1 , 0]

        s2rad, s3rad, s4rad = ((v2d_AngleRadians(self.v12, self.v23)),
                               (v2d_AngleRadians(self.v23, self.v34)),
                               (v2d_AngleRadians(self.v34, v45) - pi/2),)

        if PRINT_DEBUG_IK and debug == 1:
            print("_ %f *%f %f %d *%d %d ", v2d_AngleRadians(v23, self.v12),
                    v2d_AngleRadians(v23, v34),
                    v2d_AngleRadians(v34, v45),
                    s2rad,
                    s3rad,
                    s4rad
                    )
        return s2rad, s3rad, s4rad

# For alternate version of v2d_AngleRadians
#def v2d_Length(vec):
#    #if vec *is* normalised:
#    #    return 1.0
#    return sqrt(vec[0]*vec[0] + vec[1]*vec[1])
#
## Find the dot product of two vectors
#def v2d_DotProduct(v1, v2):
#    #
#    return v1[0] * v2[0] + v1[1] * v2[1]

def v2d_AngleRadians(v1, v2):
    # Calculate the change in angle going from V1 to V2, preserving sign.
    a2 = atan2(v2[1], v2[0])
    a1 = atan2(v1[1], v1[0])
    return a2 - a1
    # Alternate method that doesn't preserve sign
    # Calculate the dTheta going from v1 to v2 using:
    # angle2 - angle 1
    # Get the absolute angle between 0 and 180?
    #vDot = v2d_DotProduct(v1, v2) / (v2d_Length(v1) * v2d_Length(v2))
    ## Uhhh
    #if vDot < -1.0:
    #    vDot = -1.0
    #elif vDot >  1.0:
    #    vDot = 1.0

    #return acos(vDot)
