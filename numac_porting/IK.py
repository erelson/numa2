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
    # Important note, `now` must be in the range from 0 to 2 * half_loopLength

    if now >= (2.0 - time_down_frac) * half_loopLength:
        footH = int(height_frac * foot_h_max * (2.0 * half_loopLength - now) / \
            (time_down_frac * half_loopLength))
    elif now >= (2.0 - transition_frac) * half_loopLength:
        footH = int((height_frac * foot_h_max + (1.0 - height_frac) * foot_h_max) * \
            ((2.0 - time_down_frac) * half_loopLength - now) / \
            ((transition_frac - time_down_frac) * half_loopLength))
    # Steady at max height
    elif now >= (1 + transition_frac) * half_loopLength:
        footH = foot_h_max
    elif now >= half_loopLength * (1.0 + time_down_frac):
        footH = int(height_frac * foot_h_max + (1.0 - height_frac) * foot_h_max * \
            ((now - half_loopLength * (1.0 + time_down_frac)) / \
            ((transition_frac - time_down_frac) * half_loopLength)))
    elif now >= half_loopLength:
        footH = int(height_frac * foot_h_max*(now - half_loopLength) / \
            (time_down_frac * half_loopLength))
    else:
        footH = 0

    return footH


# Two functions defined here:
# - walkCode: takes params defining sawtooth (?) time wave and point on wave; returns ...
# - turnCode

TURN_ANGLE = 25 # angle sweep of legs when turning??
TURN_INCR = 10
FH = 25  # maximum foot height, (mm)
#FH_FRAC is the fraction of time over which foot height slowly increases from zero
FH_FRAC = 0.07
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

# Leg constants for Numa1
defaultbodyH = 150 #130 + 20 # mm

defaultL0 = 105 + 5 # mm - pretty close to actual...
defaultL12 = 58
defaultL23 = 63
defaultL34 = 67
defaultL45 = 57

#Leg ServoPos Offsets
#  this is in units of 10bit position of 300 deg range
#  currently not used... manually entered values are in
offsetServo1 = 0
offsetServo2 = -237 # 70
offsetServo3 = -70
offsetServo4 = 0*20 #39 per CAD with original feet.

class Gaits():
    """Class for generating servo angles for walking and turning.

    Scope is over-large, as it include a generic leg IK solver.
    """
    def __init__(self,
                 L0=defaultL0,
                 L12=defaultL12,
                 L23=defaultL23,
                 L34=defaultL34,
                 L45=defaultL45,
                 bodyH=defaultbodyH,
                ):
        self.L0 = L0
        self.L12 = L12
        self.L23 = L23
        self.L34 = L34
        self.L45 = L45
        self.bodyH = bodyH
        # TODO weird negative; origin at joint 2 axis?
        self.v12 = [-1., 0]  # constant
        self.v23 = [0, 0]
        self.v34 = [0, 0]

        self.initTrig()
        # Leg constants
        #sqL12 = L12 * L12  # unused
        self.sqL23 = L23 * L23
        self.sqL34 = L34 * L34
        #sqL45 = L45 * L45

        # TODO safe initial values?
        self.s11pos = 0
        self.s21pos = 0
        self.s31pos = 0
        self.s41pos = 0
        self.s12pos = 0
        self.s22pos = 0
        self.s32pos = 0
        self.s42pos = 0
        self.s13pos = 0
        self.s23pos = 0
        self.s33pos = 0
        self.s43pos = 0
        self.s14pos = 0
        self.s24pos = 0
        self.s34pos = 0
        self.s44pos = 0

    def initTrig(self):
        # # See WalkingOmni.nb
        # Center angles for coax servo of each leg.plus test offsets.
        # Measured from 0deg = front of bot  --------- this doesn't look right.
        #/But are the
        s11A0 = 225
        self.s11Aoff = 10
        s21A0 = -45 #315 //Hmmm no idea here....
        self.s21Aoff = -10
        s31A0 = 45
        self.s31Aoff = 0
        s41A0 = 135
        self.s41Aoff = 10
        # Center angles for each leg.
        servo11Ang = pi / 180 * (s11A0 + self.s11Aoff) # radians
        servo21Ang = pi / 180 * (s21A0 + self.s21Aoff)
        servo31Ang = pi / 180 * (s31A0 + self.s31Aoff)
        servo41Ang = pi / 180 * (s41A0 + self.s41Aoff)
        #
        self.cos_servo11Ang = cos(servo11Ang)
        self.sin_servo11Ang = sin(servo11Ang)
        self.cos_servo21Ang = cos(servo21Ang)
        self.sin_servo21Ang = sin(servo21Ang)
        self.cos_servo31Ang = cos(servo31Ang)
        self.sin_servo31Ang = sin(servo31Ang)
        self.cos_servo41Ang = cos(servo41Ang)
        self.sin_servo41Ang = sin(servo41Ang)
        return

    # TODO maybe pull servoX1Ang value calcs out of initTrig so we can get them in test
    # code easily?
    #def calc_center_angles(self)


    def walkCode(self, loopLength, half_loopLength, travRate, double_travRate, now1, now2, now3, now4, ang_dir):

        # OPT: self added for jupyter
        # Get height of each pair of feet
        self.footH13 = calc_foot_h(now2, FH, ALL_FEET_DOWN_TIME_FRAC, half_loopLength, TRANSITION_FRAC, FH_FRAC)
        self.footH24 = calc_foot_h(now3, FH, ALL_FEET_DOWN_TIME_FRAC, half_loopLength, TRANSITION_FRAC, FH_FRAC)

        #Now change the now variables so that they track a triangle wave instead of a sawtooth wave
        if now2 >= half_loopLength:                # Goes from 0ms...5000ms
            now2 = loopLength - now2            # then 5000ms...0ms
        if now3 >= half_loopLength:                # Goes from 0ms...5000ms
            now3 = loopLength - now3            # then 5000ms...0ms
        if now1 >= half_loopLength:                # Goes from 0ms...5000ms
            now1 = loopLength - now1            # then 5000ms...0ms
        if now4 >= half_loopLength:                # Goes from 0ms...5000ms
            now4 = loopLength - now4            # then 5000ms...0ms

        #print("%u\t%u\n"  (self.footH13, self.footH24))

        #/ ///////////////////
        #/ Now we use the travel direction to calculate leg lengths and coax angles.
        #/ ///////////////////

        direction = ang_dir * pi / 180 #Radians
        cdir = cos(direction)
        sdir = sin(direction)

        # trav values are calculated at each time step
        # represent offset of foot position in walking direction from standing point of foot.
        # travN is vector length in walking direction to offset foot position by
        trav2 = ( double_travRate * (now2 / half_loopLength) ) - travRate
        trav3 = ( double_travRate * (now3 / half_loopLength) ) - travRate
        trav4 = ( double_travRate * (now4 / half_loopLength) ) - travRate
        trav1 = ( double_travRate * (now1 / half_loopLength) ) - travRate

        # Compute sines and cosines of trav#...
        # These are x and y components for offsetting foot position from default foot position.
        trav_cdir2 = trav2 * cdir
        trav_sdir2 = trav2 * sdir

        trav_cdir1 = trav1 * cdir
        trav_sdir1 = trav1 * sdir

        trav_cdir3 = trav3 * cdir
        trav_sdir3 = trav3 * sdir

        trav_cdir4 = trav4 * cdir
        trav_sdir4 = trav4 * sdir

    #    self.doLegKinem( myT, posSwap,
    #                     cos_s1Ang, sin_s1Ang, trav_cdir, trav_sdir,
    #                     footH, myTrav, debug)
        self.s12pos, self.s13pos, self.s14pos = \
                self.doLegKinem(now1, 1, self.cos_servo11Ang, self.sin_servo11Ang,
                                     trav_cdir1, trav_sdir1, self.footH13, 0)
        self.s22pos, self.s23pos, self.s24pos = \
                self.doLegKinem(now2, -1, self.cos_servo21Ang, self.sin_servo21Ang,
                                     trav_cdir2, trav_sdir2, self.footH24, 0)
        self.s32pos, self.s33pos, self.s34pos = \
                self.doLegKinem(now3, 1, self.cos_servo31Ang, self.sin_servo31Ang,
                                     trav_cdir3, trav_sdir3, self.footH13, 0)
        self.s42pos, self.s43pos, self.s44pos = \
                self.doLegKinem(now4, -1, self.cos_servo41Ang, self.sin_servo41Ang,
                                     trav_cdir4, trav_sdir4, self.footH24, 1)

        #if PRINT_DEBUG_IK:
        #    print("%u %u " % (s42pos, s43pos))


        # Calculate coax servo positions
        # Note trav_sdir/cdir -> trav_sdir/cdir2 and trav_sdir/cdir34->trav_sdir/cdir3
        self.s31pos = 511 + (195.3786 * atan2(100 * self.sin_servo31Ang + trav_cdir3, 100 * self.cos_servo31Ang + trav_sdir3) )
        self.s41pos = 511 + (-613.8 + 195.3786 * atan2(100 * self.sin_servo41Ang + trav_cdir2, 100 * self.cos_servo41Ang + trav_sdir2) )
        self.s11pos = 511 + ( 613.8 + 195.3786 * atan2(100 * self.sin_servo11Ang + trav_cdir3, 100 * self.cos_servo11Ang + trav_sdir3) )
        self.s21pos = 511 + (195.3786 * atan2(100 * self.sin_servo21Ang + trav_cdir2, 100 * self.cos_servo21Ang + trav_sdir2) )


    #////////////
    #def turnCode(short turn_dir, int16_t loopLength, int16_t half_loopLength):
    def turn_code(self, turn_dir, loopLength, half_loopLength, now1, now2, now3, now4):
        # uint8_ts
        self.footH13 = 0
        self.footH24 = 0

        self.footH13 = calc_foot_h(now1, FH_TURN, ALL_FEET_DOWN_TIME_FRAC_TURNING, half_loopLength, TRANSITION_FRAC_TURNING, FH_FRAC_TURN)
        self.footH24 = calc_foot_h(now4, FH_TURN, ALL_FEET_DOWN_TIME_FRAC_TURNING, half_loopLength, TRANSITION_FRAC_TURNING, FH_FRAC_TURN)

        print("\n%d" % (self.footH24))


        #///Now change the now variables so that they track a triangle wave instead of a sawtooth wave
        if now2 >= half_loopLength:         # Goes from 0ms...5000ms
            now2 = loopLength - now2     # then 5000ms...0ms

        if now3 >= half_loopLength:         # Goes from 0ms...5000ms
            now3 = loopLength - now3     # then 5000ms...0ms
        # Don't need to change now1 and now4?
        #if now1 >= half_loopLength:         // Goes from 0ms...5000ms
        #    now1 = loopLength - now1     // then 5000ms...0ms
        #if now4 >= half_loopLength:         // Goes from 0ms...5000ms
        #    now4 = loopLength - now4     // then 5000ms...0ms

        self.s31pos = 511 + ( 45 + self.s31Aoff + turn_dir * TURN_ANGLE*(2*now3/loopLength - 0.5))*1024.0/300.0
        self.s41pos = 511 + (-45 + self.s41Aoff + turn_dir * TURN_ANGLE*(2*now2/loopLength - 0.5))*1024.0/300.0
        self.s11pos = 511 + ( 45 + self.s11Aoff + turn_dir * TURN_ANGLE*(2*now3/loopLength - 0.5))*1024.0/300.0
        self.s21pos = 511 + (-45 + self.s21Aoff + turn_dir * TURN_ANGLE*(2*now2/loopLength - 0.5))*1024.0/300.0

    #        self.doLegKinem( myT, s2pos, s3pos, s4pos, posSwap, \
    #     cos_s1Ang, sin_s1Ang, trav_cdir, trav_sdir, \
    #     footH, myTrav, debug)
        self.s12pos, self.s13pos, self.s14pos = self.doLegKinem(now3, 1, 0, 0, 0, 0, self.footH13, 0)
        self.s22pos, self.s23pos, self.s24pos = self.doLegKinem(now2, -1, 0, 0, 0, 0, self.footH24, 0)
        self.s32pos, self.s33pos, self.s34pos = self.doLegKinem(now3, 1, 0, 0, 0, 0, self.footH13, 0)
        self.s42pos, self.s43pos, self.s44pos = self.doLegKinem(now4, -1, 0, 0, 0, 0, self.footH24, 1)

    #/////// end turnCode()

    # Method does a few things depending on whether walking or turning, then invokes rest of IK
    # TODO seems I could split out the calculation of leg length from the calculation of
    # individual segment's relative angles? So far I made the vectors class level for access
    def doLegKinem(self, myT, posSwap, cos_s1Ang, sin_s1Ang, trav_cdir, trav_sdir,
             footH, debug=0):
        # posSwap determines which direction to offset servo position from centered.
        # Unneeded filtering?
        posSwap = posSwap if posSwap in [-1, 1] else 1

        # Turning (see turn_code())
        if cos_s1Ang == 0 and sin_s1Ang == 0:
            legLen = 1.05 * self.L0  # Apparently I'm making the legs stick out a little bit further when turning.
        else:
            # Top down x-y plane; lenx and leny form the vector of the leg, from which we get length
            # Note: cos_s1Ang and sin_s1Ang do not change.
            # Yes, trav_sdir goes to lenx and etc. (per measuring angle from 0 deg == forward, CCW)
            lenx = self.L0 * cos_s1Ang + trav_sdir
            leny = self.L0 * sin_s1Ang + trav_cdir

            # LEG LENGTH (length of projection of leg onto floor plane)
            legLen = sqrt( lenx * lenx + leny * leny ) # current length of leg
        #PRINT LEG LENGTH
        # print("%u,", myT)

        return self.genericLegKinem(posSwap, footH, legLen, debug=0)

    # Method does IK calculations for the non-coax parts of a leg. returns AX12 positions
    def genericLegKinem(self, posSwap, footH, legLen, debug=0):

        # now x y are side view of leg
        len24x = legLen - self.L12 #uint16_t
        len24y = self.bodyH - (self.L45 + footH) # body height minus height of point 4; could factor in foot angle
        L24 = sqrt(len24x * len24x + len24y * len24y) # Length btw points 2 and 4 in leg.
        sqL24 = L24*L24

        # float angle between horizontal and hypotenuse (L24)
        alph1 = acos(len24x/L24)
        # We use len24y's sign because angle 3-4-horizontal is either alph2 + alph1 or alph2 - alph1
        # depending on whether point 4 is below or above point 2, respectively.
        # We also do this with alph3 but end up subtracting alph1
        alph1 = copysign(alph1, len24y)

        #angle... Law of cosines: cos(C) = (a a + b b - c c) / (2 a b)
        # alph2 is the angle of 3-4-2
        alph2 = acos( (self.sqL34 + sqL24 - self.sqL23) / ( 2 * self.L34 * L24 ) )
        # alph3 is the angle of 3-2-4
        alph3 = acos( (self.sqL23 + sqL24 - self.sqL34) / ( 2 * self.L23 * L24 ) )

        #DEBUG
        self.alph1 = alph1
        self.alph2 = alph2
        self.alph3 = alph3

        if PRINT_DEBUG_IK and debug == 1:
            print("IK: %u _ %u %u %f %f  ", footH, len24x, len24y, L24, alph1*57.3)

        #///////////
        #Everything seems to be OK up to here
        #/////////

        # Sine and cosine of angle between 4-3 and horizontal
        c_alph1plus2 = cos(alph2 + alph1)
        s_alph1plus2 = sin(alph2 + alph1)
        c_alph3minus1 = cos(alph3 - alph1)
        s_alph3minus1 = sin(alph3 - alph1)

        # OLD:
        # Determine and create leg segment vectors
        # For diagram of math, see: .................. .nb
        #Orientation: Am I using the XY plane, not the (-X,Y) plane? YES

        # NOTE replaced with self.v12.
        # TODO weird negative; origin at joint 2 axis?

        # REDOING THE VECTORS HERE; Origin is below servo1, at ground height. Vectors go to the right; mirror of pic at top
        # gndpt = [0,0]
        # vgnd_1 = [0, bodyH]
        #v12 = [L12, 0] # Assuming always horiz, but measured from vertical.
        v12vert = [0, 1] # Just 90 degrees from v12, up
        #v23 = [self.L12 - (legLen - self.L34 * c_alph1plus2),
        #v23 = [legLen - self.L12 - self.L34 * c_alph1plus2),
        v23 = [self.L23 * c_alph3minus1,
               self.L23 * s_alph3minus1]
               #bodyH - ((footH + self.L45) + self.L34 * s_alph1plus2)]
               #bodyH - ((footH + self.L45) + self.L34 * s_alph1plus2)]
        #v34 = [-self.L34 * c_alph1plus2, # legLen - self.L12 - v23[0]
        v34 = [self.L34 * c_alph1plus2, # legLen - self.L12 - v23[0]
               self.L34 * s_alph1plus2]
        #self.v12 = v12
        self.v23 = v23
        self.v34 = v34

   #     #v12 = [-1, 0] # Assuming always horiz, but measured from vertical.
   #     v12vert = [0, 1]
   #     v23 = [self.L12 - (legLen - self.L34 * c_alph1plus2),
   #            bodyH - ((footH + self.L45) + self.L34 * s_alph1plus2)]
   #     v34 = [-self.L34 * c_alph1plus2,
   #            self.L34 * s_alph1plus2]
   #     #self.v12 = v12
   #     self.v23 = v23
   #     self.v34 = v34
   #     #v12 = array('i', [-1, 0]) # Assuming always horiz, but measured from vertical.

        #v12vert = array('i', [0, 1])
        #v23 = array('f', [self.L12 -  (legLen - self.L34 * c_alph1plus2),
        #             bodyH - ((footH + self.L45) + self.L34 * s_alph1plus2) ])
        #v34 = array('f', [-self.L34 * c_alph1plus2,
        #             self.L34 * s_alph1plus2 ])

        # To avoid problems with sign of angle being incorrectly handled by v2D_Angle
        # ... we instead compare a near right angle and later subtract Pi/2 radians
        v45 = [-1 , 0]
        #v45 = array('i', [-1 , 0])

        # For calculating the angles, we want both vectors v23 and v32... so we make v32 by inverting v23.
        v32 = [-1 * v23[0], -1 * v23[1]]

# TODO redo these calculations to factor in my changed vectors?
        # Calculate the servo positions with scale factor 195.2 for radians ->  300deg/1022 scale
        #   .... Note that the offsetServo variables are sometimes negative.
        s2pos = 511 + posSwap * ( (pi/2.0 - v2d_AngleRadians(v23, v12vert) ) * 195.2 + offsetServo2) # -237 #OLD:70.08
        s3pos = 511 + posSwap * ((pi - v2d_AngleRadians(v34, v32)) * 195.2 + offsetServo3) #- 70.08)
        s4pos = 511 - posSwap * ((v2d_AngleRadians(v34, v45) - pi/2) * -195.2 + offsetServo4) #+ 39.19)
        #s4pos = 511 - posSwap * (v2d_AngleRadians(v34, v45) * 195.2 + offsetServo4) #+ 39.19)

        if PRINT_DEBUG_IK and debug == 1:
            print("_ %f *%f %f %d *%d %d ", v2d_AngleRadians(v23, self.v12),
                    v2d_AngleRadians(v34, v32),
                    v2d_AngleRadians(v34, v45),
                    s2pos,
                    s3pos,
                    s4pos
                    )
        return s2pos, s3pos, s4pos


def v2d_Length(vec):
    #if vec *is* normalised:
    #    return 1.0
    return sqrt(vec[0]*vec[0] + vec[1]*vec[1])

# Find the dot product of two vectors
def v2d_DotProduct(v1, v2):
    #
    return v1[0] * v2[0] + v1[1] * v2[1]

def v2d_AngleRadians(v1, v2):
    #
    vDot = v2d_DotProduct(v1, v2) / (v2d_Length(v1) * v2d_Length(v2))
    if vDot < -1.0:
        vDot = -1.0
    elif vDot >  1.0:
        vDot = 1.0

    return acos(vDot)
