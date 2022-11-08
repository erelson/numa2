# NOTE This is not used!; see IK.py




















#include "hardware.h"
#include <math.h>
#include "Maths/Vector2D.h"
# #include <Gait/GaitRunner.h>

from math import cos, sin, pi

# Two functions defined here:
# - walkCode
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


    #def walkCode(int16_t loopLength, int16_t half_loopLength,
    #            int16_t travRate, int16_t double_travRate):
    def walkCode(self, loopLength, half_loopLength,
                travRate, double_travRate):
        uint8_t footH13 = 0
        uint8_t footH24 = 0
        
        footH13 = calc_foot_h(now2, FH, ALL_FEET_DOWN_TIME_FRAC, half_loopLength, TRANSITION_FRAC, FH_FRAC)
        footH24 = calc_foot_h(now3, FH, ALL_FEET_DOWN_TIME_FRAC, half_loopLength, TRANSITION_FRAC, FH_FRAC)
        
        #Now change the now variables so that they track a triangle wave instead of a sawtooth wave
        if now2 >= half_loopLength:                # Goes from 0ms...5000ms
            now2 = loopLength - now2            # then 5000ms...0ms
        if now3 >= half_loopLength:                # Goes from 0ms...5000ms
            now3 = loopLength - now3            # then 5000ms...0ms
        if now1 >= half_loopLength:                # Goes from 0ms...5000ms
            now1 = loopLength - now1            # then 5000ms...0ms
        if now4 >= half_loopLength:                # Goes from 0ms...5000ms
            now4 = loopLength - now4            # then 5000ms...0ms
    
        #print("%u\t%u\n"  (footH13, footH24))
        
        #/ ///////////////////
        #/ Now we use the travel direction to calculate leg lengths and coax angles.
        #/ ///////////////////
    
        direction = ang_dir * pi / 180 #Radians
        cdir = cos(direction)
        sdir = sin(direction)
    
        #trav values are calculated at each time step
        # represent offset of foot position in walking direction from standing point of foot.
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
        
    #    self.doLegKinematics( myT, mys2pos, mys3pos, mys4pos, posSwap,
    #                     cos_s1Ang, sin_s1Ang, my_trav_cdir, my_trav_sdir,
    #                     myFootH, myTrav, debug)
        self.doLegKinematics(now1, s12pos, s13pos, s14pos, 1, cos_servo11Ang, sin_servo11Ang,
                        trav_cdir1, trav_sdir1, footH13, trav1, 0)
        self.doLegKinematics(now2, s22pos, s23pos, s24pos, -1, cos_servo21Ang, sin_servo21Ang,
                        trav_cdir2, trav_sdir2, footH24, trav2, 0)
        self.doLegKinematics(now3, s32pos, s33pos, s34pos, 1, cos_servo31Ang, sin_servo31Ang,
                        trav_cdir3, trav_sdir3, footH13, trav3, 0)
        self.doLegKinematics(now4, s42pos, s43pos, s44pos, -1, cos_servo41Ang, sin_servo41Ang,
                        trav_cdir4, trav_sdir4, footH24, trav4, 1)
        
        if PRINT_DEBUG_IK:
            print("%u %u " % (s42pos, s43pos))
        
        
        #/Calculate coax servo positions
        #Note trav_sdir/cdir -> trav_sdir/cdir2 and trav_sdir/cdir34->trav_sdir/cdir3
        s31pos = 511 + (195.3786 * atan2(100 * sin_servo31Ang + trav_cdir3, 100 * cos_servo31Ang + trav_sdir3) )
    
        s41pos = 511 + (-613.8 + 195.3786 * atan2(100 * sin_servo41Ang + trav_cdir2, 100 * cos_servo41Ang + trav_sdir2) )
    
        s11pos = 511 + ( 613.8 + 195.3786 * atan2(100 * sin_servo11Ang + trav_cdir3, 100 * cos_servo11Ang + trav_sdir3) )
        
        s21pos = 511 + (195.3786 * atan2(100 * sin_servo21Ang + trav_cdir2, 100 * cos_servo21Ang + trav_sdir2) )
    
    
    #////////////
    #def turnCode(short my_turn_dir, int16_t loopLength, int16_t half_loopLength):
    def turnCode(self, short my_turn_dir, loopLength, half_loopLength):
        # uint8_ts
        footH13 = 0
        footH24 = 0
        
        #short turn_dir = 1
        #if turnleft: turn_dir = -1
        
        footH13 = calc_foot_h(now1, FH_TURN, ALL_FEET_DOWN_TIME_FRAC_TURNING, half_loopLength, TRANSITION_FRAC_TURNING, FH_FRAC_TURN)
        footH24 = calc_foot_h(now4, FH_TURN, ALL_FEET_DOWN_TIME_FRAC_TURNING, half_loopLength, TRANSITION_FRAC_TURNING, FH_FRAC_TURN)
        
        print("\n%d" % (footH24))
        
        
        #///Now change the now variables so that they track a triangle wave instead of a sawtooth wave
        if now2 >= half_loopLength:                # Goes from 0ms...5000ms
            now2 = loopLength - now2            # then 5000ms...0ms
        
        if now3 >= half_loopLength:                # Goes from 0ms...5000ms
            now3 = loopLength - now3            # then 5000ms...0ms
        #/
        #/if now1 >= half_loopLength:                // Goes from 0ms...5000ms
        #/    now1 = loopLength - now1            // then 5000ms...0ms
        #/if now4 >= half_loopLength:                // Goes from 0ms...5000ms
        #/    now4 = loopLength - now4            // then 5000ms...0ms
        
        s31pos = 511 + ( 45 + s31Aoff + my_turn_dir * TURN_ANGLE*(2*(float)now3/loopLength - 0.5))*1024.0/300.0
        s41pos = 511 + (-45 + s41Aoff + my_turn_dir * TURN_ANGLE*(2*(float)now2/loopLength - 0.5))*1024.0/300.0
        s11pos = 511 + ( 45 + s11Aoff + my_turn_dir * TURN_ANGLE*(2*(float)now3/loopLength - 0.5))*1024.0/300.0
        s21pos = 511 + (-45 + s21Aoff + my_turn_dir * TURN_ANGLE*(2*(float)now2/loopLength - 0.5))*1024.0/300.0
        
    #        self.doLegKinematics( myT, mys2pos, mys3pos, mys4pos, posSwap, \
    #     cos_s1Ang, sin_s1Ang, my_trav_cdir, my_trav_sdir, \
    #     myFootH, myTrav, debug)
        self.doLegKinematics(now3, s12pos, s13pos, s14pos, 1, 0, 0, 0, 0, footH13, 0, 0)
        self.doLegKinematics(now2, s22pos, s23pos, s24pos, -1, 0, 0, 0, 0, footH24, 0, 0)
        self.doLegKinematics(now3, s32pos, s33pos, s34pos, 1, 0, 0, 0, 0, footH13, 0, 0)
        self.doLegKinematics(now4, s42pos, s43pos, s44pos, -1, 0, 0, 0, 0, footH24, 0, 1)
                    
    #/////// end turnCode()
