#include "hardware.h"
#include <math.h>
#include "Maths/Vector2D.h"
#include <Gait/GaitRunner.h>

#include "HeaderDefs.h"

#include "servoPosConsts.h"
#include "timingConsts.h"

#include "Commander.h"

#include "gait2.h"
##endif

from math import pi, sqrt
import struct

import utime

import ax
from helpers import clamp
from init import initTrig, myServoReturnLevels, myServoSpeeds, initServoLims
from commander import CommanderRx

#short turn_loops
#short turn_dir
#int8_t standing

#define True 1
#define False 0

##define LISTEN UART2toXbee38400
##define LISTEN uart1


##define TURNG8 G8_ANIM_TURNLEFT
#define TURNG8 G8_ANIM_TURNSLOW

PROG_LOOP_TIME = 19500 # in microseconds

#define AGITATE_TIME 1000 #milliseconds - how long it takes to do an agitate sequence.

#define USE_ONE_SPEED 0
#define THE_ONE_SPEED 3
#define THE_TURN_SPEED 7
#define MAX_WALK_SPD 5

# bitmasks for buttons array
BUT_R1 = 0x01
BUT_R2 = 0x02
BUT_R3 = 0x04
BUT_L4 = 0x08
BUT_L5 = 0x10
BUT_L6 = 0x20
BUT_RT = 0x40
BUT_LT = 0x80

PRINT_DEBUG_COMMANDER = True
PRINT_DEBUG_LOOP = False


PAN_CENTER = 511 + 153 # +153 is 45 deg offset for pan servo's mounting scheme.
TILT_CENTER = 511 + 95 # 4bar
PAN_MIN = PAN_CENTER - 4 * (52+30)
PAN_MAX = PAN_CENTER + 4 * (52+30)
                                    
#ACTUATOR_LIST PROGMEM all[] = {servo11.actuator,servo21.actuator,servo31.actuator,servo41.actuator,
#                        servo12.actuator,servo22.actuator,servo32.actuator,servo42.actuator,
#                        servo13.actuator,servo23.actuator,servo33.actuator,servo43.actuator,
#                        servo14.actuator,servo24.actuator,servo34.actuator,servo44.actuator}


#G8_RUNNER gait = MAKE_G8_RUNNER(all, animations)

# Binary vars
walk = False
turn = False
light = True
kneeling = False
flopCnt = 0
panic = False

# Command settings/interpretation variables (ints)
#currentAnim = 0
#playingAnim = -1

# IRcnt = 1

#/MATHEMATICA CODE
#/loopSpeed = 1000
#/Plot[65.536*loopSpeed/speed, {speed, 0, 128}, PlotRange -> {500, 4000}]
#const int g8loopSpeed = 1000
#int g8speed = 25
#int g8playbackDir = 1 # value should only ever be -1 or 1.
#int g8repeatCount = 0
#int ch = '!'
#int ch_old = '?'
#int do_gait = 1
#short guns_firing = False

#Setting default walking variables...
#int16_t loopLength = 0 # = 1800
#int16_t half_loopLength = 0 # = loopLength/2 #reduced the scope and made this local where its used...

#int16_t ang_dir = 0 #UNUSED?


class NumaMain(object):

    def __init__(self):
        from stm_uart_port import UART_Port
        from bus import Bus, BusError
        #bus = UART_Port(6, 38400)
        self.cmdrbus = UART_Port(1, 38400)
        self.axbus = Bus(UART_Port(2, 1000000), show=Bus.SHOW_PACKETS)
        #else:
        #    print("Unrecognized sysname: {0}".format(sysname))
        #    sys.exit()
    
        self.crx = CommanderRx()

        self.leg_ids = [11, 12, 13, 14, #servo11, servo21, servo31, servo41,
                        21, 22, 23, 24, #servo12, servo13, servo14, servo22,
                        31, 32, 33, 34, #servo23, servo24, servo32, servo33,
                        41, 42, 43, 44] #servo34, servo42, servo43, servo44]
        self.turret_ids = [51, 52] # pan, tilt
        self.all_ids = self.leg_ids + self.turret_ids

        self.servo51Min, self.servo51Max = PAN_CENTER - 4 * (52+30),  PAN_CENTER + 4 * (52+30)
        self.servo52Min, self.servo52Max = 511 - 4 * 31,              511 + 4 * 65


        self.gunbutton = False
        self.panicbutton = False
        self.infobutton = False
        self.pan_pos = PAN_CENTER
        self.tilt_pos = TILT_CENTER
        #self.agitbutton = False

        # Defaults?
        self.trav_rate_default = 25
        
        # Various?
        self.travRate = 0
        self.double_travRate = 0
        self.turnTimeOffset = 0

    def main(self):
        self.app_init_hardware()
        self.app_init_software()
        oldLoopStart = 0
        while True:
            #TODO timing?
            loopStart = utime.ticks_us()
            self.app_control(loopStart)
            loopEnd = utime.ticks_us()
            if PRINT_DEBUG_LOOP:
                print("%ld" % (loopStart - oldLoopStart))
                oldLoopStart = self.loopStart
            utime.sleep_us(PROG_LOOP_TIME - utime.ticks_diff(loopEnd, loopStart))


    # Initialise the hardware
    def app_init_hardware(self):
        #initHardware()
        # gaitRunnerInit(&gait)
        # act_setConnected(&antijam, False)  // Setting this later so servo centers initially.
        pass
    
    # Initialise the software
    # returns TICK_COUNT, usec
    # TODO this silly loop thing needs to be rewritten
    def app_init_software(self):
        print("It begins....")
        initTrig()
        
        #ax12SetID(&servo1, 1)
        
#        #Call gait for Standing
#        g8Stand() 
    
        myServoReturnLevels(self.axbus, all_ids=self.all_ids)
        print("ServoReturnLevelsSet!")
        initServoLims(self.axbus, self.all_ids)
        print("ServoLimsSet!")
        myServoSpeeds(self.axbus, self.leg_ids, self.turret_ids)
        print("ServoSpeedsSet!")
    
#        act_setConnected(&antijam, False)  # Stop microservo
        
        #Setting mathy initial values for walking
        self.loopLength = 1800 # ms (units???)
        #self.half_loopLength = self.loopLength / 2 #redundant
        self.travRate = self.trav_rate_default
        self.double_travRate = 2 * self.travRate
    
#        standing = 1
#        g8Stand()
    
        return 0
    
    # This is the main loop
    #TICK_COUNT appControl(LOOP_COUNT loopCount, TICK_COUNT loopStart):
    def app_control(self, loopStart):
    
        # Stop IK and g8s from coinciding... make Numa stop in place.
#        if walk == True and do_gait == True:
#            g8Stand() 
        
        # -------- Start Switch/Button-------
        # Switch/Button - see switch.
        # To test if it is pressed then
    #    if button.pressed():
    #        # Triggers gun test                    //Want to run motors at 7.2V, so do PWM:
    #        act_setSpeed(&LeftGun, -70)     #NOTE: (7.2V / 12.6V) * 127 = 72.5714286
    #        act_setSpeed(&RightGun, -70)     #NOTE: (7.2V / 12.6)V * 127 = 72.5714286
    #
    #        # pressed
    #        # We use the light variable to toggle stuff.
    #        if (light == True):
    #            LED_on(&statusLED)
    #            light = False
    #            #print("on!")
    #
    #        else:
    #            LED_off(&statusLED)
    #            light = True
    #            #print("off")
        
    #    #Check whether to stop firing guns
    #    if guns_firing and clockHasElapsed(guns_firing_start_time, guns_firing_duration):
    #        # guns_firing_duration = 0
    #        guns_firing = False
    #        act_setSpeed(&LeftGun, 0)     #NOTE: (7.2 / 12.6) * 127 = 72.5714286
    #        act_setSpeed(&RightGun, 0)     #NOTE: (7.2 / 12.6) * 127 = 72.5714286
    #        guns_firing_start_time = clockGetus()
    #    
    #    # To test if it is released then
    #    if SWITCH_released(&button):
    #        # released
    #        act_setSpeed(&LeftGun, 0)
    #        act_setSpeed(&RightGun, 0)
            
        # -------- End   Switch/Button-------
    
        # -------- Start Dynamixel AX-12 Driver-------
        # Dump the current values for all servos on AX12_driver to print
    #    ax12DumpAll(&AX12_driver)
        # -------- End   Dynamixel AX-12 Driver-------
    
        #/////////////////////////////////////////
        
        self.CmdrReadMsgs()
    
        # Get current time in ms
#        ms = (loopStart) / 1000 + spdChngOffset
    
        # We always move the turret to the position specified by the Commander.
        self.axbus.sync_write(self.turret_ids, ax.GOAL_POSITION, [struct.pack('<H', self.pan_pos),
                                                                  struct.pack('<H', self.tilt_pos)])
    
        # dump temperatures
        #if infobutton:
        #    ax12TempAll(UART3toAX12_driver)
        #    infobutton = False
        
        #if panicbutton:
        #    flopCnt += 1
        #    if flopCnt >= 3:
        #        g8Crouch()
        #        panic = True
        #        print("Howdydoo? %d",flopCnt)
        #        flopCnt = 0
        #    else:
        #        # Exit crouch/panic, enable standing, and re-enable torque to 2nd servo of each leg.
        #        panic = False
        #        standing = 0
        #        
        #        ax12SetTORQUE_ENABLE(&servo12, 1)
        #        ax12SetTORQUE_ENABLE(&servo22, 1)
        #        ax12SetTORQUE_ENABLE(&servo32, 1)
        #        ax12SetTORQUE_ENABLE(&servo42, 1)
        #        
        #    panicbutton = False
    
        #    # Limit to one press toggling at a time.
        #    utime.sleep_ms(100)
        #    uartFlushReceiveBuffer(LISTEN)
    
        #FIRE THE GUNS!!!!!
        #TODO
#        if gunbutton:
#            guns_firing = True
#            act_setSpeed(LeftGun, -65)     #NOTE: (7.2 / 12.6) * 127 = 72.5714286
#            guns_firing_start_time = utime.ticks_us() #clockGetus()
    
#        #We put "panic" before anything else that might move the legs.
#        if panic:
#            return 25000 #micro seconds
#        
#        # Decrement turn_loops continuously
#        if turn_loops > 0:
#            turn_loops -= 1
#        
#        if turnleft or turnright:
#            # LED_off(&statusLED)
#            if PRINT_DEBUG: print("Turn!  %u\t%u",turnright, turnleft)
#            
#            if turn_loops < 1)
#                self.loopLength = self.loopLengthList[THE_TURN_SPEED - 1]
#                # Two parts:
#                # 1) how far 'ms' is from the beginning of a turn animation
#                # 2) how far from the beginning of a turn animation we want to start at
#                self.turnTimeOffset = (ms % self.loopLength) - (0.2 * self.loopLength)
#            
#            turn = True
#            turn_loops = 20
#            turn_dir = 1
#            if turnleft: turn_dir = -1 # Reverse turn dir here
#            
#            standing = 0
#        
#        elif turn_loops > 0:
#            turn_loops = turn_loops # ?
#        
#        #Else, walking, possibly
#        else:
#            
#            self.turnTimeOffset = 0
#            
#            # walkNewDirIK(0)
#            walkSPD = sqrt(self.crx.walkv * self.crx.walkv + self.crx.walkh * self.crx.walkh)
#            walkDIR = atan2(self.crx.walkh, self.crx.walkv)
#            
#            walkSPD = interpolate(walkSPD, 0,102, 0,6)
#            
#            if walkSPD == 0 and turn_loops == 0:
#                #g8Stand()
#                walk = False
#                if standing < 6:
#                    standing += 1
#            
#            elif walkSPD > 0:
#                # Debug info
#                if PRINT_DEBUG:
#                    print("walk! %f ", (walkDIR * 180.0 / M_PI))
#            
#                #Disable turning when walking joystick is moved.
#                turn_loops = 0  #REDUNDANT
#                #Disable standing g8
#                standing = 0
#            
#                if walkSPD > MAX_WALK_SPD:
#                    walkSPD = MAX_WALK_SPD
#                elif walkSPD == 1 or walkSPD == 2:
#                    walkSPD = 3
#                
#                if USE_ONE_SPEED:
#                    walkSPD = THE_ONE_SPEED
#                
#                newLoopLength = self.loopLengthList[walkSPD - 1] # Temp storage
#                if newLoopLength != self.loopLength:                    # So we can check for change
#                    spdChngOffset += speedPhaseFix(loopStart, self.loopLength, newLoopLength)
#                    self.loopLength = newLoopLength
#                    #spdChngOffset = spdChngOffset%loopLength
#                walk = True
#                walkNewDirIK(int(walkDIR * 180.0 / M_PI))
#            do_gait = False
        #////////////////////////////////////////
        
        self.half_loopLength = self.loopLength / 2
        #travRate = 30 - 10 // this was redundant
        self.double_travRate = 2 * self.travRate
    
#        #//////////////////////////
#        # -------- Start Leg stuff-------
#    
#        #the 'now' variables are essentially a sawtooth waves.
#        now2 = (ms - self.turnTimeOffset) % self.loopLength
#    
#        now3 = (ms - self.turnTimeOffset +  half_loopLength) % self.loopLength
#    
#        now4 = self.loopLength - (ms - self.turnTimeOffset) % self.loopLength
#    
#        now1 = self.loopLength - (ms - self.turnTimeOffset + half_loopLength) % self.loopLength
#    
#        #/ Above is where the commander input is interpretted.
#        #/
#        #/ The next few blocks are where we determine what gait to use.
#        #/
#        #/
#        
#        # WALKING WITH IK via walkCode()
#        #if 0:  //Disables walking
#        if walk == True and turn_loops == 0:
#            walkCode(self.loopLength, half_loopLength, self.travRate, double_travRate)
#        # //Do this in the middle of the calculations to give guns a better firing time accuracy
#            # if guns_firing and clockHasElapsed(guns_firing_start_time, guns_firing_duration):
#                # guns_firing_duration = 0
#                # guns_firing = False
#                # act_setSpeed(&LeftGun,0)     //NOTE: (7.2 / 12.6) * 127 = 72.5714286
#                # act_setSpeed(&RightGun,0)     //NOTE: (7.2 / 12.6) * 127 = 72.5714286
#                # guns_firing_start_time = utime.ticks_us() #clockGetus()
#    
#        #Turnning with IK
#        elif turn_loops > 0 and walk == False:
#            
#            turnCode(turn_dir, self.loopLength, half_loopLength)
#                    
#            #print("%d\t%d\t%d\t%d",s12pos,s42pos, footH13,footH24)
#        
#        elif standing > 0 and standing <= 5:
#            # g8Stand()
#            g8FeetDown()
#        
#        elif turn_loops > 0 and walk == True:
#            # g8Stand()
#            g8FeetDown()
#        
#        turnright = False
#        turnleft = False
#        
#        
#        # Move all servos
#        if walk == True or turn_loops > 0:
#            self.axbus.sync_write(self.leg_ids, ax.GOAL_POSITION,
#                    [struct.pack('<H', pos) for pos in
#                               (s11pos, s21pos, s31pos, s41pos,
#                               s12pos, s13pos, s14pos, s22pos,
#                               s23pos, s24pos, s32pos, s33pos,
#                               s34pos, s42pos, s43pos, s44pos)])
#                    )

        # TODO
        #if PRINT_IR_RANGE:
        #    IRcnt += 1
        #    
        #    if IRcnt >= 8:
        #        
        #        distanceRead(distance)
        #        print("L")
        #        distanceDump(distance)
        #        print("\t")
        #        
        #        distanceRead(distance2)
        #        print("R")
        #        distanceDump(distance2)
        #        printCRLF()
        #        
        #    IRcnt = 0
        
#        if PRINT_DEBUG and walk == True:
#            print("")
        # elif PRINT_DEBUG_IK == True and turn == True: print("")
        #elif PRINT_DEBUG_IK == True and turn_loops > 0:
        #    print("")
        
        return PROG_LOOP_TIME #45000  //micro seconds
        #return 0
    #////////////////
    #/ End of Main Loop
    #////////////////
    
    
    #TICK_COUNT speedPhaseFix(TICK_COUNT clocktime, TICK_COUNT loopLenOld, TICK_COUNT loopLen)
    def speedPhaseFix(self, clocktime, loopLenOld, loopLen):
        #print(clocktime, loopLenOld, loopLen)
        return (((clocktime/1000) % loopLenOld) / loopLenOld -
            ((clocktime/1000) % loopLen) / loopLen * loopLen)
    

    #TODO
    def CmdrReadMsgs(self):
        while True:
            byte = self.cmdrbus.read_byte()
            if byte is None:
                break
            #print('Byte =', byte)
            # process_byte will update crx with latest values from a complete packet; no need to do anything with it here
            self.crx.process_byte(byte)
            #if self.crx.process_byte(byte) == CommanderRx.SUCCESS:
                #print('Walk: {:4d}h {:4d}v Look: {:4d}h {:4d}v {:08b}'.format(crx.walkh, crx.walkv, crx.lookh, crx.lookv, crx.button))

        # Update variables:
        out = ""
        buttonval = self.crx.button
        if buttonval & BUT_L6:
            self.gunbutton = True
            if PRINT_DEBUG_COMMANDER: out += "guns\t"
        else: self.gunbutton = False

        if buttonval & BUT_R3:
            self.panicbutton = True
            if PRINT_DEBUG_COMMANDER: out += "panic\t"
        else: self.panicbutton = False
        
        if buttonval & BUT_L4:
            self.infobutton = True
            if PRINT_DEBUG_COMMANDER: out += "info\t"
        else: self.infobutton = False
        
        if buttonval & BUT_R2:
            self.pan_pos = PAN_CENTER
            self.tilt_pos = TILT_CENTER
            if PRINT_DEBUG_COMMANDER: out += "look\t"
        # else: infobutton = False
        
        # Look right
        if buttonval & BUT_R1:
            #self.agitbutton = True
            if PRINT_DEBUG_COMMANDER: out += "lookR\t"
            self.pan_pos = PAN_MAX
        
        # Look left
        if buttonval & BUT_R3:
            #self.agitbutton = True
            if PRINT_DEBUG_COMMANDER: out += "lookL\t"
            self.pan_pos = PAN_MIN

        #if laserbutton and cmdrAlive > 0:
        #    #TODO
        #    setDigitalOutput(param_laser_pin, HIGH)
        #else: setDigitalOutput(param_laser_pin, LOW)}

        if out:
            print(out)

        pan_add = int(-self.crx.lookh / 17)
        tilt_add = int(-self.crx.lookv / 25)
         
        self.pan_pos = clamp(self.pan_pos + pan_add, self.servo51Min, self.servo51Max)
        self.tilt_pos = clamp(self.tilt_pos + tilt_add, self.servo52Min, self.servo52Max)
#            
#            //Default handling in original Commander.c - sets to range of -127 to 127 or so...
#            walkV = self.crx.walkv - 128
#            walkH = self.crx.walkv - 128

        return


def main():
    x = NumaMain()
    x.app_init_software()
    input("Waiting... (press enter)")
    input("Now onwards! (press enter again)")
    x.main()

if __name__ == "__main__":
    main()
