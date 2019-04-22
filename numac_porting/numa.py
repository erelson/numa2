from math import pi, sqrt, atan2
import struct
#import os
import sys

# system-type dependent imports
#sysname = os.uname().sysname
sysname = sys.platform
if sysname == 'linux' or sysname == 'win32':
    # Mocks for non-pyboard use
    import time
    def ticks_us(): return (time.time()%1.e4)*1.e6
    def ticks_diff(x, y): return x - y
    def sleep_us(x): return time.sleep(x/1.e6)
    def sleep_ms(x): return time.sleep(x/1.e3)
    #Bus = lambda x: x
    from unittest.mock import Mock, MagicMock
    Pin = Mock()
    #_read = MagicMock()
    #_read.__gt__.return_value = 0
    #_read().__gt__.return_value = 0
    #MotorDriver = MagicMock()#cs_adc=MagicMock(read=1))
    #MotorDriver.__gt__.return_value = 5
    #x = MotorDriver()
    #print(x)
    #sys.exit()
    from mock_hardware import MockMotorDriver as MotorDriver
    from mock_hardware import MockUART_Port_from_COM as UART_Port
    from mock_hardware import MockBusToQueue as Bus
    print("UART_PORT is:", UART_Port)

elif sysname == 'pyboard':
    from stm_uart_port import UART_Port
    from bus import Bus
    from utime import ticks_us, ticks_diff, sleep_us, sleep_ms
    from pyb import Pin
    from MotorDriver import MotorDriver

#
import ax
from helpers import clamp, speedPhaseFix
from init import myServoReturnLevels, myServoSpeeds, initServoLims
from commander import CommanderRx
from poses import gen_numa2_legs, g8Stand, g8FeetDown, g8Flop, g8Crouch
from IK import Gaits


PROG_LOOP_TIME = 19500 # in microseconds

USE_ONE_SPEED = 0
THE_ONE_SPEED = 3
THE_TURN_SPEED = 7
MAX_WALK_SPD = 5

# bitmasks for buttons array
BUT_R1 = 0x01 # center pan
BUT_R2 = 0x02 # center pan and tilt
BUT_R3 = 0x04 # panic
BUT_L4 = 0x08 # second switch! fast pan mode
BUT_L5 = 0x10 # laser switch
BUT_L6 = 0x20 # fire gun
BUT_RT = 0x40 # turn right
BUT_LT = 0x80 # turn left

PRINT_DEBUG = False
PRINT_DEBUG_COMMANDER = False
PRINT_DEBUG_LOOP = False

# +153 is 45 deg offset for pan servo's mounting scheme.
PAN_CENTER = 511 + 153
TILT_CENTER = 511 + 95

LOADER_TIMEOUT_DURATION = 3000000 # microseconds
LOADER_SPEED_ON = -24
LOADER_SPEED_OFF = 0
ADC_LOADER_LIMIT = 100

GUN_SPEED_ON = -65 #NOTE: (7.2 / 12.6) * 127 = 72.5714286
GUN_SPEED_OFF = 0
GUNS_FIRING_DURATION = 250000 # us; 1/4 s

PARAM_LASER_PIN = Pin.board.X12
CMDR_ALIVE_CNT = 100

LOOPS_B4_FLOP = 3 # See flopCnt

# Command settings/interpretation variables (ints)
#currentAnim = 0

# IRcnt = 1


class NumaMain(object):
    """Class containing the main loop logic for the robot

    Invoke main() after initialization.
    """

    def __init__(self,
                 gaits,
                 cmdrbus=None,
                 axbus=None#, show=Bus.SHOW_PACKETS) # can print the packets...
            ):
        print("Initializing NumaMain!...")
        if cmdrbus:
            self.cmdrbus = cmdrbus
        else:
            self.cmdrbus = UART_Port(1, 38400)
        if axbus:
            self.axbus = axbus
            print("AXBUS QUEUE:", axbus.queue) # TODO debug remove
        else:
            self.axbus = Bus(UART_Port(2, 1000000))

        self.crx = CommanderRx()
        self.cmdrAlive = 0

        self.gaits = gaits

        self.leg_ids = [11, 21, 31, 41,
                        12, 22, 32, 42,
                        13, 23, 33, 43,
                        14, 24, 34, 44]
        self.turret_ids = [51, 52] # pan, tilt
        self.all_ids = self.leg_ids + self.turret_ids

        self.servo51Min, self.servo51Max = PAN_CENTER - 4 * (52+30),  PAN_CENTER + 4 * (52+30)
        self.servo52Min, self.servo52Max = 511 - 4 * 31,              511 + 4 * 65

        self.flopCnt = 0 # Counter incremented by button presses, eventually disabling leg servos

        self.loader_timeout_mode = 1 # 1 off, 0 on; see bb_loader()
        self.loader_timeout_end = LOADER_TIMEOUT_DURATION + ticks_us() # Adding the duration delay wasn't validated...

        self.loopLengthList = [6500, 2900, 2300, 1800, 1600, 1450,
                               1000] # This last value is for turn speed?

        # directionPinNameA, directionPinNameB, pwmNumber, encoderNumber=None
        # NOTE: VNH5019 current sense is 0.14 V/A. Cont/peak C is 12/30A.
        # NOTE: pyboard ADC is 12 bit with 3V limit.
        self.gunMotor = MotorDriver(Pin.board.X5, Pin.board.X8, Pin.board.X6, cs=Pin.board.X7)
        self.ammoMotor = MotorDriver(Pin.board.Y9, Pin.board.Y12, Pin.board.Y10, cs=Pin.board.Y11)
        self.laserGPIO = Pin(PARAM_LASER_PIN)

        # Binary vars
        self.walk = False
        self.turn = False
        self.light = True
        self.kneeling = False
        self.panic = False # If True, will not move leg servos
        self.guns_firing = False

        self.gunbutton = False
        self.panicbutton = False
        self.fastturret = False
        self.pan_pos = PAN_CENTER
        self.tilt_pos = TILT_CENTER

        # Defaults?
        trav_rate_default = 25 # TODO distance covered by steps?
        self.travRate = trav_rate_default
        self.double_travRate = 2 * self.travRate

        # Various?
        self.turnTimeOffset = 0
        self.ang_dir = 0 # degrees, 0 is forward, range = -180 to 180
        self.turn_dir = 1
        self.turnright = False
        self.turnleft = False
        self.turn = False

    def main(self, leg_geom):
        """Main loop is `self.app_control()`"""
        self.app_init_hardware()
        self.app_init_software(leg_geom)
        oldLoopStart = 0
        print("Starting NUMA loop...")
        while True:
            #TODO timing? ???
            loopStart = ticks_us()
            #print("------------------------LOOPSTART:", loopStart)
            desired_loop_time = self.app_control(loopStart)
            loopEnd = ticks_us()
            #print("------------------------LOOPEND:", loopEnd)
            if PRINT_DEBUG_LOOP:
                print("%ld" % (loopStart - oldLoopStart))
                oldLoopStart = self.loopStart
            makeup_time = desired_loop_time - ticks_diff(loopEnd, loopStart)
            if makeup_time > 0:
                sleep_us(makeup_time)
            else:
                print("Slow loop, exceeded looptime by:", -1 * makeup_time)
            #sleep_us(PROG_LOOP_TIME - ticks_diff(loopEnd, loopStart))
            if sysname == 'linux' or sysname == 'win32':
                pass
                #print("Simulation loopstart time:", loopStart, "us")


    # Initialise the hardware
    def app_init_hardware(self):
        #initHardware()
        pass

    # Initialise the software
    # returns TICK_COUNT, usec
    # TODO this silly loop thing needs to be rewritten
    def app_init_software(self, leg_geom):
        print("It begins....")

        # This can be used to set a replacement servo's ID in a pinch
        #ax12SetID(&servo1, 1)

        # Call gait for Standing
        g8Stand(self.gaits, self.axbus, self.leg_ids)

        myServoReturnLevels(self.axbus, all_ids=self.all_ids)
        print("ServoReturnLevelsSet!")
        initServoLims(self.axbus, self.all_ids, self.gaits)
        print("ServoLimsSet!")
        myServoSpeeds(self.axbus, self.leg_ids, self.turret_ids)
        print("ServoSpeedsSet!")

        # Setting mathy initial values for walking
        self.loopLength = 1800 # ms
        #self.half_loopLength = self.loopLength / 2 #redundant
        self.spdChngOffset = 0 # ms
        self.turn_loops = 0 # TODO explain this variable

        self.standing = 1 # 0: not standing; 1-5 will lower feet; 6+: feet are down, and robot is standing
        g8Stand(self.gaits, self.axbus, self.leg_ids)

        return 0

    # This is the main loop
    #TICK_COUNT appControl(LOOP_COUNT loopCount, TICK_COUNT loopStart):
    def app_control(self, loopStart):
        """

        loopStart: float
            Current clock time in microseconds
        """

        # Stop IK and g8s from coinciding... make Numa stop in place.
        #if self.walk == True:
        #    g8Stand(self.axbus, self.leg_ids)

        # -------- Start Switch/Button-------
        # Switch/Button - see switch.
        # To test if it is pressed then
    #    if button.pressed():
    #        # Triggers gun test           #Want to run motors at 7.2V, so do PWM:
    #        act_setSpeed(&LeftGun, -70)   #NOTE: (7.2V / 12.6V) * 127 = 72.5714286
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

        # TODO: Consider whether I can justify disabling guns when self.cmdrAlive=0
        # Check whether to stop firing guns
        if self.guns_firing and loopStart > self.guns_firing_end_time:
            self.guns_firing = False
            self.gunMotor.direct_set_speed(GUN_SPEED_OFF)
            self.guns_firing_end_time = loopStart

    #    # To test if it is released then
    #    if SWITCH_released(&button):
    #        # released
    #        act_setSpeed(&LeftGun, 0)

        # -------- End   Switch/Button-------

        # -------- Start Dynamixel AX-12 Driver-------
        # Dump the current values for all servos on AX12_driver to print
    #    ax12DumpAll(&AX12_driver)
        # -------- End   Dynamixel AX-12 Driver-------

        #/////////////////////////////////////////

        self.bb_loader(loopStart)

        # Get current commander command
        self.CmdrReadMsgs()
        self.cmdrAlive -= 1
        self.cmdrAlive = clamp(self.cmdrAlive, 0, CMDR_ALIVE_CNT)

        # Get current time in ms
        ms = (loopStart) / 1000 + self.spdChngOffset

        # We always move the turret to the position specified by the Commander.
        self.axbus.sync_write(self.turret_ids, ax.GOAL_POSITION, [struct.pack('<H', self.pan_pos),
                                                                  struct.pack('<H', self.tilt_pos)])

        # dump temperatures
        #if fastturret:
        #    ax12TempAll(UART3toAX12_driver)
        #    fastturret = False

        # TODO Test this!
        # Guessing: We go into the g8Crouch pose, then we reenable the torque to the 2nd servo in each leg afterwards
        if self.panicbutton:
            self.flopCnt += 1
            if self.flopCnt >= LOOPS_B4_FLOP:
                g8Crouch(self.gaits, self.axbus, self.leg_ids) # This disables torque to second servo in each leg
                self.panic = True
                print("Howdydoo? %d", self.flopCnt)
                self.flopCnt = 0
            else:
                # Exit crouch/panic, enable standing, and re-enable torque to 2nd servo of each leg.
                self.panic = False
                self.standing = 0

                # Enable torque second servo of each leg
                self.axbus.sync_write(self.leg_ids[4:8], ax.TORQUE_ENABLE, [bytearray([1]) for _ in range(4)])

            # Limit to one increment of flopCnt per button press
            sleep_ms(100)

        #FIRE THE GUNS!!!!!
        #TODO
        if self.gunbutton:
            print("bang!!!")
            self.guns_firing = True
            self.gunMotor.direct_set_speed(GUN_SPEED_ON)
            self.guns_firing_end_time = ticks_us() + GUNS_FIRING_DURATION
            self.loader_timeout_end = loopStart + LOADER_TIMEOUT_DURATION

        # We check "panic" before anything else that might move the legs
        if self.panic:
            return 25000  # microseconds

        # Decrement turn_loops continuously
        if self.turn_loops > 0:
            self.turn_loops -= 1

        if self.turnleft or self.turnright:
            #if PRINT_DEBUG: print("Turn!  %u\t%u", self.turnright, self.turnleft)

            if self.turn_loops < 1:
                self.loopLength = self.loopLengthList[THE_TURN_SPEED - 1]
                self.half_loopLength = self.loopLength / 2
                # Two parts:
                # 1) how far 'ms' is from the beginning of a turn animation
                # 2) how far from the beginning of a turn animation we want to start at
                self.turnTimeOffset = (ms % self.loopLength) - (0.2 * self.loopLength)

            self.turn = True
            self.turn_loops = 20
            self.turn_dir = 1
            if self.turnright:
                self.turn_dir = -1 # Reverse turn dir here

            self.standing = 0
            self.turnright = False
            self.turnleft = False

        elif self.turn_loops > 0:
            pass

        # Else, walking, possibly
        else:
            self.turnTimeOffset = 0

            # self.set_new_heading(0) # ??? # Walking forward = 0
            walkDIR = atan2(self.crx.walkh, self.crx.walkv) # -pi to pi

            # walkSPD is an integer value; 0 or positive
            walkSPD = sqrt(self.crx.walkv * self.crx.walkv + self.crx.walkh * self.crx.walkh)
            walkSPD = int(0 + (6 - 0) * (walkSPD - 0) / (102 - 0)) # interpolate(walkSPD, 0,102, 0,6)
            print("WalkDIR:", walkDIR, "WalkSPD:", walkSPD)

            # Not walking, and not turning, so stand!
            if walkSPD == 0 and self.turn_loops == 0:
                g8Stand(self.gaits, self.axbus, self.leg_ids) # this is the end result already.
                self.walk = False
                if self.standing < 6:
                    self.standing += 1

            # Walking
            elif walkSPD > 0:
                if PRINT_DEBUG:
                    print("walk! %f ", (walkDIR * 180.0 / pi))

                # Disable turning when walking joystick is moved.
                self.turn_loops = 0  #REDUNDANT
                # Disable standing g8
                self.standing = 0

                if walkSPD > MAX_WALK_SPD:
                    walkSPD = MAX_WALK_SPD
                elif walkSPD == 1 or walkSPD == 2:
                    walkSPD = 3

                if USE_ONE_SPEED:
                    walkSPD = THE_ONE_SPEED

                newLoopLength = self.loopLengthList[walkSPD - 1]
                if newLoopLength != self.loopLength:  # So we can check for change
                    # Note speedPhaseFix both incr and decrs
                    self.spdChngOffset += speedPhaseFix(loopStart, self.loopLength, newLoopLength)
                    self.loopLength = newLoopLength
                    #self.spdChngOffset = spdChngOffset%loopLength
                self.walk = True
                self.set_new_heading(int(walkDIR * 180.0 / pi))
        #////////////////////////////////////////

        self.half_loopLength = self.loopLength / 2
        # These don't change....
        #self.travRate = 30 - 10 # this was redundant
        #self.double_travRate = 2 * self.travRate

        #//////////////////////////
        # -------- Start Leg stuff-------

        # the 'now' variables are sawtooth waves (or triangle waves???).
        now1, now2, now3, now4 = self.get_now(ms)

        # Above is where the commander input is interpretted.
        #
        # The next few blocks are where we determine what gait to use.

        # WALKING WITH IK via walk_code()
        #if 0:  #Disables walking
        if self.walk == True and self.turn_loops == 0:
            self.gaits.walk_code(self.loopLength, self.half_loopLength,
                                self.travRate, self.double_travRate,
                                now1, now2, now3, now4, self.ang_dir)
        #   #Do this in the middle of the calculations to give guns a better firing time accuracy
            #if self.guns_firing and loopStart > self.guns_firing_end_time:
            #    self.guns_firing = False
            #    self.gunMotor.direct_set_speed(GUN_SPEED_OFF)
            #    self.guns_firing_end_time = loopStart

        # Turning with IK
        elif self.turn_loops > 0 and self.walk == False:
            self.gaits.turn_code(self.turn_dir, self.loopLength, self.half_loopLength, now1, now2, now3, now4)
            #print("%d\t%d\t%d\t%d",s12pos,s42pos, footH13,footH24)

        elif self.standing > 0 and self.standing <= 5:
            # or g8Stand?
            g8FeetDown(self.gaits, self.axbus, self.leg_ids)

        elif self.turn_loops > 0 and self.walk == True:
            # or g8Stand?
            g8FeetDown(self.gaits, self.axbus, self.leg_ids)


        # Move all servos
        try:
            if self.walk == True or self.turn_loops > 0:
                self.axbus.sync_write(self.leg_ids, ax.GOAL_POSITION,
                        [struct.pack('<H', int(pos)) for pos in
                           (self.gaits.s11pos, self.gaits.s21pos, self.gaits.s31pos, self.gaits.s41pos,
                            self.gaits.s12pos, self.gaits.s22pos, self.gaits.s32pos, self.gaits.s42pos,
                            self.gaits.s13pos, self.gaits.s23pos, self.gaits.s33pos, self.gaits.s43pos,
                            self.gaits.s14pos, self.gaits.s24pos, self.gaits.s34pos, self.gaits.s44pos)])
        except Exception:
            for cnt, x in enumerate([self.gaits.s11pos, self.gaits.s21pos, self.gaits.s31pos, self.gaits.s41pos,
                            self.gaits.s12pos, self.gaits.s13pos, self.gaits.s14pos, self.gaits.s22pos,
                            self.gaits.s23pos, self.gaits.s24pos, self.gaits.s32pos, self.gaits.s33pos,
                            self.gaits.s34pos, self.gaits.s42pos, self.gaits.s43pos, self.gaits.s44pos]):
                print(cnt, x)


        # TODO
        #if PRINT_IR_RANGE:
        #    IRcnt += 1
        #    if IRcnt >= 8:
        #        distanceRead(distance)
        #        print("L")
        #        distanceDump(distance)
        #        print("\t")
        #        distanceRead(distance2)
        #        print("R")
        #        distanceDump(distance2)
        #        printCRLF()
        #    IRcnt = 0

#        if PRINT_DEBUG and walk == True:
#            print("")
        # elif PRINT_DEBUG_IK == True and turn == True: print("")
        #elif PRINT_DEBUG_IK == True and self.turn_loops > 0:
        #    print("")

        return PROG_LOOP_TIME #45000 # microseconds
    #////////////////
    #/ End of Main Loop
    #////////////////

    def get_now(self, ms):
        # the 'now' variables are sawtooth waves (or triangle waves???).
        now2 = (ms - self.turnTimeOffset) % self.loopLength
        now3 = (ms - self.turnTimeOffset +  self.half_loopLength) % self.loopLength
        now4 = self.loopLength - (ms - self.turnTimeOffset) % self.loopLength
        now1 = self.loopLength - (ms - self.turnTimeOffset + self.half_loopLength) % self.loopLength
        return now1, now2, now3, now4

    def set_new_heading(self, new_dir):
        # Calculate ang_dir (degrees); ranges from  ...
        # new_dir:
        if self.ang_dir == new_dir:
             return

        #if not previously walking with IK...
        #if not self.walk: #TODO Not needed?
        #    g8Stand(self.gaits, self.axbus, self.leg_ids)  # Note: walk is now FALSE; set walk after this.
        #    ang_dir = new_dir
        #    # NEED TO SET TIMING HERE
        # End former indent

        # If too big a change in direction, change to standing position, then start fresh
        #(abs(new_dir - self.ang_dir) % 360) #old
        if abs((new_dir - self.ang_dir + 180) % 360 - 180) >= 20:

            g8Stand(self.gaits, self.axbus, self.leg_ids) # Note: walk is now FALSE; g8Stand sets walk
            self.ang_dir = new_dir
        # else update direction
        else:
            self.ang_dir = new_dir
        return


    #TODO
    def CmdrReadMsgs(self):
        while True:
            byte = self.cmdrbus.read_byte()
            # TODO mocking uses serial.Serial.read() which blocks... won't ever be None currently
            if byte is None: # emptied buffer
                break
            # process_byte will update crx with latest values from a complete packet; no need to do anything with it here
            if self.crx.process_byte(byte) == CommanderRx.SUCCESS:
                self.cmdrAlive = CMDR_ALIVE_CNT # reset keepalive
                self.cmdrbus.clear_read_buffer()
                break
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
            self.fastturret = True
            if PRINT_DEBUG_COMMANDER: out += "fastpan\t"
        else: self.fastturret = False

        if buttonval & BUT_R2:
            self.pan_pos = PAN_CENTER
            self.tilt_pos = TILT_CENTER
            if PRINT_DEBUG_COMMANDER: out += "lookcenter\t"

        if buttonval & BUT_R1:
            self.pan_pos = PAN_CENTER
            if PRINT_DEBUG_COMMANDER: out += "lookfront\t"
        else:
            pass

        # laser on if button/switch pressed and commander communications are working
        if buttonval & BUT_L5 and self.cmdrAlive:
            self.laserGPIO.value(1)
        else:
            self.laserGPIO.value(0)

        dowalking = True
        if buttonval & BUT_LT:
            if PRINT_DEBUG_COMMANDER: out += "tlft\t"
            self.turnleft = True
            self.turnright = False
            dowalking = False
        elif buttonval & BUT_RT:
            if PRINT_DEBUG_COMMANDER: out += "trgt\t"
            self.turnright = True
            self.turnleft = False
            dowalking = False
        else: # Do nothing
            self.turnright = False
            self.turnleft = False
            self.turn = False

        if dowalking:
            # Walk joystick is left joystick
            # Default handling in original Commander.c - sets to range of -127 to 127 or so...
            # vals - 128 gives look a vlaue in the range from -128 to 127?
            # TODO unused wtf
            walkV = self.crx.walkv
            walkH = self.crx.walkh#v
            pass

        # Look joystick is right joystick
        if self.fastturret:
            pan_add = int(-self.crx.lookh / 10)
        else:
            pan_add = int(-self.crx.lookh / 17)
        tilt_add = int(-self.crx.lookv / 25)

        self.pan_pos = clamp(self.pan_pos + pan_add, self.servo51Min, self.servo51Max)
        self.tilt_pos = clamp(self.tilt_pos + tilt_add, self.servo52Min, self.servo52Max)

        if out:
            print("Output:", out)
        return


    def bb_loader(self, loopStart):
        if self.loader_timeout_mode:
            self.ammoMotor.direct_set_speed(LOADER_SPEED_OFF)
            # TODO bug: wraparound in calculation of loader_timeout_end; reference loopCount
            if loopStart > self.loader_timeout_end:
                self.loader_timeout_mode = 0
        else:
            #print(LOADER_SPEED_ON,  self.ammoMotor.cs_adc.read())
            self.ammoMotor.direct_set_speed(LOADER_SPEED_ON)

        # TODO port this; how many bits is the ADC? Check the voltage limits
        # Use self.ammoMotor.cs as pin
        #if a2dConvert10bit(ADC_CH_ADC10) > ADC_LOADER_LIMIT:
        adcval = self.ammoMotor.cs_adc.read()
        if adcval > ADC_LOADER_LIMIT:
            print("adcval exceeded 10:", adcval)
            self.ammoMotor.direct_set_speed(LOADER_SPEED_OFF)
            self.loader_timeout_mode = 1
            self.loader_timeout_end = loopStart + LOADER_TIMEOUT_DURATION

        # -------- Start Analogue Input-------
        # Dump out the raw value for Analogue Input
        # Dump out the mV (milli-volts) for Analogue Input
        #print("m1current: " << a2dConvert10bit(ADC_CH_ADC10) << "  m1current: " << a2dReadMv(ADC_CH_ADC10)"mV")


def main():
    leg_geom, leg1, leg2, leg3, leg4 = gen_numa2_legs()
    gaits = Gaits(leg_geom, leg1, leg2, leg3, leg4)
    x = NumaMain(gaits)
    # Safety...
    x.gunMotor.direct_set_speed(GUN_SPEED_OFF)
    x.ammoMotor.direct_set_speed(LOADER_SPEED_ON)
    sleep_ms(2000)
    #input("Waiting... (press enter)")
    #input("Now onwards! (press enter again)")
    try:
        x.main(leg_geom)
    except:
        print("Safely stopping gun!")
        x.gunMotor.direct_set_speed(GUN_SPEED_OFF)
        print("Safely stopping loader!")
        x.ammoMotor.direct_set_speed(LOADER_SPEED_OFF)
        raise

if __name__ == "__main__":
    main()
