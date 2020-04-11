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
    def ticks_us():
        return (time.time()%1.e4)*1.e6
    def ticks_diff(x, y):
        return x - y
    def sleep_us(x):
        return time.sleep(x/1.e6)
    def sleep_ms(x):
        return time.sleep(x/1.e3)
    #Bus = lambda x: x
    from unittest.mock import Mock, MagicMock
    Pin = Mock()
    ADC = Mock()
    UART = Mock()
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
    BusError = Exception
    print("UART_PORT is:", UART_Port)

elif sysname == 'pyboard':
    from stm_uart_port import UART_Port
    from pyb import Pin, UART, ADC
    from bus import Bus, BusError
    from utime import ticks_us, ticks_diff, sleep_us, sleep_ms
    from MotorDriver import MotorDriver

#
import ax
from helpers import clamp, speedPhaseFix
from init import myServoReturnLevels, myServoSpeeds, initServoLims
from commander import CommanderRx
from poses import gen_numa2_legs, g8Stand, g8FeetDown, g8Crouch
from IK import Gaits


#PROG_LOOP_TIME = 19500 # in microseconds
PROG_LOOP_TIME = 12000 # in microseconds

USE_ONE_SPEED = 0
THE_ONE_SPEED = 3
THE_TURN_SPEED = 7

# bitmasks for buttons array
BUT_R1 = 0x01 # center pan
BUT_R2 = 0x02 # center pan and tilt
BUT_R3 = 0x04 # crouch
BUT_L4 = 0x08 # second switch! slow pan mode + laser on
BUT_L5 = 0x10 # laser switch (secondary approach for now)
BUT_L6 = 0x20 # fire gun
BUT_RT = 0x40 # turn right
BUT_LT = 0x80 # turn left

PRINT_DEBUG = False
PRINT_DEBUG_COMMANDER = 0
PRINT_DEBUG_LOOP = False

# +153 is 45 deg offset for pan servo's mounting scheme.
PAN_CENTER = 511 + 153
TILT_CENTER = 511 + 45

LOADER_TIMEOUT_DURATION = 1000000 # microseconds
LOADER_SPEED_ON = -54  # counterclockwise
LOADER_SPEED_OFF = 0
ADC_LOADER_LIMIT = 100

GUN_SPEED_ON = 65 #NOTE: (7.2 / 12.6) * 127 = 72.5714286
GUN_SPEED_OFF = 0
#GUNS_FIRING_DURATION = 250000 # us; 1/4 s; burst of ~3 shots
GUNS_FIRING_DURATION = 80000 # us; 80 ms; better for marksmanship, and works fine in general

PARAM_LASER_PIN = Pin.board.X12
CMDR_ALIVE_CNT = 30 # ~360 ms

LOOPS_B4_CROUCH = 3 # See crouchCnt

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
            self.cmdrbus = UART(1, 38400) #UART_Port(1, 38400)
        if axbus:
            self.axbus = axbus
        else:
            self.axbus = Bus(UART_Port(2, 1000000))#, show=2)

        self.crx = CommanderRx()
        self.cmdrAlive = 0

        self.gaits = gaits
        self.g8countdown = 0  # When this is non-zero, main loop doesn't do any leg commands

        self.leg_ids = [11, 21, 31, 41,
                        12, 22, 32, 42,
                        13, 23, 33, 43,
                        14, 24, 34, 44]
        self.turret_ids = [51, 52] # pan, tilt
        self.all_ids = self.leg_ids + self.turret_ids

        self.servo51Min, self.servo51Max = PAN_CENTER - 4 * (52+30),  PAN_CENTER + 4 * (52+30)
        self.servo52Min, self.servo52Max = 511 - 4 * 31,              511 + 4 * 65
        self.servo52Min, self.servo52Max = 390, 700

        self.crouchCnt = 0 # Counter incremented by button presses, eventually disabling leg servos

        self.loader_timeout_mode = "timeout" # one of [running, timeout, rewind]
        self.loader_timeout_end = 3 * LOADER_TIMEOUT_DURATION + ticks_us() # Initial longer delay of 3 s
        self.loader_reverse_end = 0

        self.loopLengthList = [2900, 2000, 1400, 1050, 820, 650, # BAMF2019 preferred
        #self.loopLengthList = [2900, 2000, 1400, 1050, 800, 550, # Bit jerky
                               750] # This last value is for turn speed
        self.MAX_WALK_SPD = len(self.loopLengthList) - 1 #6

        # directionPinNameA, directionPinNameB, pwmNumber, encoderNumber=None
        # NOTE: VNH5019 current sense is 0.14 V/A. Cont/peak C is 12/30A.
        # NOTE: pyboard ADC is 12 bit with 3V limit.
        self.gunMotor = MotorDriver(Pin.board.X5, Pin.board.X8, Pin.board.X6, cs=Pin.board.X7)

        # NOTE this might not match my actual wiring due to compensating for weird behavior where it spins at startup
        self.ammoMotor = MotorDriver(Pin.board.Y9, Pin.board.Y12, Pin.board.Y10, cs=Pin.board.Y11)

        self.bb_detect_adc = ADC(Pin.board.X1)
        self.bb_detect_led = Pin(Pin.board.X2, mode=Pin.OUT)
        self.bb_detect_adc_loopcnt = 0
        self.MIN_ADC_NO_BB = 80
        self.laserGPIO = Pin(PARAM_LASER_PIN, mode=Pin.OUT)

        # Binary vars
        self.walk = False
        self.turn = False
        self.light = True
        self.crouch = False # If True, will not move leg servos
        self.guns_firing = False

        self.gunbutton = False
        self.crouchbutton = False
        self.crouchtimeout = 0
        self.slowturret = False
        self.pan_pos = PAN_CENTER
        self.tilt_pos = TILT_CENTER

        # Defaults (currently I never change these)
        trav_rate_default = 25 # distance covered by a stride is twice this
        self.travRate = trav_rate_default
        self.double_travRate = 2 * self.travRate

        # Various?
        self.turnTimeOffset = 0
        self.ang_dir = 0 # degrees, 0 is forward, range = -180 to 180
        self.turn_dir = 1
        self.turnright = False
        self.turnleft = False
        self.turn = False
        self.walkV = 0
        self.walkH = 0

        # Temperature reading loop parameters
        self.t_temp_read = 0 # ms
        self.n_temp_read = 4 # 4 servos
        self.temp_read_cnt = 0
        self.temps = [0,0,0,0]
        self.temp_read_wait = 1000 # ms

    def zero_all_actions(self):
        #self.g8countdown = 0 # TODO do we also want this when we crouch? do we ever want this?
        self.turn_loops = 0

    def main(self, leg_geom):
        """Main loop body is in `self.app_control()`
        """
        self.app_init_hardware()
        self.app_init_software(leg_geom)
        t0 = ticks_us()
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
            oldLoopStart = loopStart
            makeup_time = desired_loop_time - ticks_diff(loopEnd, loopStart)
            if makeup_time > 0:
                sleep_us(makeup_time)
            else:
                print("Slow loop, took:", (PROG_LOOP_TIME - makeup_time) / 1000, "ms (ideal loop time:", PROG_LOOP_TIME/1000, ")")
                #print("Slow loop, exceeded looptime by:", -1 * makeup_time / 1000, "ms (ideal loop time:", PROG_LOOP_TIME/1000, ")")
            #sleep_us(PROG_LOOP_TIME - ticks_diff(loopEnd, loopStart))
            if sysname == 'linux' or sysname == 'win32':
                #print("Simulation loopstart time:", loopStart, "us", "Uptime:", (loopStart - t0)/1e6)
                pass


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
        self.g8countdown = g8Stand(self.gaits, self.axbus, self.leg_ids)

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

        # TODO self.standing doesn't seem accurate anymore
        self.standing = 1 # 0: not standing; 1-5 will lower feet; 6+: feet are down, and robot is standing
        self.g8countdown = g8Stand(self.gaits, self.axbus, self.leg_ids)

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
        #    self.g8countdown = g8Stand(self.axbus, self.leg_ids)

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

        #/////////////////////////////////////////

        # Get current commander command
        self.CmdrReadMsgs()
        self.cmdrAlive -= 1
        self.cmdrAlive = clamp(self.cmdrAlive, 0, CMDR_ALIVE_CNT)

        #
        adcval_effort = self.ammoMotor.cs_adc.read()
        adcval_loaded = self.bb_detect_adc.read()
        self.bb_loader_service(loopStart, adcval_effort, adcval_loaded)

        # Get current time in ms
        ms = (loopStart) / 1000

        # We always move the turret to the position specified by the Commander.
        self.axbus.sync_write(self.turret_ids, ax.GOAL_POSITION, [struct.pack('<H', self.pan_pos),
                                                                  struct.pack('<H', self.tilt_pos)])

        # dump temperatures
        self.read_write_s2_temps(ms)

        # If a gait has recently set the g8countdown, we won't process any leg movement code
        if self.g8countdown:
            #
            pass

        # TODO Test this!
        # Guessing: We go into the g8Crouch pose, then we reenable the torque to the 2nd servo in each leg afterwards
        if self.crouchbutton:
            self.crouchCnt += 1

            # If we're ready to crouch and aren't already
            if self.crouchCnt >= LOOPS_B4_CROUCH and not self.crouch:
                self.g8countdown = g8Crouch(self.gaits, self.axbus, self.leg_ids) # This disables torque to second servo in each leg
                self.crouch = True
                print("Howdydoo?", self.crouchCnt)
                self.crouchCnt = 0
                # After doing safety-critical things, we'll end the loop early
            # Already crouched
            elif self.crouchCnt >= LOOPS_B4_CROUCH:
                print("Hurray, not spasming now?", self.crouchCnt)
                pass

            #self.crouchbutton = False
            # Limit to one increment of crouchCnt per button press
            #sleep_ms(10)


        # If crouch no longer pressed and were in crouch mode, Exit crouch/panic, enable standing, and re-enable torque to 2nd servo of each leg.
        elif self.crouch:
            self.crouch = False
            self.standing = 0
            # Will subsequently automatically stand

            # Enable torque second servo of each leg
            self.axbus.sync_write(self.leg_ids[4:8], ax.TORQUE_ENABLE, [bytearray([1]) for _ in range(4)])

        #elif self.crouchCnt:

        #FIRE THE GUNS!!!!!
        #TODO
        # NOTE could also make this conditional on loader_timeout_mode == "running"
        # But (hypothesis) I'm not going to be continually firing for more than a second at a time, so I should
        # have sufficient BBs to feed in via gravity while unjamming occurs.
        if self.gunbutton:
            print("bang!!!")
            self.guns_firing = True
            self.gunMotor.direct_set_speed(GUN_SPEED_ON)
            self.guns_firing_end_time = ticks_us() + GUNS_FIRING_DURATION
            self.loader_timeout_end = loopStart + LOADER_TIMEOUT_DURATION # microseconds

        # We check "panic" before anything else that might move the legs
        if self.crouch:
            self.zero_all_actions()
            return PROG_LOOP_TIME # microseconds

        # Decrement turn_loops continuously
        if self.turn_loops > 0:
            self.turn_loops -= 1

        # If commander says to turn
        if self.turnleft or self.turnright:
            #if PRINT_DEBUG: print("Turn!  %u\t%u", self.turnright, self.turnleft)

            # If not currently turning, setup turning timing
            if self.turn_loops <= 0:
                self.loopLength = self.loopLengthList[THE_TURN_SPEED - 1]
                self.half_loopLength = self.loopLength / 2
                self.spdChngOffset = 0
                # Two parts: TODO 4-28 re-evaluate this
                # 1) how far 'ms' is from the beginning of a turn animation
                # 2) how far from the beginning of a turn animation we want to start at
                fractional_offset = 0.5 # used 0.2 for Numa V1
                self.turnTimeOffset = (ms % self.loopLength) - (fractional_offset * self.loopLength)
                print(self.get_now(ms))

            # Make sure we continue turning for at least... 30 loops
            self.turn = True # TODO redundant
            self.turn_loops = 30
            self.turn_dir = 1
            if self.turnright:
                self.turn_dir = -1 # Reverse turn dir here

            self.standing = 0
            self.turnright = False
            self.turnleft = False
        # Continue turning if we're already turning, until we exceed turn_loops
        elif self.turn_loops > 0:
            pass
        # Else, walking, possibly
        else:
            self.turnTimeOffset = 0

            # New walk direction; Walking forward = 0; TODO am I actually generating clockwise angles instead of cc?
            walkDIR = atan2(self.walkH, self.walkV) # -pi to pi

            # walkSPD is an integer value; 0 or positive
            walkSPD = sqrt(self.walkV * self.walkV + self.walkH * self.walkH)
            walkSPD = int(0 + (6 - 0) * (walkSPD - 0) / (102 - 0)) # interpolate(walkSPD, 0,102, 0,6)
            #print("WalkDIR:", walkDIR, "WalkSPD:", walkSPD)

            # Not walking, and not turning, so stand! We send this pose 5 times to ensure we reach the position.
            if walkSPD == 0 and self.turn_loops == 0:
                self.g8countdown = g8Stand(self.gaits, self.axbus, self.leg_ids)
                self.walk = False
                if self.standing < 6:
                    self.standing += 1
            # else: already standing, don't re-send the pose

            # Walking
            elif walkSPD > 0:
                if PRINT_DEBUG:
                    print("walk! %f ", (walkDIR * 180.0 / pi))

                # Disable turning when walking joystick is moved.
                self.turn_loops = 0  #REDUNDANT
                # Disable standing g8
                self.standing = 0

                # With this logic, we're effectively using 3 speeds
                if walkSPD > self.MAX_WALK_SPD:
                    walkSPD = self.MAX_WALK_SPD
                elif walkSPD == 1:# or walkSPD == 2:
                    walkSPD = 2
                    #walkSPD = 3

                if USE_ONE_SPEED:
                    walkSPD = THE_ONE_SPEED

                newLoopLength = self.loopLengthList[walkSPD - 1]
                if newLoopLength != self.loopLength:  # So we can check for change
                    print("Changed from", self.loopLength, "to", newLoopLength)
                    # Note speedPhaseFix both incrs and decrs our time tracking variable, `ms`
                    self.spdChngOffset = speedPhaseFix(self.spdChngOffset, ms, self.loopLength, newLoopLength)
                    self.loopLength = newLoopLength
                self.walk = True
                self.set_new_heading(int(walkDIR * 180.0 / pi))
        #////////////////////////////////////////

        self.half_loopLength = self.loopLength / 2
        # These don't change currently...
        #self.travRate = 30 - 10 # this was redundant
        #self.double_travRate = 2 * self.travRate

        #//////////////////////////
        # -------- Start Leg stuff-------

        # the 'now' variables are sawtooth waves (or triangle waves???).
        now1, now2, now3, now4 = self.get_now(ms + self.spdChngOffset)

        # Above is where the commander input is interpretted.
        #
        # The next few blocks are where we determine what gait to use.

        # WALKING WITH IK via walk_code()
        #if 0:  #Disables walking
        if self.walk == True and self.turn_loops == 0:
            self.gaits.walk_code(self.loopLength, self.half_loopLength,
                                self.travRate, self.double_travRate,
                                now1, now2, now3, now4, self.ang_dir,
                                self.curve_dir)
        #   #Do this in the middle of the calculations to give guns a better firing time accuracy
            #if self.guns_firing and loopStart > self.guns_firing_end_time:
            #    self.guns_firing = False
            #    self.gunMotor.direct_set_speed(GUN_SPEED_OFF)
            #    self.guns_firing_end_time = loopStart

        # Turning with IK
        elif self.turn_loops > 0 and self.walk == False:
            self.gaits.turn_code(self.turn_dir, self.loopLength, self.half_loopLength, now1, now2, now3, now4)

        elif self.standing > 0 and self.standing <= 5:
            # or g8Stand?
            self.g8countdown = g8FeetDown(self.gaits, self.axbus, self.leg_ids)

        elif self.turn_loops > 0 and self.walk == True:
            # or g8Stand?
            self.g8countdown = g8FeetDown(self.gaits, self.axbus, self.leg_ids)


        # Move all servos
        try:
            if self.walk == True or self.turn_loops > 0:
                self.axbus.sync_write(self.leg_ids, ax.GOAL_POSITION,
                        [struct.pack('<H', int(pos)) for pos in
                           (self.gaits.s11pos, self.gaits.s21pos, self.gaits.s31pos, self.gaits.s41pos,
                            self.gaits.s12pos, self.gaits.s22pos, self.gaits.s32pos, self.gaits.s42pos,
                            self.gaits.s13pos, self.gaits.s23pos, self.gaits.s33pos, self.gaits.s43pos,
                            self.gaits.s14pos, self.gaits.s24pos, self.gaits.s34pos, self.gaits.s44pos)])
                #print("Coax positions:",self.gaits.s11pos, self.gaits.s21pos, self.gaits.s31pos, self.gaits.s41pos)
                #print("Femur positions:",self.gaits.s12pos, self.gaits.s22pos, self.gaits.s32pos, self.gaits.s42pos)
                #print("Tibia positions:",self.gaits.s13pos, self.gaits.s23pos, self.gaits.s33pos, self.gaits.s43pos)
        except Exception:
            self.print_gait_positions()


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
        ms = ms - self.turnTimeOffset  #
        # the 'now' variables are sawtooth waves (or triangle waves???).
        now2 = ms % self.loopLength
        now3 = (ms +  self.half_loopLength) % self.loopLength
        now4 = self.loopLength - ms % self.loopLength
        now1 = self.loopLength - (ms + self.half_loopLength) % self.loopLength
        return now1, now2, now3, now4

    def set_new_heading(self, new_dir):
        # Returns True if the change is a big change
        # Calculate ang_dir (degrees); ranges from  ...
        # new_dir:
        if self.ang_dir == new_dir:
             return False

        #if not previously walking with IK...
        #if not self.walk: #TODO Not needed?
        #    self.g8countdown = g8Stand(self.gaits, self.axbus, self.leg_ids)  # Note: walk is now FALSE; set walk after this.
        #    ang_dir = new_dir
        #    # NEED TO SET TIMING HERE
        # End former indent

        # If too big a change in direction, change to standing position, then start fresh
        #(abs(new_dir - self.ang_dir) % 360) #old
        #if self.ang_dir is None or abs((new_dir - self.ang_dir + 180) % 360 - 180) >= 20: #TODO enhancement; we don't need to be sending g8Stand if we go: forward. stop. backwards. (vs just forward, then instantly backwards)
        if abs((new_dir - self.ang_dir + 180) % 360 - 180) >= 20:

            self.g8countdown = g8Stand(self.gaits, self.axbus, self.leg_ids) # Note: walk is now FALSE; g8Stand sets walk
            self.ang_dir = new_dir
            return True
        # else update direction
        else:
            self.ang_dir = new_dir
            return False

    def read_write_s2_temps(self, t_ms):
        # Grab a single temperature periodically and send over Xbee when all temps have been updated
        if t_ms - self.t_temp_read < self.temp_read_wait:
            return
        # else...
        self.t_temp_read = t_ms

        servoID = 2 + 10 * (self.temp_read_cnt + 1)
        try:
            self.temps[self.temp_read_cnt] = self.axbus.read(servoID, ax.PRESENT_TEMP, 1)[0]
        except BusError:
            # Nothing received? Timeout probably.
            pass

        if self.temp_read_cnt == self.n_temp_read - 1:
            try:
                #self.cmdrbus.write_packet(self.temps)
                self.cmdrbus.write(str(self.temps) + "\r\n")
                print("wrote a set of bytes?", self.temps)
            except Exception as e:
                print(self.temps)
                print(e)
                raise
                # Nothing sent?
                pass
        self.temp_read_cnt = (self.temp_read_cnt + 1 ) % self.n_temp_read

    #TODO
    def CmdrReadMsgs(self):
        while self.cmdrbus.any():  # avoid waiting for timeout when nothing to read
            byte = self.cmdrbus.read(1)
            # TODO mocking uses serial.Serial.read() which blocks... won't ever be None currently
            if byte is None: # emptied buffer; conditional probably not needed
                break
            # process_byte will update crx with latest values from a complete packet; no need to do anything with it here
            if self.crx.process_byte(ord(byte)) == CommanderRx.SUCCESS:
                self.cmdrAlive = CMDR_ALIVE_CNT # reset keepalive
                self.cmdrbus.read(self.cmdrbus.any())  # empty buffer
                break

        # Update variables:
        out = ""
        buttonval = self.crx.button
        if buttonval & BUT_L6:
            self.gunbutton = True
            if PRINT_DEBUG_COMMANDER: out += "guns\t"
        else: self.gunbutton = False

        if buttonval & BUT_R3:
            #self.crouchbutton = True
            #if PRINT_DEBUG_COMMANDER: out += "crouch\t"
            self.turn_mode = True
        else:
            #if PRINT_DEBUG_COMMANDER: out += "!panic\t"
            #self.crouchbutton = False
            self.turn_mode = False

        if buttonval & BUT_L4:
            self.slowturret = True
            if PRINT_DEBUG_COMMANDER: out += "fastpan\t"
            if self.cmdrAlive:
                self.laserGPIO.value(1)
            else:
                self.laserGPIO.value(0)
        else:
            self.slowturret = False
            self.laserGPIO.value(0)

        if buttonval & BUT_R2:
            self.pan_pos = PAN_CENTER
            self.tilt_pos = TILT_CENTER
            if PRINT_DEBUG_COMMANDER: out += "lookcenter\t"

        if buttonval & BUT_R1:
            self.pan_pos = PAN_CENTER
            if PRINT_DEBUG_COMMANDER: out += "lookfront\t"
        else:
            pass

        # If button is pressed (or switch is "on") disable BB Loader
        if buttonval & BUT_L5:# and self.cmdrAlive:
            self.enable_bb_loader = False
        else:
            self.enable_bb_loader = True

        # Defaults that might change below
        dowalking = True
        self.curve_dir = 0
        self.walkV = 0
        self.walkH = 0

        if self.turn_mode:
            turnH = self.crx.walkh#v
            dowalking = False
            if turnH < -80: # LEFT buttonval & BUT_LT:
                self.turnleft = True
                self.turnright = False
            elif turnH > 80: # RIGHT buttonval & BUT_RT:
                self.turnright = True
                self.turnleft = False
            elif abs(turnH) > 10: # Curved walking
                # Walk curving from straight
                self.walkH = 0
                # Conditionals for four regions of joystick location
                if turnH > 5:
                    if crx.walkv > 0:
                        self.walkV = min(self.crx.walkv, 80)
                    else:
                        self.walkV = max(self.crx.walkv, -80)
                    self.curve_dir = 1
                elif turnH < -5:
                    if urx.walkv > 0:
                        self.walkV = min(self.crx.walkv, 80)
                    else:
                        self.walkV = max(self.crx.walkv, -80)
                    self.curve_dir = -1
            else: # Do nothing
                self.turnright = False
                self.turnleft = False
                self.turn = False

        # Old code for turning; holding trigger buttons was unreliable in noisey environ
        #if buttonval & BUT_LT:
        #    if PRINT_DEBUG_COMMANDER: out += "tlft\t"
        #    self.turnleft = True
        #    self.turnright = False
        #    dowalking = False
        #elif buttonval & BUT_RT:
        #    if PRINT_DEBUG_COMMANDER: out += "trgt\t"
        #    self.turnright = True
        #    self.turnleft = False
        #    dowalking = False
        #else: # Do nothing
        #    self.turnright = False
        #    self.turnleft = False
        #    self.turn = False

        if dowalking:
            # Walk joystick is left joystick
            # Default handling in original Commander.c - sets to range of -127 to 127 or so...
            # vals - 128 gives look a value in the range from -128 to 127?
            self.walkV = self.crx.walkv
            self.walkH = self.crx.walkh#v
        #else:
        #    self.walkV = 0
        #    self.walkH = 0

        # Look joystick is right joystick
        if self.slowturret:
            pan_add = int(-self.crx.lookh / 40)#17) # 17 is old value from Numa1
        else:
            pan_add = int(-self.crx.lookh / 10)
        tilt_add = int(-self.crx.lookv / 25)

        self.pan_pos = clamp(self.pan_pos + pan_add, self.servo51Min, self.servo51Max)
        self.tilt_pos = clamp(self.tilt_pos + tilt_add, self.servo52Min, self.servo52Max)

        if out:
            print("Output:", out)
        return


    def bb_loader_service(self, loopStart, adcval_effort, adcval_loaded):
        # If in timeout, check if we've exceeded timeout duration
        enable_led = True
        if not self.cmdrAlive or not self.enable_bb_loader:
            enable_led = False
            if self.ammoMotor.actualSpeed != 0:
                self.ammoMotor.direct_set_speed(LOADER_SPEED_OFF)
        elif self.loader_timeout_mode == "timeout":
            self.ammoMotor.direct_set_speed(LOADER_SPEED_OFF)
            if loopStart > self.loader_timeout_end:
                self.loader_timeout_mode = "running"
                self.ammoMotor.direct_set_speed(LOADER_SPEED_ON)
                print("resuming loading ammo!")
            # TODO if we transition from timeout to running, bb_detect_adc_loop_cnt should still be high?
            #if adcval_loaded < self.MIN_ADC_NO_BB and self.loader_timeout_mode != "rewind":  # Only check this in forward dir
            #    self.bb_detect_adc_loopcnt += 1
        elif self.loader_timeout_mode == "jammed":
            self.ammoMotor.direct_set_speed(LOADER_SPEED_OFF)
            # TODO bug: wraparound in calculation of loader_timeout_end; reference loopCount
            if loopStart > self.loader_timeout_end:
                self.loader_timeout_mode = "rewind"
                self.loader_reverse_end = loopStart + 250 # ms
                # Run the motor backwards very briefly
                self.ammoMotor.run_reversed()
                self.ammoMotor.direct_set_speed(LOADER_SPEED_ON)
                print("rewinding for 250ms!")
        elif self.loader_timeout_mode == "rewind":
            if loopStart > self.loader_reverse_end:
                self.loader_timeout_mode = "running"
                self.ammoMotor.direct_set_speed(LOADER_SPEED_OFF)
                print("resuming bb loading")
        elif adcval_loaded < self.MIN_ADC_NO_BB and self.loader_timeout_mode != "rewind":  # Only check this in forward dir
            self.bb_detect_adc_loopcnt += 1
            print("adcval_loaded detected BB; adcval:", adcval_loaded, self.bb_detect_adc_loopcnt)
            if self.bb_detect_adc_loopcnt > 10: # ~200 ms
                print("entering timeout (bb detected)")
                self.ammoMotor.direct_set_speed(LOADER_SPEED_OFF)
                self.loader_timeout_mode = "timeout"
                self.loader_timeout_end = loopStart + LOADER_TIMEOUT_DURATION
        elif adcval_effort > ADC_LOADER_LIMIT and self.loader_timeout_mode != "rewind":  # Only check this in forward dir
            print("adcval_effort exceeded 10:", adcval_effort)
            self.ammoMotor.direct_set_speed(LOADER_SPEED_OFF)
            self.loader_timeout_mode = "jammed"
            self.loader_timeout_end = loopStart + LOADER_TIMEOUT_DURATION
            self.bb_detect_adc_loopcnt = 0
        else:  # running
            #print(LOADER_SPEED_ON,  self.ammoMotor.cs_adc.read())
            self.ammoMotor.run_forward()
            self.ammoMotor.direct_set_speed(LOADER_SPEED_ON)
            self.bb_detect_adc_loopcnt = 0

        if bool(self.bb_detect_led.value()) != enable_led:
            self.bb_detect_led.value(1 if enable_led else 0)

        # -------- Start Analogue Input-------
        # Dump out the raw value for Analogue Input
        # Dump out the mV (milli-volts) for Analogue Input
        #print("m1current: " << a2dConvert10bit(ADC_CH_ADC10) << "  m1current: " << a2dReadMv(ADC_CH_ADC10)"mV")

    def print_gait_positions(self):
        for cnt, x in enumerate([self.gaits.s11pos, self.gaits.s21pos, self.gaits.s31pos, self.gaits.s41pos,
                        self.gaits.s12pos, self.gaits.s13pos, self.gaits.s14pos, self.gaits.s22pos,
                        self.gaits.s23pos, self.gaits.s24pos, self.gaits.s32pos, self.gaits.s33pos,
                        self.gaits.s34pos, self.gaits.s42pos, self.gaits.s43pos, self.gaits.s44pos]):
            print(cnt, x)

def main():
    leg_geom, leg1, leg2, leg3, leg4 = gen_numa2_legs()
    gaits = Gaits(leg_geom, leg1, leg2, leg3, leg4)
    x = NumaMain(gaits)
    # Safety...
    x.gunMotor.direct_set_speed(GUN_SPEED_OFF)
    x.ammoMotor.direct_set_speed(LOADER_SPEED_OFF)
    #input("Waiting... (press enter)")
    #input("Now onwards! (press enter again)")
    try:
        x.main(leg_geom)
    except BaseException as e:
        print("Safely stopping gun!")
        x.gunMotor.direct_set_speed(GUN_SPEED_OFF)
        print("Safely stopping loader!")
        x.ammoMotor.direct_set_speed(LOADER_SPEED_OFF)
        x.laserGPIO.value(0)
        x.bb_detect_led.value(0)
        if "can't convert NaN to int" in str(e):
            print("NaN value generated...")
            x.print_gait_positions()
            print(" ^-- Previous set of positions...")
        raise

if __name__ == "__main__":
    main()
