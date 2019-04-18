import struct
import sys
sysname = sys.platform
if sysname == 'linux' or sysname == 'win32':
    # Mocks for non-pyboard use
    import time
    def sleep_ms(x): return time.sleep(x/1e3)
elif sysname == 'pyboard':
    from utime import sleep_ms

import ax


# See WalkingOmni.nb
#Center angles for coax servo of each leg.plus test offsets.
#Measured from 0deg = front of bot  --------- this doesn't look right.
#/But are the

# Servo position limits, range from 0 to 1023
# From AX12 manual: CW Angle Limit <= Goal Position <= CCW
PAN_CENTER = 511 + 153


def initServoLims(axbus, _all_ids, gaits):
    leg1, leg2, leg3, leg4 = gaits.leg1, gaits.leg2, gaits.leg3, gaits.leg4
    # Set the limits for motion range on the servos
    # NOTE order does not match leg_ids; we sort `all_ids` to match this order later
    lims = [ # CW        # CCW
            [leg1.s1max, leg1.s1min],
            [leg1.s2max, leg1.s2min],
            [leg1.s3max, leg1.s3min],
            [leg1.s4max, leg1.s4min],
            [leg2.s1max, leg2.s1min],
            [leg2.s2max, leg2.s2min],
            [leg2.s3max, leg2.s3min],
            [leg2.s4max, leg2.s4min],
            [leg3.s1max, leg3.s1min],
            [leg3.s2max, leg3.s2min],
            [leg3.s3max, leg3.s3min],
            [leg3.s4max, leg3.s4min],
            [leg4.s1max, leg4.s1min],
            [leg4.s2max, leg4.s2min],
            [leg4.s3max, leg4.s3min],
            [leg4.s4max, leg4.s4min],
            [PAN_CENTER - 4 * (52+30),  PAN_CENTER + 4 * (52+30)], # 51
            [511 - 4 * 31,              511 + 4 * 65], # 52
    ]

    # sync_write(dev_ids, offset, values):
    all_ids = _all_ids.copy()
    all_ids.sort()
    axbus.sync_write(all_ids, ax.CW_ANGLE_LIMIT_L, [struct.pack('<H', lim[0]) for lim in lims])
    sleep_ms(25)
    axbus.sync_write(all_ids, ax.CCW_ANGLE_LIMIT_L, [struct.pack('<H', lim[1]) for lim in lims])
    sleep_ms(25)
    axbus.sync_write(all_ids, ax.TORQUE_ENABLE, [bytearray([1]) for _ in range(len(all_ids))])
    sleep_ms(25)


MY_COAX_SPEED = 200
MY_SERVO_SPEED = 300
MY_TURRET_SERVO_SPEED = 200
def myServoSpeeds(axbus, leg_ids, turret_ids):
    axbus.sync_write(leg_ids, ax.MOVING_SPEED, [struct.pack('<H', MY_SERVO_SPEED) for _ in range(len(leg_ids))])
    #for cnt in range(16):
    #    ax12SetMOVING_SPEED( (AX12_driver_list[cnt]), MY_SERVO_SPEED)
    #    sleep_ms(25)
    sleep_ms(25)
    axbus.sync_write(leg_ids[4:8], ax.MOVING_SPEED, [struct.pack('<H', MY_COAX_SPEED) for _ in range(4)])
    sleep_ms(25)
    axbus.sync_write(turret_ids, ax.MOVING_SPEED, [struct.pack('<H', MY_TURRET_SERVO_SPEED), struct.pack('<H', MY_TURRET_SERVO_SPEED)])


RTN_LVL = 1
def myServoReturnLevels(axbus, all_ids):
    axbus.sync_write(all_ids, ax.RETURN_LEVEL, [bytearray([RTN_LVL]) for _ in range(len(all_ids))])
    sleep_ms(25)
