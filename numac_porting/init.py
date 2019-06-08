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
    l1, l2, l3, l4 = gaits.leg1, gaits.leg2, gaits.leg3, gaits.leg4
    # Set the limits for motion range on the servos
    # NOTE order does not match leg_ids; we sort `all_ids` to match this order later
    lims = [ # CW        # CCW
       [l1.s1min, l1.s1max],
       [l1.s2min, l1.s2max],
       [l1.s3min, l1.s3max],
       [l1.s4min, l1.s4max],
       [l2.s1min, l2.s1max],
       [l2.s2min, l2.s2max],
       [l2.s3min, l2.s3max],
       [l2.s4min, l2.s4max],
       [l3.s1min, l3.s1max],
       [l3.s2min, l3.s2max],
       [l3.s3min, l3.s3max],
       [l3.s4min, l3.s4max],
       [l4.s1min, l4.s1max],
       [l4.s2min, l4.s2max],
       [l4.s3min, l4.s3max],
       [l4.s4min, l4.s4max],
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


COAX_SPEED = 200
SERVO_SPEED = 300
TURRET_SERVO_SPEED = 200
def myServoSpeeds(axbus, leg_ids, turret_ids):
    axbus.sync_write(leg_ids, ax.MOVING_SPEED, [struct.pack('<H', SERVO_SPEED) for _ in range(len(leg_ids))])
    sleep_ms(25)
    axbus.sync_write(leg_ids[4:8], ax.MOVING_SPEED, [struct.pack('<H', COAX_SPEED) for _ in range(4)])
    sleep_ms(25)
    axbus.sync_write(turret_ids, ax.MOVING_SPEED, [struct.pack('<H', TURRET_SERVO_SPEED), struct.pack('<H', TURRET_SERVO_SPEED)])


RTN_LVL = 1
def myServoReturnLevels(axbus, all_ids):
    axbus.sync_write(all_ids, ax.RETURN_LEVEL, [bytearray([RTN_LVL]) for _ in range(len(all_ids))])
    sleep_ms(25)
