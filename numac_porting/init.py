import struct
import utime

import ax


# See WalkingOmni.nb
#Center angles for coax servo of each leg.plus test offsets.
#Measured from 0deg = front of bot  --------- this doesn't look right.
#/But are the

# Servo position limits, range from 0 to 1023
# From AX12 manual: CW Angle Limit <= Goal Position <= CCW
PAN_CENTER = 511 + 153
# NOTE order does not match leg_ids
lims = [   # ID  # CW           # CCW
            [11, 511 - 4 * 16, 511 + 4 * 55],
            [12, 511 - 4 * 84, 511 + 4 * 27],
            [13, 511 - 4 * 77, 511 + 4 * 87],
            [14, 511 - 4 * 77, 511 + 4 * 77],

            [21, 511 - 4 * 55, 511 + 4 * 16],
            [22, 511 - 4 * 27, 511 + 4 * 84],
            [23, 511 - 4 * 87, 511 + 4 * 77],
            [24, 511 - 4 * 77, 511 + 4 * 77],

            [31, 511 - 4 * 16, 511 + 4 * 55],
            [32, 511 - 4 * 84, 511 + 4 * 27],
            [33, 511 - 4 * 77, 511 + 4 * 87],
            [34, 511 - 4 * 77, 511 + 4 * 77],

            [41, 511 - 4 * 55, 511 + 4 * 16],
            [42, 511 - 4 * 27, 511 + 4 * 84],
            [43, 511 - 4 * 87, 511 + 4 * 77],
            [44, 511 - 4 * 77, 511 + 4 * 77],

            [51, PAN_CENTER - 4 * (52+30),  PAN_CENTER + 4 * (52+30)],
            [52, 511 - 4 * 31,              511 + 4 * 65],
        ]


def initServoLims(axbus):
    # Set the limits for motion range on the servos

    all_ids = [s[0] for s in lims]
    axbus.sync_write(all_ids, ax.CW_ANGLE_LIMIT_L, [struct.pack('<H', lim[1]) for lim in lims])
    utime.sleep_ms(25)
    axbus.sync_write(all_ids, ax.CCW_ANGLE_LIMIT_L, [struct.pack('<H', lim[2]) for lim in lims])
    utime.sleep_ms(25)
    axbus.sync_write(all_ids, ax.TORQUE_ENABLE, [struct.pack('<H', 1) for _ in range(len(all_ids))])
    utime.sleep_ms(25)


MY_COAX_SPEED = 200
MY_SERVO_SPEED = 300
MY_TURRET_SERVO_SPEED = 200
def myServoSpeeds(axbus, leg_ids, turret_ids):
    axbus.sync_write(leg_ids, ax.MOVING_SPEED, [bytearray([MY_SERVO_SPEED]) for _ in range(len(leg_ids))])
    #for cnt in range(16):
    #    ax12SetMOVING_SPEED( (AX12_driver_list[cnt]), MY_SERVO_SPEED)
    #    utime.sleep_ms(25)
    utime.sleep_ms(25)
    axbus.sync_write(leg_ids[4:8], ax.MOVING_SPEED, [bytearray([MY_COAX_SPEED]) for _ in range(4)])
    utime.sleep_ms(25)
    axbus.sync_write(turret_ids, ax.MOVING_SPEED, [bytearray([MY_TURRET_SERVO_SPEED]), bytearray([MY_TURRET_SERVO_SPEED])])


RTN_LVL = 1
def myServoReturnLevels(axbus, all_ids):
    axbus.sync_write(all_ids, ax.RETURN_LEVEL, [bytearray([RTN_LVL]) for _ in range(len(all_ids))])
    utime.sleep_ms(25)
