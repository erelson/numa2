import utime
import struct

import ax

# Send standing positions to all servos.
def g8Stand(axbus, leg_ids):

    s11pos = 639 + 20
    s21pos = 383 - 20
    s31pos = 639
    s41pos = 383
    s12pos = 358
    s22pos = 664
    s32pos = 358
    s42pos = 664
    s13pos = 664
    s23pos = 358
    s33pos = 664
    s43pos = 358
    s14pos = 511#409+39)
    s24pos = 511#613-39)
    s34pos = 511#409+39)
    s44pos = 511#613-39)

    axbus.sync_write(leg_ids, ax.GOAL_POSITION,
            [struct.pack('<H', pos) for pos in
                       (s11pos, s21pos, s31pos, s41pos,
                       s12pos, s13pos, s14pos, s22pos,
                       s23pos, s24pos, s32pos, s33pos,
                       s34pos, s42pos, s43pos, s44pos)])
#    utime.sleepms(2000)

    # stop IK and Gait from processing, whichever was active...
    #walk = False
    return False

# Send standing positions to all servos. BUT don't rotate legs to center position
def g8FeetDown(axbus, leg_ids):
    #s11pos = 639 + 20
    #s21pos = 383 - 20
    #s31pos = 639
    #s41pos = 383
    # skip the shoulders
    my_leg_ids = leg_ids[1:4] + leg_ids[5:8] + leg_ids[9:12] + leg_ids[13:16]
    s12pos = 358
    s22pos = 664
    s32pos = 358
    s42pos = 664
    s13pos = 664
    s23pos = 358
    s33pos = 664
    s43pos = 358
    s14pos = 511#409+39)
    s24pos = 511#613-39)
    s34pos = 511#409+39)
    s44pos = 511#613-39)
    axbus.sync_write(my_leg_ids, ax.GOAL_POSITION,
            [struct.pack('<H', pos) for pos in
                       (s12pos, s13pos, s14pos, s22pos,
                       s23pos, s24pos, s32pos, s33pos,
                       s34pos, s42pos, s43pos, s44pos)])

    #utime.sleepms(2000) #probably too short, but a long wait is scary, too.

    # stop IK and Gait from processing, whichever was active...
    #walk = False
    return False

# Send standing positions to all servos.
def g8Flop(axbus, leg_ids):

    s11pos = 639 + 20
    s21pos = 383 - 20
    s31pos = 639
    s41pos = 383

    s12pos = 358 - 105
    s22pos = 664 + 105
    s32pos = 358 - 105
    s42pos = 664 + 105

    s13pos = 358 - 130
    s23pos = 664 + 130
    s33pos = 358 - 130
    s43pos = 664 + 130

    s14pos = 511 #409+39)
    s24pos = 511 #613-39)
    s34pos = 511 #409+39)
    s44pos = 511 #613-39)

    axbus.sync_write(leg_ids, ax.GOAL_POSITION,
            [struct.pack('<H', pos) for pos in
                       (s11pos, s21pos, s31pos, s41pos,
                       s12pos, s13pos, s14pos, s22pos,
                       s23pos, s24pos, s32pos, s33pos,
                       s34pos, s42pos, s43pos, s44pos)])
#    utime.sleepms(2000) #probably too short, but a long wait is scary, too.

    # stop IK and Gait from processing, whichever was active...
    #walk = False
    return False

def g8Crouch(axbus, leg_ids):
    my_leg_ids = leg_ids[1:4] + leg_ids[5:8] + leg_ids[9:12] + leg_ids[13:16]

    s12pos = 358 - 175
    s22pos = 664 + 175
    s32pos = 358 - 175
    s42pos = 664 + 175
    # TODO 350 and 150 are for ...?
    s13pos = 358 + 350 + 150
    s23pos = 664 - 350 - 150
    s33pos = 358 + 350 + 150
    s43pos = 664 - 350 - 150
    s14pos = 511 #409+39)
    s24pos = 511 #613-39)
    s34pos = 511 #409+39)
    s44pos = 511 #613-39)

#    utime.sleepms(2000) #probably too short, but a long wait is scary, too.

    # stop IK and Gait from processing, whichever was active...
    #walk = False
    # TODO wtf lol... noticed 11-25 that I have a return here and am not sending positions
    return False

    utime.sleepms(200)

    axbus.sync_write(my_leg_ids, ax.GOAL_POSITION,
            [struct.pack('<H', pos) for pos in
                       (s12pos, s13pos, s14pos, s22pos,
                        s23pos, s24pos, s32pos, s33pos,
                        s34pos, s42pos, s43pos, s44pos)])

    # Disable torque to 2nd servo of each leg
    axbus.sync_write(leg_ids[1::4], ax.TORQUE_ENABLE, [struct.pack('<H', 0)
                                                      for _ in range(4)])
