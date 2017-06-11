#include <math.h>
#include "HeaderDefs.h"

#define zTRUE 1
#define zFALSE 0

#define RTN_LVL 1
#define MY_COAX_SPEED 200
#define MY_SERVO_SPEED 300
#define MY_TURRET_SERVO_SPEED 200


#Trig

# pos and min and max are uint16_t originally

# TODO don't need this 
#Servo position variables defaulting these to centered...
s11pos = 511
s12pos = 511
s13pos = 511
s14pos = 511

s21pos = 511
s22pos = 511
s23pos = 511
s24pos = 511

s31pos = 511
s32pos = 511
s33pos = 511
s34pos = 511

s41pos = 511
s42pos = 511
s43pos = 511
s44pos = 511

s51pos = 511 # added a 45° offset
s52pos = 511


# See WalkingOmni.nb
#Center angles for coax servo of each leg.plus test offsets.
#Measured from 0deg = front of bot  --------- this doesn't look right.
#/But are the 
# #define s11A0 225
# #define s11Aoff 10
# #define s21A0 -45 //315 //Hmmm no idea here....
# #define s21Aoff -10
# #define s31A0 45
# #define s31Aoff 0
# #define s41A0 135
# #define s41Aoff 10


# #define c_s11A0 cos(s11A0 + s11Aoff)
# #define s_s11A0 sin(s11A0 + s11Aoff)
# #define c_s21A0 cos(s21A0 + s21Aoff)
# #define s_s21A0 sin(s21A0 + s21Aoff)
# #define c_s31A0 cos(s31A0 + s31Aoff)
# #define s_s31A0 sin(s31A0 + s31Aoff)
# #define c_s41A0 cos(s41A0 + s41Aoff)
# #define s_s41A0 sin(s41A0 + s41Aoff)



float cos_servo11Ang = 0
float sin_servo11Ang = 0
float cos_servo21Ang = 0
float sin_servo21Ang = 0
float cos_servo31Ang = 0
float sin_servo31Ang = 0
float cos_servo41Ang = 0
float sin_servo41Ang = 0

# Leg consts
uint16_t bodyH = 130 + 20 #mm

uint16_t L0 = 105 + 5 #mm - pretty close to actual...

#Leg part lengths
preL12 58
preL23 63
preL34 67
preL45 57

y11 = 0
y21 = 0
y31 = 0
y41 = 0

L12 = preL12
L23 = preL23
L34 = preL34
L45 = preL45

sqL23 = preL23 * preL23
sqL34 = preL34 * preL34
#not used:
#int sqL45 = preL45 * preL45

#????????
int x11 = preL12
int x21 = preL12
int x31 = preL12
int x41 = preL12

#Leg ServoPos Offsets
#  this is in units of... 10bit position? of 300 deg range?
#  currently not used... manually entered values are in 
int offsetServo1 = 0
int offsetServo2 = -237 # 70 
int offsetServo3 = -70
int offsetServo4 = 0*39 - 0*20 #39 per CAD with original feet.

# 4*52 -> 60 degrees?
uint16_t servo51Min = PAN_CENTER - 4 * (52+30)# - 200
uint16_t servo51Max = PAN_CENTER + 4 * (52+30)# + 200
uint16_t servo52Min = 511 - 4 * 31# - 200
uint16_t servo52Max = 511 + 4 * 65# + 200


# Bools
turnright = zFALSE
turnleft = zFALSE
gunbutton = zFALSE
panicbutton = zFALSE
infobutton = zFALSE
agitbutton = zFALSE

TICK_COUNT agitate = 0

walkV = 0
walkH = 0
lookV = 0
lookH = 0
walkSPD = 0 #int
walkDIR = 0 #float
