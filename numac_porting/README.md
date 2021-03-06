# Notes

## Encoding per-leg servo offsets and orientations, and limits

There are a few considerations from the physical build that need to
be considered in order to generate servo positions from the leg
angles generated by the inverse kinematics.

- (A) At any given joint, a servo can be mounted in two way. We
  need to be able to indicate which orientation the servo is in.
- (B) In our generic IK model, angles are measured as the angle
  between one leg segment and the next (angle is zero if they're
  colinear). Servos are not necessarily mounted in-line with the
  leg segments, so an angle offset the servo is mounted at is
  needed for calculating servo position.

Assuming all four legs use the same geometry,
for the shoulder servos, generally (A) is the same for all of them,
but there are different (B).  For the rest of the servos in the
leg, (B) is the same for all legs, but (A) does change.

Additionally, to balance the robot, (especially for walking where
opposite leg pairs are off the ground) the default direction that
each leg points needs to be tweaked.

To minimally encode this value I define "stance", which can be
thought of as an angle offset for the legs' neutral position
from a default of 45 degrees.  Positive stance moves the front
pair of legs forward, and the rear pair back, and vice versa.
Optionally, a "rear stance" can be specified, giving separate
front and back offsets.
```
            fwd
       \     ^     / 
        \    |    /  
         \4_____3/   
          |     |    
          |numa |    
          |_____|    
         /1     2\   
        /         \  
       /           \ 
```

Given the case where the servos are mounted side-to-side (vs front-to-back),
this gives us the following "default" offsets per leg,
i.e. the zero position of the leg is
`leg=0 == servo_pos=center+/-45`

Aka, the center position of the legs corresponds with servo positions:
`Leg 1: center+45deg`
`Leg 2: center-45deg`
`Leg 3: center+45deg`
`Leg 4: center-45deg`

And with stance (3/4 are front, 1/2 are rear)

`Leg 1: center+45deg + rear_stance`
`Leg 2: center-45deg - rear_stance`
`Leg 3: center+45deg + stance`
`Leg 4: center-45deg - stance`

To encode this +/- trend, we define the `sign=+1` or `sign=-1` for the
servo so that every servo's default postiion is `sign * (45deg + stance)`

Since AX-12 servos have a 300 degree range of motion, the center is 150 degrees.
So we end up with the generic center position in degrees of
`150 + sign * (45 + stance)`

## Other

For the shoulder aka coax (first) servo, we end up calculating the angle in
a leg-centric coordinate frame, that looks like:

```
90
|      leg
|     /
|    /
|   /
|  /
| /
|/_______________ 0
```

Generally, in mech designs:
- The coax servo is mounted aligned with either the 0 or 90 degree directions.
- The robot's "neutral" position puts each leg at around 45 degrees in its
  coordinate frame
  - The stance variables above change this value from 45 degrees

The walking gait defines the change in leg angle as a function of the
direction the robot is walking in, e.g. `dA(dir, t)`


The coax servo position is then a combination of
- mount offset : In this case, +/- 45 degrees; specified by the leg's `LegGeom.aoffset1 * LegDef.s1_sign`
- offset from 45 degrees (+/- stance) : `LegGeom.a1stance` or `LegGeom.a1stance_rear`
- dynamic offset angle from the gait `dA(dir, t)` : calculated in `walk_code()` and `turn_code()`
  - Walking: This is a function of the current time in the gait, and the vector of travel.
  - Turning: This is a function of the current time in the gait, and inexact foot positioning. (TODO factor in distance between coax servos and have feet actually follow the proper arc during turning.)
  - Both walking and turning assign different `t` to different legs in order to achieve
    coordinated movement.

`ax12pos(mount + offset + dA)`

    self.s11pos = get_pos_from_angle( 45 + self.s11Aoff + turn_dir * gait_phase13)


Sometimes we specify poses/gaits in terms of angles in degrees, and other times in radians.
- Degrees: For static poses, like crouching, we define joint angles in degrees (since I can more automatically
visualize a given angle in degrees than a value in radians)
- Radians: Both the walking IK and turning IK generate angles



For servos in the leg, with a 2D leg geometry in the XY plane, and the servo rotation
about the Z axis, there are the following factors:
1. Servo body is before or after the joint (leg analogy: is the knee servo in the thigh or the calf?)
2. Servo mount results in either +/- Z  (Right hand rule...)
3. Servo body alignment with leg segment
4. Servo horn alignment with other leg segment

These correspond with the following factors which are either per leg, or generic
to all identical legs:
1. Generic: +/-1 scale factor - `sign_g`
2. Per leg: +/-1 scale factor - `sign_l`
3. Generic: angular offset - `off_b`  Measured as the angle between the leg vector and the servo body axis vector
4. Generic: angular offset - `off_h`

Here are a couple examples showing how these vary:


And these are combined with the joint angle dA determined by the walking gait. Note: `dA` is
the angle difference of the two segments being colinear from the two segments' desired position:

#TODO draw out how the offsets combine
`position_ax12 = 511 + ( sign_g * (off_b - off_h) + sign_l * sign_g * dA ) * 1024/300`

The walking gait defines the change in leg angle as a function of the
direction the robot is walking in, e.g. `dA(dir, t)`


## Transitioning between different speed gaits

If walking changes to a different speed, then we want to smoothly continue at the new speed.

At a given speed, we have a loop length of the gait, LL1. If we are X% through a loop,
then when we move to a new speed, with loop length LL2, we should stay at X% through LL2.
Thus, if we're tracking our temporal position in the loop, `t`, then
`t = X/100 * LL1` --> `t = X/100 * LL2`.  Thus we want to change `t -> t + dt`, and we
can state that `dt = X/100 * LL2 - X/100 * LL1`.

However, we use a slightly different implementation.

We calculate the current loop time as `t = (t_clock + speed_offset) % LL`

 TODO finish this section lol
