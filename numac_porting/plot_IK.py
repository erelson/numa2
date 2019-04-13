# This script is used for plotting the IK values to visually ensure good behavior

from argparse import ArgumentParser

import numpy
import pylab

#from IK import *
from IK import ALL_FEET_DOWN_TIME_FRAC, TRANSITION_FRAC, FH_FRAC, FH, Gaits
from poses import gen_numa2_legs

NUMA_SERVO_LOOKUP = \
        [11, 12, 13, 14,
         21, 22, 23, 24,
         31, 32, 33, 34,
         41, 42, 43, 44]

class Wrapper():

    def __init__(self,
                 timestep=20, # ms
                 ang_dir=0.0,
                 foot_h_max=FH,
                 time_down_frac=ALL_FEET_DOWN_TIME_FRAC,
                 #half_loopLength=,
                 transition_frac=TRANSITION_FRAC,
                 height_frac=FH_FRAC
            ):
        self.timestep = timestep # ms
        self.ang_dir = ang_dir # degrees
        self.foot_h_max = foot_h_max # mm
        self.time_down_frac = time_down_frac
        #self.half_loopLength = half_loopLength
        self.transition_frac = transition_frac
        self.height_frac = height_frac

        self.times= []
        self.points = []

    def walk_angle(self, travRate=25, #mm
                      loopLength=10000, repeat_count=1):
        """Invokes walk_code, which invokes doLegKinem
        """
        self.times = []
        self.points = []

        half_loopLength = loopLength/2.0
        double_travRate = 2 * travRate
        leg_geom, leg1, leg2, leg3, leg4 = gen_numa2_legs()
        gait = Gaits(leg_geom, leg1, leg2, leg3, leg4)

        gait.initTrig()

        turnTimeOffset = 0 # unused usually

        for cnt in range(repeat_count):
            offset = cnt * loopLength
            for ms in numpy.arange(0, loopLength, self.timestep):
                now2 = (ms - turnTimeOffset) % loopLength
                now3 = (ms - turnTimeOffset +  half_loopLength) % loopLength
                now4 = loopLength - (ms - turnTimeOffset) % loopLength
                now1 = loopLength - (ms - turnTimeOffset + half_loopLength) % loopLength

                #calc_foot_h(now, foot_h_max, time_down_frac, half_loopLength, transition_frac, height_frac)

                gait.walk_code(loopLength, half_loopLength, travRate, double_travRate, now1, now2, now3, now4, self.ang_dir)

                self.times.append(ms + offset)
                self.points.append([gait.s11pos, gait.s12pos, gait.s13pos, gait.s14pos,
                                   gait.s21pos, gait.s22pos, gait.s23pos, gait.s24pos,
                                   gait.s31pos, gait.s32pos, gait.s33pos, gait.s34pos,
                                   gait.s41pos, gait.s42pos, gait.s43pos, gait.s44pos]
                                  )

        self.points = numpy.array(self.points)
        print(len(self.points))


STYLES = [
        'b--',
        'b:',
        'b-.',
        'b-',
        'r--',
        'r:',
        'r-.',
        'r-',
        'k--',
        'k:',
        'k-.',
        'k-',
        'g--',
        'g:',
        'g-.',
        'g-',
        ]


def plot(times, points, block=True, servos=None):
    if len(points) == 0:
        print("No Data to Plot")
        return
    desc = "arm_test"
    fig = pylab.figure(desc)
    t = pylab.array(times)
    t = (t - t[0])/60.0
    t = t[:len(points)] # truncate
    ax1 = fig.add_subplot(1,1,1)
    #line1 = ax1.plot(t, self.forward_efforts,'r',label='forward effort')
    #line2 = ax1.plot(t, reverse_efforts,'b',label='reverse effort')
    ax1.set_ylabel("Position")
    #ax2 = ax1.twinx()
    #ax2.set_ylabel("Unused")

    # Single legend
    if not servos:
        lns = [ax1.plot(t, [x[n] for x in points], STYLES[n], label='servo{0}'.format(NUMA_SERVO_LOOKUP[n]))
                for n in range(len(points[0]))]
    else:
        lns = [ax1.plot(t, [x[n] for x in points], STYLES[n], label='servo{0}'.format(NUMA_SERVO_LOOKUP[n]))
                for n in range(len(points[0])) if str(NUMA_SERVO_LOOKUP[n]) in servos]
    ax1.legend()#lns, loc=0)

    ax1.set_title("Positions from {0}-{1}".format(times[0], times[-1]))
    pylab.show()#block=block)


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-r", "--repeat", action="store", default=1, type=int,
                        help="Number of gait loops to simulate")
    parser.add_argument("-s", "--servos", action="store", default=None, nargs="*",
                        help="Servo IDs to plot (as space delimited list of args)")
    parser.add_argument("-w", "--walkdir", action="store", default=0.0, type=float,
                        help="Servo IDs to plot (as space delimited list of args)")

    args = parser.parse_args()

    run = Wrapper(ang_dir=args.walkdir)
    run.walk_angle(repeat_count=args.repeat)
    #mypoints = [x[0::4] for x in run.points]
    mypoints = run.points
    print(mypoints[0])
    plot(run.times, mypoints, servos=args.servos)
    print("hmm")
