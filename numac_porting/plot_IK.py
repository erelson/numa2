# This script is used for plotting the IK values to visually ensure good behavior

from argparse import ArgumentParser

import numpy
import pylab

from IK import *


class Wrapper():

    def __init__(self,
                 timestep=20, # ms
                 ang_dir=0,
                 foot_h_max=FH,
                 time_down_frac=ALL_FEET_DOWN_TIME_FRAC,
                 #half_loopLength=,
                 transition_frac=TRANSITION_FRAC,
                 height_frac=FH_FRAC
            ):
        self.timestep = timestep
        self.ang_dir = ang_dir
        self.foot_h_max = foot_h_max
        self.time_down_frac = time_down_frac
        #self.half_loopLength = half_loopLength
        self.transition_frac = transition_frac
        self.height_frac = height_frac

        self.times= []
        self.points = []

    def walk_angle(self, travRate=25, #mm
                      loopLength=10000):
        """"TODO"""
        self.times= []
        self.points = []

        half_loopLength = loopLength/2.0
        double_travRate = 2 * travRate
        gait = Gaits()

        gait.initTrig()

        turnTimeOffset = 0 # unused usually

        for ms in numpy.arange(0, loopLength, self.timestep):
            now2 = (ms - turnTimeOffset) % loopLength
            now3 = (ms - turnTimeOffset +  half_loopLength) % loopLength
            now4 = loopLength - (ms - turnTimeOffset) % loopLength
            now1 = loopLength - (ms - turnTimeOffset + half_loopLength) % loopLength

            #calc_foot_h(now, foot_h_max, time_down_frac, half_loopLength, transition_frac, height_frac)

            gait.walkCode(loopLength, half_loopLength, travRate, double_travRate, now1, now2, now3, now4, self.ang_dir)

            self.times.append(ms)
            self.points.append([gait.s11pos, gait.s12pos, gait.s13pos, gait.s14pos,
                               gait.s21pos, gait.s22pos, gait.s23pos, gait.s24pos,
                               gait.s31pos, gait.s32pos, gait.s33pos, gait.s34pos,
                               gait.s41pos, gait.s42pos, gait.s43pos, gait.s44pos]
                              )

        self.points = numpy.array(self.points)



def plot(times, points, block=True):
    if len(points) == 0:
        print("No Data to Plot")
        return
    desc = "arm_test"
    fig = pylab.figure(desc)
    t = pylab.array(times)
    t = (t- t[0])/60.0
    ax1 = fig.add_subplot(1,1,1)
    #line1 = ax1.plot(t, self.forward_efforts,'r',label='forward effort')
    #line2 = ax1.plot(t, reverse_efforts,'b',label='reverse effort')
    ax1.set_ylabel("Position")
    #ax2 = ax1.twinx()
    #ax2.set_ylabel("Unused")
    # Single legend
    lns = [ax1.plot(t, reverse_efforts,'b',label='reverse effort') for n in xrange(len(points[0]))]
    #lns = line1 + line2 + line3
    labs = [l.get_label() for l in lns]
    ax1.legend(lns, labs, loc=0)

    ax1.set_title("Positions from {0}-{1}".format(times[0], times[-1]))
    pylab.show()#block=block)


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-x", action="store",
                        help="TODO")

    args = parser.parse_args()

    run = Wrapper()
    run.walk_angle()
    plot(run.times, run.points[0::4])
