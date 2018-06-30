import time

from IK import *

NUMA_SERVO_LOOKUP = \
        [11, 12, 13, 14,
         21, 22, 23, 24,
         31, 32, 33, 34,
         41, 42, 43, 44]

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
                      loopLength=10000): #ms
        """"TODO"""
        self.times= []
        self.points = []

        half_loopLength = loopLength/2.0
        double_travRate = 2 * travRate
        gait = Gaits()

        gait.initTrig()

        turnTimeOffset = 0 # unused usually

        for step in range(0, int(loopLength/self.timestep)):
            ms = step * self.timestep
            now2 = (ms - turnTimeOffset) % loopLength
            now3 = (ms - turnTimeOffset +  half_loopLength) % loopLength
            now4 = loopLength - (ms - turnTimeOffset) % loopLength
            now1 = loopLength - (ms - turnTimeOffset + half_loopLength) % loopLength

            #calc_foot_h(now, foot_h_max, time_down_frac, half_loopLength, transition_frac, height_frac)

            gait.walkCode(loopLength, half_loopLength, travRate, double_travRate, now1, now2, now3, now4, self.ang_dir)

            # Can't save all this data, lol
            #self.times.append(ms)
            #self.points.append([gait.s11pos, gait.s12pos, gait.s13pos, gait.s14pos,
            #                   gait.s21pos, gait.s22pos, gait.s23pos, gait.s24pos,
            #                   gait.s31pos, gait.s32pos, gait.s33pos, gait.s34pos,
            #                   gait.s41pos, gait.s42pos, gait.s43pos, gait.s44pos]
            #                  )

        #self.points = numpy.array(self.points)


if __name__ == "__main__":


    time.sleep(10)

    run = Wrapper()
    start_time = time.ticks_us()
    looplength = 10000
    run.walk_angle(loopLength=looplength)
    end_time = time.ticks_us()
    #mypoints = [x[0::4] for x in run.points]
    mypoints = run.points
    #print mypoints[0]

    print("Took {0} seconds to do {1} iterations. Uses {2}% of specified step time."
            "".format((end_time - start_time)/1.e6, looplength/run.timestep, 
            100 * (end_time - start_time)/1.e3 / (looplength/run.timestep) / run.timestep))
