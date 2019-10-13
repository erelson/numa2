from math import sqrt, pi
from IK import Gaits, calc_foot_h
from poses import gen_numa2_legs
import unittest

RAD_TO_ANGLE = 180./pi

class TestGait(Gaits):
    def __init__(self):
        pass

class TestGenericLegKinem(unittest.TestCase):

    def test_genericLegKinem(self):
        # Points 0,0, 2,2, 3,1
        g = TestGait()
        g.L12 = 1
        g.bodyH = 1.
        g.L45 = 0
        g.v12 = [g.L12, 0]
        g.L23 = 2*sqrt(2)
        g.sqL23 = 8
        g.L34 = sqrt(2.)
        g.sqL34 = 2.

        a2, a3, _ = g.genericLegKinem(footH=2, legLen=g.L12+3)

        self.assertAlmostEqual(pi/4, a2)
        self.assertAlmostEqual(-pi/2, a3)

    def test_genericLegKinem2(self):
        # Points 0,0, 1,1, 2,0
        g = TestGait()
        g.L12 = 1
        g.L45 = 0
        g.v12 = [g.L12, 0]
        g.bodyH = 1.
        g.L23 = sqrt(2.)
        g.sqL23 = 2.
        g.L34 = sqrt(2.)
        g.sqL34 = 2.

        a2, a3, _ = g.genericLegKinem(footH=1, legLen=g.L12+2)

        self.assertAlmostEqual(pi/4, a2)
        self.assertAlmostEqual(-pi/2, a3)

    def test_genericLegKinem3(self):
        # Points 0,0, 1,1, 2,1
        g = TestGait()
        g.L12 = 1
        g.L45 = 0
        g.v12 = [g.L12, 0]
        g.bodyH = 1.
        g.L23 = sqrt(2.)
        g.sqL23 = 2.
        g.L34 = 1.
        g.sqL34 = 1.

        a2, a3, _ = g.genericLegKinem(footH=2, legLen=g.L12+2)

        self.assertAlmostEqual(pi/4, a2)
        self.assertAlmostEqual(-pi/4, a3)

# TODO we don't handle the leg being below horizontal yet
    def test_genericLegKinem4(self):
        # Points 0,0, 1,-1, 1,-2
        g = TestGait()
        g.L12 = 1
        g.L45 = 0
        g.v12 = [g.L12, 0]
        g.bodyH = 2.
        g.L23 = sqrt(2.)
        g.sqL23 = 2.
        g.L34 = 1.
        g.sqL34 = 1.

        a2, a3, _ = g.genericLegKinem(footH=0, legLen=g.L12+1)
        print(g.v23, g.v34)

        self.assertAlmostEqual(-pi/4, a2)
        self.assertAlmostEqual(-pi/4, a3)

    def test_genericLegKinem5(self):
        # Points 0,0, 1,1, 1,-1
        g = TestGait()
        g.L12 = 1
        g.L45 = 0
        g.v12 = [g.L12, 0]
        g.bodyH = 1.
        g.L23 = sqrt(2.)
        g.sqL23 = 2.
        g.L34 = 2.
        g.sqL34 = 4.

        a2, a3, _ = g.genericLegKinem(footH=0, legLen=g.L12+1)
        print(g.v23, g.v34)

        self.assertAlmostEqual(pi/4, a2)
        self.assertAlmostEqual(-3*pi/4, a3)

    def test_genericLegKinem6(self):
        # Points 0,0, 1,0, 2,-1
        g = TestGait()
        g.L12 = 1
        g.L45 = 0
        g.v12 = [g.L12, 0]
        g.bodyH = 1.
        g.L23 = 1.
        g.sqL23 = 1.
        g.L34 = sqrt(2.)
        g.sqL34 = 2.

        a2, a3, _ = g.genericLegKinem(footH=0, legLen=g.L12+2)
        print(g.v23, g.v34)

        self.assertAlmostEqual(0, a2)
        self.assertAlmostEqual(-pi/4, a3)



class TestCalcFootH(unittest.TestCase):
    def test_calc_foot_h(self):
        foot_h_max = 1
        time_down_frac = 0.1
        half_loopLen = 50 # our `now` is between 0 and this
        trans_frac = 0.1
        h_frac = 0.2

        # zero; on ground for first half
        footh = calc_foot_h(0, foot_h_max, time_down_frac, half_loopLen, trans_frac, h_frac)
        self.assertEqual(footh, 0)
        # still on ground
        footh = calc_foot_h(half_loopLen, foot_h_max, time_down_frac, half_loopLen, trans_frac, h_frac)
        self.assertEqual(footh, 0)
        # going up slowly
        # going up quickly
        # steady at max height
        footh = calc_foot_h(1.5*half_loopLen, foot_h_max, time_down_frac, half_loopLen, trans_frac, h_frac)
        self.assertEqual(footh, foot_h_max)
        # going down quickly
        # going down slowly
        # idle on ground
        footh = calc_foot_h(2*half_loopLen, foot_h_max, time_down_frac, half_loopLen, trans_frac, h_frac)
        self.assertEqual(footh, 0)
        # again

class TestDoLegKinem(unittest.TestCase):
    def test_do_leg_kinem(self):
        #foot_h_max = 1
        #time_down_frac = 0.1
        #loopLen = 100 # our `now` is between 0 and this
        #trans_frac = 0.1
        #h_frac = 0.2

        # Points 0,0, 2,2, 3,1
        leg_geom, leg1, leg2, leg3, leg4 = gen_numa2_legs()
        g = Gaits(leg_geom, leg1, leg2, leg3, leg4)
        #g = TestGait()
        #g.L12 = 1
        #g.bodyH = 1.
        #g.L45 = 0
        #g.v12 = [g.L12, 0]
        #g.L23 = 2*sqrt(2)
        #g.sqL23 = 8
        #g.L34 = sqrt(2.)
        #g.sqL34 = 2.

        #n1, n2, n3, n4 = 1
        l1a2, l1a3, _ = g.doLegKinem(0, 0, footH=0)
        l2a2, l2a3, _ = g.doLegKinem(0, 0, footH=0)

        #self.assertAlmostEqual(0, g.footH13)
        #self.assertAlmostEqual(0, g.footH24)
        #print(leg1.s2_center, g.s12pos, leg1.s2_sign * leg_geom.aoffset2)
        #print(leg2.s2_center, g.s22pos, leg2.s2_sign * leg_geom.aoffset2)
        #print(leg1.s2_center, g.s12pos, leg1.s2_sign * leg_geom.aoffset2)
        #print(leg2.s2_center, g.s22pos, leg2.s2_sign * leg_geom.aoffset2)
        #self.assertAlmostEqual(l1a2 * RAD_TO_ANGLE, leg1.s2_sign * leg_geom.aoffset2)
        #self.assertAlmostEqual(l2a2 * RAD_TO_ANGLE, leg2.s2_sign * leg_geom.aoffset2)
        print(l1a2, l1a3)
        print(l2a2, l2a3)
        self.assertAlmostEqual(l1a2 * RAD_TO_ANGLE, leg1.s2_sign * leg_geom.aoffset2)
        self.assertAlmostEqual(l2a2 * RAD_TO_ANGLE, leg2.s2_sign * leg_geom.aoffset2)
        #self.assertAlmostEqual(leg1.s2_center, g.s12pos)
        #self.assertAlmostEqual(leg2.s2_center, g.s22pos)

class TestWalkCode(unittest.TestCase):
    def test_trav_calculation(self):
        foot_h_max = 1
        time_down_frac = 0.1
        loopLen = 100 # our `now` is between 0 and this
        trans_frac = 0.1
        h_frac = 0.2
        travRate = 10

        # Points 0,0, 2,2, 3,1
        g = TestGait()
        g.L12 = 1
        g.bodyH = 1.
        g.L45 = 0
        g.v12 = [g.L12, 0]
        g.L23 = 2*sqrt(2)
        g.sqL23 = 8
        g.L34 = sqrt(2.)
        g.sqL34 = 2.

        # numa.get_now(ms = 0)
        n1, n2, n3, n4 = loopLen/2, 0, loopLen/2, 0  # From plots
        a2, a3, _ = g.walk_code(loopLen, loopLen/2, travRate, 2*travRate, n1, n2, n3, n4)

        self.assertAlmostEqual(0, g.footH13)
        self.assertAlmostEqual(0, g.footH24)

#    def test_walk_code(self):
#        foot_h_max = 1
#        time_down_frac = 0.1
#        loopLen = 100 # our `now` is between 0 and this
#        trans_frac = 0.1
#        h_frac = 0.2
#
#        # Points 0,0, 2,2, 3,1
#        g = TestGait()
#        g.L12 = 1
#        g.bodyH = 1.
#        g.L45 = 0
#        g.v12 = [g.L12, 0]
#        g.L23 = 2*sqrt(2)
#        g.sqL23 = 8
#        g.L34 = sqrt(2.)
#        g.sqL34 = 2.
#
#        n1, n2, n3, n4 = 1
#        a2, a3, _ = g.walk_code(loopLen, loopLen/2, travRate, 2*travRate, n1, n2, n3, n4)
#
#        self.assertAlmostEqual(0, g.footH13)
#        self.assertAlmostEqual(0, g.footH24)
