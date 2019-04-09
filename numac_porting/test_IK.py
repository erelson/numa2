from math import sqrt, pi
from IK import Gaits
import unittest


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

