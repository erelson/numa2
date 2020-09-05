from math import pi
from IK import v2d_AngleRadians
#from vecmath import *
import unittest

class TestVecMath(unittest.TestCase):
    def test_45(self):
        v1 = [1,0]
        v2 = [1,1]

        expect = pi / 4

        res = v2d_AngleRadians(v1, v2)
        self.assertAlmostEqual(expect, res)

    def test_135(self):
        v1 = [1,0]
        v2 = [-1,1]

        expect = 135 * pi / 180

        print(45*pi/180)
        res = v2d_AngleRadians(v1, v2)
        self.assertAlmostEqual(expect, res)

    def test_neg45(self):
        v1 = [1,0]
        v2 = [1,-1]

        expect = -pi / 4

        res = v2d_AngleRadians(v1, v2)
        self.assertAlmostEqual(expect, res)

    def test_neg135(self):
        v1 = [0,1]
        v2 = [1,-1]

        expect = -3/4 * pi

        res = v2d_AngleRadians(v1, v2)
        self.assertAlmostEqual(expect, res)

