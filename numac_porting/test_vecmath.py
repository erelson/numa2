from math import pi
from vecmath import *

class TestVecMath(Unittest.Testcase):
    def test_45(self):
        v1 = [1,0]
        v2 = [1,1]

        expect = 45 * pi / 180

        res = vector2d_AngleRadians(v1, v2)
        assert_equal(expect, res)

    def test_135(self):
        v1 = [0,1]
        v2 = [1,-1]

        expect = 135 * pi / 180

        res = vector2d_AngleRadians(v1, v2)
        assert_equal(expect, res)

