#  From: https://www.cs.hmc.edu/ACM/lectures/intersections.html
#
# intersections.py
#
# Python for finding line intersections
#   intended to be easily adaptable for line-segment intersections
#
from __future__ import print_function

import math

def intersectLines( pt1, pt2, ptA, ptB ):
    """This returns the intersection of Line(pt1,pt2) and Line(ptA,ptB)

       returns a tuple: (xi, yi, valid, r, s), where
       (xi, yi) is the intersection
       r is the scalar multiple such that (xi,yi) = pt1 + r*(pt2-pt1)
       s is the scalar multiple such that (xi,yi) = pt1 + s*(ptB-ptA)
           valid == 0 if there are 0 or inf. intersections (invalid)
           valid == 1 if it has a unique intersection ON the segment    """

    DET_TOLERANCE = 0.00000001

    # the first line is pt1 + r*(pt2-pt1)
    # in component form:
    x1, y1 = pt1;   x2, y2 = pt2
    dx1 = x2 - x1;  dy1 = y2 - y1

    # the second line is ptA + s*(ptB-ptA)
    x, y = ptA;   xB, yB = ptB;
    dx = xB - x;  dy = yB - y;

    # we need to find the (typically unique) values of r and s
    # that will satisfy
    #
    # (x1, y1) + r(dx1, dy1) = (x, y) + s(dx, dy)
    #
    # which is the same as
    #
    #    [ dx1  -dx ][ r ] = [ x-x1 ]
    #    [ dy1  -dy ][ s ] = [ y-y1 ]
    #
    # whose solution is
    #
    #    [ r ] = _1_  [  -dy   dx ] [ x-x1 ]
    #    [ s ] = DET  [ -dy1  dx1 ] [ y-y1 ]
    #
    # where DET = (-dx1 * dy + dy1 * dx)
    #
    # if DET is too small, they're parallel
    #
    DET = (-dx1 * dy + dy1 * dx)

    if math.fabs(DET) < DET_TOLERANCE: return (0,0,0,0,0)

    # now, the determinant should be OK
    DETinv = 1.0/DET

    # find the scalar amount along the "self" segment
    r = DETinv * (-dy  * (x-x1) +  dx * (y-y1))

    # find the scalar amount along the input line
    s = DETinv * (-dy1 * (x-x1) + dx1 * (y-y1))

    # return the average of the two descriptions
    xi = (x1 + r*dx1 + x + s*dx)/2.0
    yi = (y1 + r*dy1 + y + s*dy)/2.0
    return ( xi, yi, 1, r, s )


def testIntersection( pt1, pt2, ptA, ptB ):
    """ prints out a test for checking by hand... """
    print("Line segment #1 runs from", pt1, "to", pt2)
    print("Line segment #2 runs from", ptA, "to", ptB)

    result = intersectLines( pt1, pt2, ptA, ptB )
    print("    Intersection result =", result)
    print()


if __name__ == "__main__":
    pt1 = (10,10)
    pt2 = (20,20)

    pt3 = (10,20)
    pt4 = (20,10)

    pt5 = (40,20)

    testIntersection( pt1, pt2, pt3, pt4 )
    testIntersection( pt1, pt3, pt2, pt4 )
    testIntersection( pt1, pt2, pt4, pt5 )



