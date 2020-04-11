"""
These are methods that help with evaluating gaits and leg designs, but are not used on the robot itself.

Frequently used in Juptyer notebooks. Generally these were developed in notebooks too, before being
generalized for several notebooks, etc.
"""
from math import cos, sin, atan2, copysign, pi
from numpy import array


def rotate_point(theta, point):
    return array([cos(theta)*point[0] - sin(theta)*point[1],
                  sin(theta)*point[0] + cos(theta)*point[1]])

def align_to_vector(ref_vec, vecs_to_align):
    """
    Parameters
    ----------
    ref_vec : listlike
        Vector representing a leg segment (joint to joint). Angle is calculated
        and used to rotate the vecs_to_align
    vecs_to_align : list of listlikes
        set of vectors representing edge(s) statically linked to the ref_vec,
        e.g. sides of the leg
    """
    try:
        theta = atan2(ref_vec[1], ref_vec[0]) if ref_vec[0] != 0.0 else copysign(ref_vec[1], pi/4)
    except Exception:
        print(ref_vec)
        raise
    return [rotate_point(theta, vec) for vec in vecs_to_align]


def gen_sides(legpts, sides, ptsets=None):#=[side23, side34]):
    if ptsets is None:
        ptsets = [[2,3],[3,4]] # pairs of indices in legpts to form ref_vecs from
        
    new_sides = []
    for pt, side in zip(ptsets, sides):
        new_sides.append(legpts[pt[0]] + align_to_vector(legpts[pt[1]] - legpts[pt[0]], side))
    return new_sides


def torque(pts, bodyH, masses=None, body_mass=None):
    torques = []
    if masses is None:
        masses = [115, 60, 20]
        
    if body_mass is None:
        body_mass = 2000/2 + sum(masses) # 2kg / 2 + a leg
    # on the ground
    if abs(pts[-1][1]) > 0.9 * bodyH:
        #pt_servo, pt_gnd = pts[0], pts[-1]
        #torque = pt_gnd[0] - pt_servo[0]
        for pt in pts[2:-2]:
            torques.append(body_mass * (pts[-1][0] - pt[0])) # expect mostly positive
    # off the ground
    else:
        # for each servo point of interest
        for cnt, pt in enumerate(pts[2:-2]):
            # Get the midpoint's distance of each segment multiplied by mass
            # For point  
            torques.append(0)
            for x in range(len(pts[cnt+1:-1])):
                midpt = pts[cnt+x][0] - pts[cnt+x+1][0]
                dist = pt[0] - midpt # expect mostly negative
                mass_dist = dist * 1# masses[cnt]
                torques[-1] += mass_dist
            #for ptb in pts:
            #    pt[0] - ptb
            
    
    #torque = pt_gnd[0] - pt_servo[0]
    #pass
    return torques

# DON"T USE YET; needs a class implementation probably
#def mkpts(gait, bodyH): # calcualte the points I want to plot; points correspond to the servo joints
#    pt1 = array([L0, 0])
#    pt2 = pt1 + array([L12,0])
#    pt3 = pt2 + gait.v23 # we do this negative in the code, too
#    pt4 = pt3 + [gait.v34[0], -gait.v34[1]]
#    pt5 = pt4 + [0, -L45] #[10,0]#
#    pt0 = pt1 - [0, bodyH]
#
#    pts = array([pt0, pt1, pt2, pt3, pt4, pt5])
#    
#    #sides = []
#    #sides.append(array([])
#    return pts#, sides


def snapshot(gait, ms, loopLength=10000, ang_dir=0.0):
    """Get leg position at a single time within the loop length"""
    
    travRate = 25 # distance in mm that feet travel in a straight line back and forth

    half_loopLength = loopLength/2.0

    # TODO move this
    #ms = 0 #0
    turnTimeOffset = 0 # probalby don't need this for now except as placeholder
    now2 = (ms - turnTimeOffset) % loopLength
    now3 = (ms - turnTimeOffset +  half_loopLength) % loopLength
    now4 = loopLength - (ms - turnTimeOffset) % loopLength
    now1 = loopLength - (ms - turnTimeOffset + half_loopLength) % loopLength

    gait.initTrig()
    gait.walkCode(loopLength, half_loopLength, travRate, 2*travRate,
                  now1, now2, now3, now4, ang_dir)
