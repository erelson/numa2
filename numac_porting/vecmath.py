from math import sqrt, atan, atan2
#Example: 
#        vector2d_AngleRadians(v23, v12vert)

def norm(a)
    alen = sqrt(a[0] * a[0] + a[1] * a[1])
    anorm = [_ / alen for _ in a]
    return anorm

def vector2d_AngleRadians(a, b):
    # Expects two lists
    anorm = norm(a)
    bnorm = norm(b)

    atan2(b[0] - a[0]
    


