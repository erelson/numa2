def clamp(x, minimum, maximum):
    return max(minimum, min(x, maximum))

def speedPhaseFix(clocktime, loopLenOld, loopLen):
    # clocktime: us
    # loopLenOld, loopLen: ms
    # Returns ms
    # NOTE: Return value can be negative
    #print(clocktime, loopLenOld, loopLen)
    #time_ms = clocktime/1000
    return loopLen * \
            (((clocktime/1000) % loopLenOld) / loopLenOld -
        ((clocktime/1000) % loopLen) / loopLen)

#def interpolate(val, a,b, c,d):
#    pass # TODO
