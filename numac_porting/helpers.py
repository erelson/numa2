def clamp(x, minimum, maximum):
    return max(minimum, min(x, maximum))

#def speedPhaseFix(clocktime, loopLenOld, loopLen):
def speedPhaseFix(current_offset, clocktime, loopLenOld, loopLenNew):
    # clocktime: ms
    # loopLenOld, loopLen: ms
    # Returns ms
    # NOTE: Return value can be negative
    #print(clocktime, loopLenOld, loopLen)
    #time_ms = clocktime/1000
    # OLD
    #return loopLen * \
    #        (((clocktime/1000) % loopLenOld) / loopLenOld -
    #    ((clocktime/1000) % loopLen) / loopLen)

    return loopLenNew / loopLenOld * ((clocktime + current_offset) % loopLenOld) - clocktime % loopLenNew

#def interpolate(val, a,b, c,d):
#    pass # TODO
