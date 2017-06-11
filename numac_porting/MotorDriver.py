import pyb

class MotorDriver:

    def __init__(self, directionPinNameA, directionPinNameB, pwmNumber, encoderNumber=None, cs=None):
        self.directionPinA = directionPinNameA
        self.directionPinB = directionPinNameB
        self.pwm = pyb.Pwm(pwmNumber)
        self.directionFactor = 1
        self.directSetSpeed(0)
        #self.encoder = pyb.Encoder(encoderNumber)
        #self.pid = PIDController(0.25, 0.75, 0.01)
        self.pid.enable()
        self.encoderLastCount = 0
        self.desiredSpeed = 0
        self.cs = cs

    def run_reversed(self):
        self.directionFactor = -1

    def set_speed(self, newDesiredSpeed):
        # Uses PID
        # Value from -127 to 127
        #self.pid.setSetpointValue(newDesiredSpeed)
        self.desiredSpeed = newDesiredSpeed

    def direct_set_speed(self, speed):
        # Value from -127 to 127
        actualSpeed = speed * self.directionFactor
        if actualSpeed < 0:
            pyb.gpio(self.directionPin, 1)
            self.pwm.duty_cycle(-actualSpeed)
        else:
            pyb.gpio(self.directionPin, 0)
            self.pwm.duty_cycle(actualSpeed)

        # New compare threshold (?) TODO old from wixel code
        #ticks_on = 0
        
        if actualSpeed > 0:
            #ticks_on = interpolateU(speed, 0, DRIVE_SPEED_MAX, 0, top);

            # Set direction1 high, direction2 low
            pyb.gpio(self.directionPinA, 1)
            pyb.gpio(self.directionPinB, 0)
        elif actualSpeed < 0:
            #ticks_on = interpolateU(speed, 0, DRIVE_SPEED_MIN, 0, top)

            # Set direction1 low, direction2 high low
            pyb.gpio(self.directionPinA, 0)
            pyb.gpio(self.directionPinB, 1)
        else:
            # brake
            #ticks_on = 0
            pyb.gpio(self.directionPinA, 0)
            pyb.gpio(self.directionPinB, 0)

    #def get_count(self):
    #    return self.encoder.count()

    #def update(self):
        #currentCount = self.encoder.count()
        #deltaCount = currentCount - self.encoderLastCount
        #pidInput = deltaCount * ENCODER_TO_PID_RATIO
        #self.pid.setInput(pidInput)
        #if self.pid.compute():
#       #     print("PID Output: ", self.pid.getOutputValue(), " Input: ", pidInput, " Setpoint: ", self.desiredSpeed)
        #    self.directSetSpeed(self.pid.getOutputValue())
        #    self.encoderLastCount = currentCount
