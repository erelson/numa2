from pyb import ADC, Pin, Timer


#PWM_PINS = {"Y9": [2, 4],
#            "X5": [2, 1],
PWM_PINS = {Pin.board.Y10.name(): [2, 4],
            Pin.board.X6.name(): [2, 1],
            }


class PWM:

    def __init__(self, pin_name):
        tim_num = PWM_PINS[pin_name.name()][0]
        ch_num = PWM_PINS[pin_name.name()][1]
        self.p = Pin(pin_name)
        self.tim = Timer(tim_num)
        self.tim.init(freq=10000) # need 5k+ for VNH5019 current sense to work
        # TODO is this right?
        self.ch = self.tim.channel(ch_num, mode=Timer.PWM, pin=self.p,
                pulse_width_percent=0) # especially this

    def duty_cycle(self, dc):
        # dc is duty cycle from 0 to 100
        self.ch.pulse_width_percent(dc)


class MotorDriver:

    def __init__(self, directionPinA, directionPinB, pwmPin, encoderNumber=None, cs=None, max_speed=127):
        """

        Parameters
        ----------
        directionPinA : Pin name, pyb.Pin, e.g. pyb.Pin.board.Y7
        directionPinB : pyb.Pin, e.g. pyb.Pin.board.Y7
        pwmPin : pyb.Pin, e.g. pyb.Pin.board.Y7
        encoderNumber :
        cs : pyb.Pin, e.g. pyb.Pin.board.Y7
        max_speed : int
        """
        self.directionPinA = Pin(directionPinA, Pin.OUT)
        self.directionPinB = Pin(directionPinB, Pin.OUT)
        self.pwm = PWM(pwmPin)
        self.directionFactor = 1
        #self.directSetSpeed(0)
        #self.set_speed(0)
        #self.encoder = pyb.Encoder(encoderNumber)
        self.encoderLastCount = 0
        self.desiredSpeed = 0
        self.cs = cs
        self.cs_adc = ADC(self.cs)
        self.max_speed = max_speed

        self.direct_set_speed(0)

    def run_reversed(self):
        self.directionFactor = -1

    #def set_speed(self, newDesiredSpeed):
    #    # Uses PID
    #    # Value from -127 to 127
    #    #self.pid.setSetpointValue(newDesiredSpeed)
    #    self.desiredSpeed = newDesiredSpeed

    def direct_set_speed(self, speed):
        """
        speed : ???
        """
        # TODO clamp speed to max_speed
        # Value from -127 to 127
        actualSpeed = speed * self.directionFactor
        # Forward
        if actualSpeed > 0:
            print("A!")
            self.directionPinA.value(1)
            self.directionPinB.value(0)
            self.pwm.duty_cycle(actualSpeed/self.max_speed * 100)
        # Off
        elif actualSpeed == 0:
            self.directionPinA.value(0)
            self.directionPinB.value(0)
            self.pwm.duty_cycle(0)
        # Backward
        else:
            self.directionPinA.value(0)
            self.directionPinB.value(1)
            # TODO why would negative value have been passed?
            #self.pwm.duty_cycle(-actualSpeed)
            self.pwm.duty_cycle(-actualSpeed/self.max_speed * 100)
        print("actualSpeed:", actualSpeed, "max:", self.max_speed)

        # New compare threshold (?) TODO old from wixel code
        #ticks_on = 0
        
        #if actualSpeed > 0:
        #    #ticks_on = interpolateU(speed, 0, DRIVE_SPEED_MAX, 0, top);

        #    # Set direction1 high, direction2 low
        #    self.directionPinA.value(1)
        #    self.directionPinB.value(0)
        #elif actualSpeed < 0:
        #    #ticks_on = interpolateU(speed, 0, DRIVE_SPEED_MIN, 0, top)

        #    # Set direction1 low, direction2 high low
        #    self.directionPinA.value(0)
        #    self.directionPinB.value(1)
        #else:
        #    # brake
        #    #ticks_on = 0
        #    self.directionPinA.value(0)
        #    self.directionPinB.value(0)

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
