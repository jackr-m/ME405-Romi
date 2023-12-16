"""Houses RomiCoder class and test code.
"""
from pyb import Timer
from utime import ticks_ms, ticks_diff
from math import pi


class RomiCoder: 
    
    def __init__(self, encoderChannelAPin, encoderChannelBPin, AR, PS, encoderTimerNumber): 
        """Controls quadrature encoder.

        Each instance of an encoder takes two timers to operate.
        one timer is already determined by the pins used for the particular encoder.
        This timer is set up in the constructor. The second timer is used to set an interrupt.
        for the encoder to recalculate its positon.

        First Last
            January 1, 1970
            Interface with quadrature encoders
            Constructs an encoder object
            Updates encoder position and delta
            Gets the most recent encoder position
            Gets the most recent encoder delta
            Resets the encoder position to zero

        Args:
            encoderChannelAPin (pyb.Pin): Corresponds to channel 1 of the encoder timer.
            encoderChannelBPin (pyb.Pin): Corresponds to channel 2 of the encoder timer.
            AR (int): The auto reload value of the encoder timer.
            PS (int): The timer prescaler.
            encoderTimerNumber (int): The number of the timer that is to be used for the encoder (eg TIM1).
        """        
        
        self.halfAR = int((AR+1)/2)
        self.AR = AR
        self.encoderTimer = Timer(encoderTimerNumber, period = AR, prescaler = PS)
        
        #makes the channels for the encoder timer
        self.encoderTimer.channel(1, pin=encoderChannelAPin, mode=Timer.ENC_AB)
        self.encoderTimer.channel(2, pin=encoderChannelBPin, mode=Timer.ENC_AB)
        
        #self.updateTimer = updateTimer
        self.oldRaw = 0
        self.currentRaw = 0
        self.encoderPosition = float(0)
        self.rawDelta = 0
        self.trueDelta = 0

        self.oldTime = ticks_ms()
        self.newTime = ticks_ms()
        self.startTime = ticks_ms()
        self.timeDelta = float(0)
        self.rate = float(0)

        self.totalEncPos = 0

        # Scaling
        # was using 256 CPR, 16x gear ratio for old motor
        counts_per_revolution = 1440
        gear_ratio = 1 # ignore, using Pololu adjusted CPR
        self.position_scale = (2 * pi)/(counts_per_revolution * gear_ratio)
    

    def update(self): 
        """Updates true known encoder position

        This is different from the raw encoder position, since that value overflows according to the
        auto-reload value. takes difference between current and prior raw encoder 
        values, then determines based on the delta whether to count that as an overflow or not
        This means that the sample rate for calling this function needs to be
        sufficiently fast to avoid mis-classifying deltas as overflowed when they are not.
        This does not return any position

        Returns:
            None
        """
        
        # get the current time
        self.newTime = ticks_ms()

        # get delta of current time to old time
        self.timeDelta = ticks_diff(self.newTime, self.oldTime)


        # get the current raw value
        self.currentRaw = self.encoderTimer.counter()
                
        # take the delta from this raw value
        self.rawDelta = self.currentRaw - self.oldRaw
        
        # determine if the delta came from a step change or not
        if self.rawDelta > self.halfAR: 
            self.trueDelta = self.rawDelta - (self.AR+1)
        elif self.rawDelta < -self.halfAR:
            self.trueDelta = self.rawDelta + (self.AR+1)
        else:
            self.trueDelta = self.rawDelta

        # update the encoder positon variable    
        self.encoderPosition += self.trueDelta
    
        # get position in terms of radians
        self.encoderPosition *= self.position_scale

        self.totalEncPos += self.encoderPosition

        # regardless of true encoder position, raw values must be updated with regards to themselves
        self.oldRaw = self.currentRaw
        self.oldTime = self.newTime
        self.newTime -= self.startTime

        # calculate the rate
        if self.timeDelta != 0:
            self.rate = self.trueDelta/self.timeDelta
    
    def get_position(self): 
        """returns total encoder position

        the encoder will be updating at the rate specified by the
        frequency of the timer input when an encoder instance is created. This function only needs to be called when a position is desired.

        Returns:
            int: total encoder position
        """        
       
        return self.totalEncPos
        
    
    def get_delta(self): 
        """This returns a delta not a rate.

        Takes the encoder delta between the previous true encoder reading and the most recent one.

        Returns:
            int: True delta value
        """

        return self.trueDelta
    
    def get_time(self):
        """Returns the last update() time.

        Takes the time_ms of the most recent update() call.

        Returns:
            float: Time in ms.
        """
        
        
        '''!
            @brief This returns the last update() time.
            @details takes the time_ms of the most recent
            .update() call
            @return newTime float millisecond time
        '''

        return self.newTime

    def get_timeDelta(self):
        """Returns a delta of time.

        Takes the time_ms of the most recent .update() call, ticks_diff() from previous .update() call.

        Returns:
            float: Time delta in ms.
        """
        
        return self.timeDelta

    def get_rate(self):
        """Returns a rate per milliseconds.

        Takes the encoder delta between the previous true encoder reading and the most recent one, divides it by the difference between current time and previous time.

        Returns:
            float: Rate unit/ms value.
        """

        # divide by 1000 for ms to s conversion
        return  (self.rate)
        

    def zero(self): 
        """Sets the encoder position back to 0

        This does not reset the delta values. Raw readings stay intact

        Returns:
            None
        """
        
        self.encoderPosition = 0
        self.totalEncPos = 0
    

