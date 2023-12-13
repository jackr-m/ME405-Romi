
'''!@file
    @brief Encoder class with test functionality
    @details contains encoder class and test code that runs when file is named main
    @author Casey Pickett and Jack Miller
    @date 10/16/23
'''

from pyb import Timer
from utime import ticks_ms, ticks_diff
from math import pi


class RomiCoder: 
    '''!
        @brief encoder.py
        @details class for quadrature encoders. Each instance of an encoder takes two timers to operate
        one timer is already determined by the pins used for the particular encoder.
        This timer is set up in the constructor. The second timer is used to set an interrupt
        for the encoder to recalculate its positon.
    '''
    def __init__(self, encoderChannelAPin, encoderChannelBPin, AR, PS, encoderTimerNumber): 
        '''!
            @brief encoder.py
            A driver for reading from Quadrature Encoders
            First Last
            January 1, 1970
            Interface with quadrature encoders
            Constructs an encoder object
            Updates encoder position and delta
            Gets the most recent encoder position
            Gets the most recent encoder delta
            Resets the encoder position to zero
                    @details The timer used to drive the encoder itsself will be set up
                    in the constructor. Only pins and rate parameters will need to be 
                    input. The second timer, being the one that prompts the instance 
                    of the encoder to update its position, is input by the user.
                    @param encoderChannelAPin (pyb.Pin) is the pin that corresponds to channel 1 of the
                    encoder timer
                    @param encoderChannelBPin (pyb.Pin) is the the pin that corresponds to channel 2 of the
                    encoder timer
                    @param AR is the auto reload value of the encoder timer.
                    @param PS is the timer prescaler. 
                    @encoderTimerNumber is the number of the timer that is to 
                    be used for the encoder (eg TIM1)
        '''
        
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
        counts_per_revolution = 1440
        gear_ratio = 1 # ignore, using Pololu adjusted CPR
        self.position_scale = (2 * pi)/(counts_per_revolution * gear_ratio)
    
    
    
    def update(self): 
        '''!
            @brief updates true known encoder positon.
            @details This is different from the raw encoder position, since that value overflows according to the
            auto-reload value. takes difference between current and prior raw encoder 
            values, then determines based on the delta whether to count that as an overflow or not
            This means that the sample rate for calling this function needs to be
            sufficiently fast to avoid mis-classifying deltas as overflowed when they are not.
            This does not return any position
        '''
        
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
    
        # get position in terms of revolutions
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
        '''!
            @brief returns most recently updated true encoder position
            @details the encoder will be updating at the rate specified by the
            frequency of the timer input when an encoder instance is created.
            @return encoderPosition integer position value
        '''
        return self.totalEncPos
        
    
    def get_delta(self): 
        '''!
            @brief This returns a delta not a rate.
            @details takes the encoder delta between the previous true 
            encoder reading and the most recent one
            @return trueDelta integer delta value
        '''

        return self.trueDelta
    
    def get_time(self):
        '''!
            @brief This returns the last update() time.
            @details takes the time_ms of the most recent
            .update() call
            @return newTime float millisecond time
        '''

        return self.newTime

    def get_timeDelta(self):
        '''!
            @brief This returns a delta of time.
            @details takes the time_ms of the most recent
            .update() call, ticks_diff() from previous .update() call
            @return timeDelta float millisecond time
        '''
        return self.timeDelta

    def get_rate(self):
        '''!
            @brief This returns a rate per milliseconds.
            @details takes the encoder delta between the previous true 
            encoder reading and the most recent one, divides it by
            the difference between current time and previous time
            @return rate float unit/ms value
        '''

        # divide by 1000 for ms to s conversion
        return  (self.rate)
        

    def zero(self): 
        '''!
            @brief This sets the encoder position back to 0
            @details this does not reset the delta values. Raw readings stay intact
        '''
        self.encoderPosition = 0
        self.totalEncPos = 0
    

