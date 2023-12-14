from math import isnan

'''!@file
    @brief Closed Loop control algorithm
    @details Implements a PID loop
    @author Casey Pickett and Jack Miller
    @date 10/17/23
'''

class ClosedLoop: 
    '''!
        @brief ClosedLoop.py
        @details class for PID control.
    '''

    def __init__ (self, Kp, Ki, Kd, lowerSatLimit, upperSatLimit):
        '''!
            @brief 
            @details 
            @param lowerSatLimit lower saturation limit for I path
            @param upperSatLimit upper saturation limit for I path
            @param Kp Proportional gain
            @param Ki Integral gain
            @param Kd Derivative gain
        '''

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.upperSatLimit = upperSatLimit
        self.lowerSatLimit = lowerSatLimit
        
        self.currentError = 0
        self.previousError = 0
        self.integratedError = 0
        self.rateOfError = 0
        self.samplePeriod = 0
        self.P_effort = 0
        self.I_effort = 0
        self.D_effort = 0
        self.target = 0
        self.hasRan = False

    def setTarget(self, target):
        '''!
            @brief change target value
            @details does not change I path.
        '''
        self.target = target
        
    def clearIntegral(self):
        '''!
            @brief sets integrated error to 0 but does not turn I path off
            @details to disable the I path, use SetKi to change the I gain to 0
        '''
        self.integratedError = 0
   
    def calculateEfforts(self, currentValue, msTimePassed):
        '''!
            @brief calculates P, I, D efforts
            @details saturated according to set saturation bounds. Use this in an interrupt
            routiene that is called at the same period according to samplePeriod
            @param currentValue is the current value of the sensor element used to control the loop.
            This should have the same units as the target value.
            @param msTimePassed is the time passed in milliseconds since the last sample.
            @return [self.P_effort, self.I_effort, self.D_effort] (list) of all efforts.
            The effort itself is not saturated.
        '''

        if msTimePassed != 0:
            #P path
            #positive error means self is behind the target
            self.currentError = self.target - currentValue
            self.P_effort = self.currentError * self.Kp
            
            #I path
            self.integratedError += self.currentError * msTimePassed/1000
            
            #saturate I path
            if self.integratedError > self.upperSatLimit:
                self.integratedError = self.upperSatLimit
            
            elif self.integratedError < self.lowerSatLimit:
                self.integratedError = self.lowerSatLimit
            
            
            #effort based on saturated I path
            self.I_effort = self.integratedError * self.Ki
            
            #D path
            if self.hasRan:
                self.rateOfError = (self.currentError - self.previousError)/msTimePassed * 1000
                self.D_effort = self.rateOfError * self.Kd
                #reset for next D calculation
            else:
                self.hasRan = True

            self.previousError = self.currentError
        
        #could change this to return a different type, but this case should only happen at the start of the control loop's use
        else:
            self.P_effort = 0
            self.I_effort = 0 
            self.D_effort = 0

        #list of effort values
        return([self.P_effort, self.I_effort, self.D_effort])

    
    def getError(self):
        '''!
            @brief returns error
            @return [self.currentError, self.integratedError, self.rateOfError] list of errors according to path
        '''
        return([self.currentError, self.integratedError, self.rateOfError])
    
    def setKp(self, Kp):
        self.Kp = Kp
        
    def setKi(self, Ki):
        self.Ki = Ki
    
    def setKd(self, Kd):
        self.Kd = Kd

    def getTarget(self):
        return self.target

    def setSatLower(self, lowerSatLimit):
        self.lowerSatLimit = lowerSatLimit
        
    def setSatUpper(self, upperSatLimit):
        self.upperSatLimit = upperSatLimit
        
        
  