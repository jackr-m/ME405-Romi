"""Houses ClosedLoop Module"""

from math import isnan

class ClosedLoop: 
    
    def __init__ (self, Kp, Ki, Kd, lowerSatLimit, upperSatLimit):
        """PID Controller with changeable setpoint and gains.
        
        Does not need to run at a set frequency, adjust gains based on user-input time deltas.

        Args:
            Kp (float): value of proportional gain
            Ki (float): value of integral gain
            Kd (float): value of derivative gain
            lowerSatLimit (any): Int or Long, lower saturation limit for I path.
            upperSatLimit (any): Int or Long, upper saturation limit for I path.
        """

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
        '''Changes target value.

            Does not change I path.
        '''
        self.target = target
        
    def clearIntegral(self):
        '''Sets integrated error to 0 but does not turn I path off
            
        To disable the I path, use SetKi to change the I gain to 0
        '''
        self.integratedError = 0
   
    def calculateEfforts(self, currentValue, msTimePassed):
        """Calculates P, I, D efforts

        Saturated according to set saturation bounds. Use this in an interrupt routiene that is called at the same period according to samplePeriod.
        
        Args:
            currentValue (float): Current "actual" value used to calculate error.
            msTimePassed (float): Time passed in ms since function last called. Used to calculate I and D efforts.

        Returns:
            list of float: [P effort, I effort, D effort]. The effort itself is not saturated.
        """
        
        
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
        """Returns list of errors used to caluclate P, I, and D efforts.

        Returns:
            list of floats: [currentError, integratedError, rateOfError] list of errors according to each path.
        """
        return([self.currentError, self.integratedError, self.rateOfError])
    
    def setKp(self, Kp):
        """Sets P gain

        Args:
            Kp (float): P gain
        """        

        self.Kp = Kp
        
    def setKi(self, Ki):
        """Sets I gain. 
        
        Does not affect integrated error.

        Args:
            Ki (float): I gain.
        """        

        self.Ki = Ki
    
    def setKd(self, Kd):
        """Sets D gain.

        Args:
            Kd (float): D gain.
        """        

        self.Kd = Kd

    def getTarget(self):
        """Returns target value in-use by PID

        Returns:
            float: Target value
        """        

        return self.target

    def setSatLower(self, lowerSatLimit):
        """Sets the lower saturation limit

        Args:
            lowerSatLimit (float): lower saturation limit for integral error. Not a limit on the magnitude of the I effort.
        """

        self.lowerSatLimit = lowerSatLimit
        
    def setSatUpper(self, upperSatLimit):
        """Sets the upper saturation limit

        Args:
            upperSatLimit (float): upper saturation limit for integral error. Not a limit on the magnitude of the I effort.
        """

        self.upperSatLimit = upperSatLimit
        
        
  