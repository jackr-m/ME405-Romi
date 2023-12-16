"""Houses the Restructured LineFollower task class.
"""

from task_share import Share
from pyb import Pin
from PID import PID

class LineFollower:
    
    def __init__(self, s_pos: Share, leftTargetSpeed: Share, rightTargetSpeed: Share, leftDistanceTraveled: Share, rightDistanceTraveled: Share, heading: Share, s_tofdistance: Share, linePosTarget_front: int = 3000, linePosTarget_rear: int = 2000, centerSpeed: float = 2, time_scale: int = 20):
        """this is a line follower, it follows lines.
        
        Takes sensor data from 3 adjacent tasks: QTRs, IMU, and TOF
        Also takes distance traveled data from the now singular motor task and combines that with the IMU data to integrate position
        Commands motor task through requested speed shares

        Args:
            s_pos: Share object holding composite QTR position values
            leftTargetSpeed: Share object conveying desired left motor speed
            rightTargetSpeed: Share object conveying desired right motor speed
            leftDistanceTraveled: Share object conveying mm left wheel NET distance travled
            rightDistanceTraveled: Share object conveying mm right wheel NET distance traveled
            heading: Share object conveying IMU heading
            s_tofdistance: Share object conveying TOF distance for wall detection
            linePosTarget_front: integer QTR position target for front sensor
            linePosTarget_rear: integer QTR position target for rear sensor
            centerSpeed: float value that sets the desired base speed for the motors, determines how fast romi travels
            time_scale: int value describing ms/cycle for PID calculations
        """

        self._pos = s_pos

        self._leftSpeedTarget = leftTargetSpeed
        self._rightSpeedTarget = rightTargetSpeed

        self._linePosTarget_front = linePosTarget_front
        self._linePosTarget_rear = linePosTarget_rear
        
        self._centerSpeed = centerSpeed
        self._efforts = []
        self._motorSpeedOffset = 0
        self._state = 0
        
        self.time_scale = time_scale
        self._tofdistance = s_tofdistance

        self._circleArcLength = 24 * 25.4 + 20 # D*pi, 25.4 to convert to mm from in
        self._leftDistanceTraveled = leftDistanceTraveled
        self._rightDistanceTraveled = rightDistanceTraveled
        self._centerDistanceTraveled = 0
        self._prevRightDistanceTraveled = 0
        self._prevLeftDistanceTraveled = 0
        self._xCoord = 0
        self._yCoord = 0
        self._heading = heading

        self.user_button = Pin(Pin.cpu.C13, mode=Pin.IN)

        #self.kp = 1.15e-4
        #self.kp = 1.15e-4
        self.kp = 2.2e-4
        # self.ki = 1.05e-4
        #self.ki = 0
        #self.kd = self.kp/9.17
        self.kd = self.kp/9.2
        self.ki = 3.1e-6
        self._linePosPID = PID(self.kp, self.ki, self.kd, self._linePosTarget_front, sample_time=self.time_scale, scale="ms", proportional_on_measurement=False)
        #self._linePosPID.output_limits = (-0.90, 0.90)

    def run(self):
        while True:
            if self._state == 0:
                
                self._state = 1

            elif self._state == 1:
                
                # if self.user_button.value() == 0:
                #     self.kp *= 1.02 # increase by 2% every button press
                #     #self.kd = 0 * self.kp
                #     self._linePosPID.Kp = self.kp
                #     #self._linePosPID.Ki = self.ki

                

                #print(pos)
                #print("Distance : {} mm".format(self._tofdistance.get()))
                #print("Heading: {} deg".format(self._heading.get()))

                self._motorSpeedOffset = self._linePosPID(self._pos.get())

                self._leftSpeedTarget.put(self._centerSpeed+self._motorSpeedOffset)
                self._rightSpeedTarget.put(self._centerSpeed-self._motorSpeedOffset)

                # _rightWheelDelta = (self._rightDistanceTraveled.get() - self._prevRightDistanceTraveled)
                # _leftWheelDelta = (self._leftDistanceTraveled.get() - self._prevLeftDistanceTraveled)
                # _centerlineDelta = (_leftWheelDelta + _rightWheelDelta)/2

                # self._xCoord += _centerlineDelta * cos(self._heading.get())
                # self._yCoord -= _centerlineDelta * sin(self._heading.get())

                #Happens at end
                # self._prevRightDistanceTraveled = self._rightDistanceTraveled.get()
                # self._prevLeftDistanceTraveled = self._leftDistanceTraveled.get()

                if self._tofdistance.get() <= 100: # mm
                    self._leftSpeedTarget.put(0)
                    self._rightSpeedTarget.put(0)
                    print("DISTANCE MET")
                    self._state = 2

            elif self._state == 2:
                self._leftSpeedTarget.put(0)
                self._rightSpeedTarget.put(0)
               
            yield self._state