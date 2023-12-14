from task_share import Share
from pyb import Pin
from PID import PID

class LineFollower:
    """this is a line follower, it follows lines"""
    def __init__(self, qtrPos_front: Share, qtrPos_rear: Share, e1_targetSpeed: Share, e2_targetSpeed: Share, s_timeDelta: Share, e1FeedbackOn: Share, e2FeedbackOn: Share,  leftDistanceTraveled: Share, rightDistanceTraveled: Share, heading: Share,  s_tofdistance: Share, motor_1_object, motor_2_object, linePosTarget_front: int = 3000, linePosTarget_rear: int = 2000, centerSpeed: float = 2, time_scale: int = 20):
        self._qtrPos_front = qtrPos_front
        self._qtrPos_rear = qtrPos_rear
        self._e1SpeedTarget = e1_targetSpeed
        self._e2SpeedTarget = e2_targetSpeed
        self._linePosTarget_front = linePosTarget_front
        self._linePosTarget_rear = linePosTarget_rear
        self._timeDelta = s_timeDelta
        self._centerSpeed = centerSpeed
        self._efforts = []
        self._motorSpeedOffset = 0
        self._state = 0
        self._e1FeedbackOn = e1FeedbackOn
        self._e2FeedbackOn = e2FeedbackOn
        self.time_scale = time_scale
        self._tofdistance = s_tofdistance
        self._motor1 = motor_1_object
        self._motor2 = motor_2_object

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
                self._e1FeedbackOn.put(1)
                self._e2FeedbackOn.put(1)
                self._state = 1

            elif self._state == 1:
                
                # if self.user_button.value() == 0:
                #     self.kp *= 1.02 # increase by 2% every button press
                #     #self.kd = 0 * self.kp
                #     self._linePosPID.Kp = self.kp
                #     #self._linePosPID.Ki = self.ki

                # Scale rear position to have same full scale as front sensor array
                rearPos = (self._qtrPos_rear.get()) * (self._linePosTarget_front/self._linePosTarget_rear)
                frontPos = self._qtrPos_front.get()

                print(rearPos)

                if rearPos==0 and frontPos==0:
                    pos = 0
                elif (rearPos<2500 or 3500<rearPos) and 2500<frontPos<3500:
                    pos = frontPos
                elif 2500<rearPos<3500 and (frontPos<2500 or 3500<frontPos):
                    pos = rearPos
                elif rearPos==6000 and frontPos==6000:
                    pos = 6000
                else:
                    #pos = (frontPos + rearPos)/2
                    pos = frontPos

                # dead zone
                if 2600<pos<3400:
                    pos = 3000

                #print(pos)
                #print("Distance : {} mm".format(self._tofdistance.get()))
                #print("Heading: {} deg".format(self._heading.get()))

                self._motorSpeedOffset = self._linePosPID(pos)


                self._e1SpeedTarget.put(self._centerSpeed+self._motorSpeedOffset)
                self._e2SpeedTarget.put(self._centerSpeed-self._motorSpeedOffset)

                # _rightWheelDelta = (self._rightDistanceTraveled.get() - self._prevRightDistanceTraveled)
                # _leftWheelDelta = (self._leftDistanceTraveled.get() - self._prevLeftDistanceTraveled)
                # _centerlineDelta = (_leftWheelDelta + _rightWheelDelta)/2

                # self._xCoord += _centerlineDelta * cos(self._heading.get())
                # self._yCoord -= _centerlineDelta * sin(self._heading.get())

                #Happens at end
                # self._prevRightDistanceTraveled = self._rightDistanceTraveled.get()
                # self._prevLeftDistanceTraveled = self._leftDistanceTraveled.get()

                if self._tofdistance.get() <= 100: # mm
                   self._state = 2

            elif self._state == 2:
                self._e1SpeedTarget.put(0)
                self._e2SpeedTarget.put(0)
                self._motor1._motor.set_duty(0)
                self._motor2._motor.set_duty(0)
                print("DISTANCE MET")
                self._state = 3
            
            elif self._state == 3:
                self._e1SpeedTarget.put(0)
                self._e2SpeedTarget.put(0)
                self._motor1._motor.set_duty(0)
                self._motor2._motor.set_duty(0)
            
            yield self._state