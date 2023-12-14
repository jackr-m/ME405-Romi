"""Houses the Control task class.

"""

from RomiMotor import RomiMotor
from pyb import Timer, Pin
from task_share import Share
from ClosedLoop import ClosedLoop
from PID import PID

class Control:
    """Controls a DC romi motor using a PID loop
    
    encoder velocity and the time delta since the task last ran is provided as task_share.Share objects. PID gains are set when an instance of the class is created. Motor duty cycle is set according to the PID efforts.
    """

    def __init__(self, motor, s_speed: Share, s_kp: Share, s_ki: Share, s_kd: Share, feedbackOn: Share, s_timeDelta: Share, s_velocity: Share):
        '''!@brief          Initializes a data transfer task object
            @param motor    A motor driver object to send commands to
        '''
        self._desiredSpeed = s_speed
        self._kp = s_kp
        self._ki = s_ki
        self._kd = s_kd
        self._efforts = []
        self._feedbackOn = feedbackOn
        self._timeDeltas = s_timeDelta
        self._velocities = s_velocity

        self._motorLetter = motor


        # Motor object
        if motor == "A":
            tim1 = Timer(1, freq=20_000)
            self._motor = RomiMotor(tim1, Pin(Pin.cpu.B4, mode=Pin.OUT_PP), Pin(Pin.cpu.A8, mode=Pin.ALT, alt=Pin.AF1_TIM1), 1, Pin(Pin.cpu.B10, mode=Pin.OUT_PP))
        elif motor == "B":
            tim3 = Timer(3, freq=20_000)
            self._motor = RomiMotor(tim3, Pin(Pin.cpu.B3, mode=Pin.OUT_PP), Pin(Pin.cpu.C8, mode=Pin.ALT, alt=Pin.AF2_TIM3), 3, Pin(Pin.cpu.B5, mode=Pin.OUT_PP))
        else:
            raise ValueError("Invalid motor")
        
        self._closedLoop = ClosedLoop(self._kp.get(), self._ki.get(), self._kd.get(), -((2^32)-1), ((2^32)-1))
        

        # Task state
        self._state = 0
    
    def run(self):
        '''!@brief          The task implementation as a generator function
        '''

        while True:

            #if we are currently in a feedback state
            if self._state == 0:
                
                self._motor.set_duty(0)
                self._motor.enable()
            
                self._closedLoop.setKp(self._kp.get())
                self._closedLoop.setKi(self._ki.get())
                self._closedLoop.setKd(self._kd.get())

                self._state = 1
            
            elif self._state == 1:

                self._closedLoop.setTarget(self._desiredSpeed.get())

                self._efforts = self._closedLoop.calculateEfforts(self._velocities.get(), self._timeDeltas.get())

                self._motor.set_duty(int(round(sum(self._efforts), 0)))
            
            else:
                raise ValueError("Invalid state")


            yield self._state
