"""Houses the UpdateBothMotors class.
"""

from pyb import Timer, Pin
from task_share import Share, Queue
from RomiCoder import RomiCoder
from RomiMotor import RomiMotor
from utime import sleep_ms
import QTRSensors
from ClosedLoop import ClosedLoop

class controlBothMotors:
    
    def __init__(self, kp, ki, kd, s_leftRequestedSpeed, s_rightRequestedSpeed, s_leftDistanceTraveled, s_rightDistanceTraveled):
        """Controls both motors in closed loop, taking data straight from their encoders.

        Updates left and right NET distance traveled shares so LineFollower can turn them into an actual XY position
        
        Args:
            kp: kp (no longer a share)
            ki: ki (no longer a share)
            kd: kd (no longer a share)
            s_leftRequestedSpeed: share containing requested speed for left motor
            s_rightRequestedSpeed: share containing requested speed for right motor
            s_leftDistanceTraveled: share containing net left distance traveled
            s_rightDistanceTraveled: share containing net right distance traveled
        """

        self._kp = kp
        self._ki = ki
        self._kd = kd

        self._leftRequestedSpeed = s_leftRequestedSpeed
        self._rightRequestedSpeed = s_rightRequestedSpeed

        #init state
        self._state = 0

        self._leftEncoder = RomiCoder(Pin.cpu.B6, Pin.cpu.B7, 65535, 0, 4)
        tim1 = Timer(1, freq=20_000)
        self._leftMotor = RomiMotor(tim1, Pin(Pin.cpu.B4, mode=Pin.OUT_PP), Pin(Pin.cpu.A8, mode=Pin.ALT, alt=Pin.AF1_TIM1), 1, Pin(Pin.cpu.B10, mode=Pin.OUT_PP))

        self._rightEncoder = RomiCoder(Pin.cpu.C6, Pin.cpu.C7, 65535, 0, 8)
        tim3 = Timer(3, freq=20_000)
        self._rightMotor = RomiMotor(tim3, Pin(Pin.cpu.B3, mode=Pin.OUT_PP), Pin(Pin.cpu.C8, mode=Pin.ALT, alt=Pin.AF2_TIM3), 3, Pin(Pin.cpu.B5, mode=Pin.OUT_PP))

        self._leftPID = ClosedLoop(self._kp, self._ki, self._kd, -((2^32)-1), ((2^32)-1))
        self._rightPID = ClosedLoop(self._kp, self._ki, self._kd, -((2^32)-1), ((2^32)-1))

        self._leftActualSpeed = 0
        self._rightActualSpeed = 0

        self._leftDistanceTraveled = s_leftDistanceTraveled
        self._rightDistanceTraveled = s_rightDistanceTraveled

        self._leftTimeDelta = 0
        self._rightTimeDelta = 0

        self._leftEfforts = 0
        self._rightEffors = 0

    def run(self):
        
        while True:
            #immediately go to run state after qtr initialization
            if self._state == 0:

                self._leftEncoder.zero()
                self._rightEncoder.zero()

                self._leftMotor.set_duty(0)
                self._rightMotor.set_duty(0)
                
                self._leftMotor.enable()
                self._rightMotor.enable()

                self._state = 1

            elif self._state == 1:

                self._leftEncoder.update()
                self._rightEncoder.update()

                self._leftActualSpeed = self._leftEncoder.get_rate()
                self._rightActualSpeed = self._rightEncoder.get_rate()

                self._leftDistanceTraveled.put(self._leftEncoder.get_position() * 69.5/2) #  wheel radius is 35mm nominal
                self._rightDistanceTraveled.put(self._rightEncoder.get_positon() * 69.5/2)

                self._leftTimeDelta = self._leftEncoder.get_timeDelta()
                self._rightTimeDelta = self._rightEncoder.get_timeDelta()

                self._leftPID.setTarget(self._leftRequestedSpeed.get())
                self._rightPID.setTarget(self._rightRequestedSpeed.get())

                self._leftEfforts = self._leftPID.calculateEfforts(self._leftActualSpeed, self._leftTimeDelta)
                self._rightEffors = self._rightPID.calculateEfforts(self._rightActualSpeed, self._rightTimeDelta)

                self._leftMotor.set_duty(int(round(sum(self._leftEfforts), 0)))
                self._rightMotor.set_duty(int(round(sum(self._rightEffors), 0)))

            #always will be in state 1
            #gc.collect()
            yield self._state
