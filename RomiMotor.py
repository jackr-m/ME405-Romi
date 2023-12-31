"""Houses RomiMotor class

Adopted from ME405 Lab 0x04
    Jack Miller and Casey Pickett
    7 November 2023

    TI DRV8838 motor driver

"""

from pyb import Timer
from pyb import Pin

import utime as time


class RomiMotor:

    def __init__(self, PWM_tim, SLP_pin, PH_pin, PH_channel, DIR_pin):
        """A driver class for one channel of the L6206.
        
        Objects of this class can e used to apply PWM to a given DC motor on one channel of the L6206 from ST Microelectronics

        Args:
            PWM_tim (pyb.Timer): PWM timer for motor.
            SLP_pin (pyb.Pin): SLP pin acts as an enable/disable pin.
            PH_pin (pyb.Pin): PH pin corresponds to PWM signal.
            PH_channel (int): PH channel number for PWM signal.
            DIR_pin (pyb.Pin): Direction Pin.
        """
            

        self.PWM_tim = PWM_tim
        self.SLP_pin = SLP_pin
        self.PH_pin = PH_pin
        self.PH_channel = PH_channel
        self.DIR_pin = DIR_pin

    def set_duty(self, duty):
        """Set the PWM duty cycle for the DC motor.

        This method sets the duty cycle to be sent to the L6206 to a given level. Positive values cause effort in one direction, negative values in the opposite direction.

        Args:
            duty (int): Duty cycle from -100 to 100%.
        """
    
        #print("SETTING DUTY CYCLE")

        self.PWM_tim.deinit()

        self.PWM_tim.init(freq = 20_000)

        #duty = -duty

        if duty > 100:
            duty = 100
        elif duty < -100:
            duty = -100

        if duty == 0:
            # PH
            self.PWM_tim.channel(self.PH_channel, Timer.PWM, pulse_width_percent=0)
            # DIR
            self.DIR_pin.low()
        # Negative duty cycle
        elif duty <= 0:
            # PH
            self.PWM_tim.channel(self.PH_channel, Timer.PWM, pulse_width_percent=abs(duty))
            # DIR
            self.DIR_pin.high()
        # Regular positive duty cycle
        else:
            # PH
            self.PWM_tim.channel(self.PH_channel, Timer.PWM, pulse_width_percent=duty)
            #self.PH_pin.high()
            # DIR
            self.DIR_pin.low()
       
        

    def enable(self):
        '''Enable one channel of the L6206.
           
        Sets the enable pin associated with one channel of the L6206 high in order to enable that channel of the motor driver'''
        self.SLP_pin.high()
        #print("ENABLE")
        

    def disable(self):
        '''Enable one channel of the L6206.

        Sets the enable pin associated with one channel of the L6206 high in order to enable that channel of the motor driver'''
        #print("DISABLE")
        self.SLP_pin.low()