"""Main Program. 

    Implements generator "run function" for all tasks. Sets motor power to 0 when a KeyboardInterrupt occurs. The romi chassis follows a line until it gets in range of an obstacle, which it will circumnavigate to get back on the line. It goes until its coordinates indicate it is at the finish, then turns around and goes to the start.
    Motor control task handles both encoders and both motors in one instance
    QTR Update task handles both QTR sensors at once and returns just a composite position (s_pos = pos)

"""

import gc
from pyb import Timer, Pin, UART, repl_uart, USB_VCP
import micropython
from machine import I2C
from task_share import Queue, Share
import cotask
#from nb_input import NB_Input
from utime import ticks_ms, ticks_diff, sleep_ms
from math import pi, cos, sin
gc.collect()

import QTRSensors
gc.collect()



# The Task Classes
from UpdateIMU import UpdateIMU
gc.collect()
from UpdateTOF import UpdateTOF
gc.collect()
from UpdateQTRs import UpdateQTRs
gc.collect()
from ControlBothMotors import ControlBothMotors
gc.collect()
from LineFollower import LineFollower
gc.collect()
from GarbageCollector import GarbageCollector
gc.collect()



#code to run
if __name__ == "__main__":

    #disable UART REPL
    repl_uart(None)

    # Get full error messages
    micropython.alloc_emergency_exception_buf(300)
    
    motorKp = 2.7
    motorKi = 100
    motorKd = 0.01

    leftRequestedSpeed: Share = Share('f')
    rightRequestedSpeed: Share = Share('f')

    leftRequestedSpeed.put(0)
    rightRequestedSpeed.put(0)

    # initialize uart
    uart2 = UART(2, 115200)
    uart2.init(115200, bits=8, parity=None, stop=1)


    sensor_pins_front = (Pin.cpu.C0, Pin.cpu.C1, Pin.cpu.B0, Pin.cpu.A4, Pin.cpu.A1, Pin.cpu.A0, Pin.cpu.C3)
    emitter_pin_front = Pin.cpu.C2
    qtr_front = QTRSensors.QTRSensors(sensor_pins_front, emitter_pin_front, type=QTRSensors.TYPE_A)
    sensor_pins_rear = (Pin.cpu.C5, Pin.cpu.A5, Pin.cpu.A6, Pin.cpu.A7, Pin.cpu.B1)
    emitter_pin_rear = Pin.cpu.A11
    qtr_rear = QTRSensors.QTRSensors(sensor_pins_rear, emitter_pin_rear, type=QTRSensors.TYPE_A)

    lineCenterPosition_front = 3000
    lineCenterPosition_rear = 2000
    
    baseSpeed = 0.9 # rad/s

    s_xcoord: Share = Share('d')
    s_ycoord: Share = Share('d')

    period_all = 30 # ms
    lineFollowerPeriod = period_all

    s_field_heading: Share = Share('d')
    
    s_TOF_Distance: Share = Share('h')

    s_pos: Share = Share('i')

    i2c1 = I2C(1, freq=400_000)

    motor_control_obj = ControlBothMotors(motorKp, motorKi, motorKd, leftRequestedSpeed, rightRequestedSpeed, s_xCoord=s_xcoord, s_yCoord=s_ycoord, s_heading=s_field_heading)

    line_follower_obj = LineFollower(s_pos=s_pos, leftTargetSpeed=leftRequestedSpeed, rightTargetSpeed=rightRequestedSpeed, s_xCoord=s_xcoord, s_yCoord=s_ycoord, heading=s_field_heading, s_tofdistance=s_TOF_Distance, linePosTarget_front=lineCenterPosition_front, linePosTarget_rear=lineCenterPosition_rear, centerSpeed=baseSpeed)
    
    tof_updater_obj = UpdateTOF(s_tofdistance=s_TOF_Distance, i2c=i2c1)
    
    imu_updater_obj = UpdateIMU(s_field_heading=s_field_heading, i2c=i2c1)

    qtr_updater_obj = UpdateQTRs(qtrfront=qtr_front, qtrrear=qtr_rear, s_pos=s_pos, linePosTarget_front=lineCenterPosition_front, linePosTarget_rear=lineCenterPosition_rear)

    #garbage_collector_obj = GarbageCollector()

    profiling = False

    motorControl = cotask.Task(motor_control_obj.run, name = "Motors", priority = 1, period = period_all*1.0, profile = profiling)
    tofUpdate = cotask.Task(tof_updater_obj.run, name = "ToF", priority = 1, period = period_all*1.0, profile=profiling)
    imuUpdate = cotask.Task(imu_updater_obj.run, name = "IMU", priority = 1, period = period_all*1.0, profile=profiling)
    qtrUpdate = cotask.Task(qtr_updater_obj.run, name = "QTRs", priority = 1, period = period_all*1.0, profile=profiling)
    lineFollower_task = cotask.Task(line_follower_obj.run, name="Line Follow", priority=1, period=period_all*1.0, profile=profiling)
    
    #garbageCollector = cotask.Task(garbage_collector_obj.run, name="Garbage Collector", priority=1, period=2*period_all)

    cotask.task_list.append(motorControl)
    cotask.task_list.append(lineFollower_task)
    cotask.task_list.append(tofUpdate)
    cotask.task_list.append(imuUpdate)
    cotask.task_list.append(qtrUpdate)
    #cotask.task_list.append(garbageCollector)

    runNumber = 0

    try:
        while True:
            cotask.task_list.pri_sched()
            # runNumber += 1
            # if runNumber >= 5:
            #      print('\n' + str (cotask.task_list))
            #      print('')
            #      runNumber = 0

    # turn motors off on Ctrl-C
    except KeyboardInterrupt:
        leftRequestedSpeed.put(0)
        rightRequestedSpeed.put(0)
        motor_control_obj._leftMotor.set_duty(0)
        motor_control_obj._rightMotor.set_duty(0)