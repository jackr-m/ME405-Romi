'''!@file
    @brief Main file that runs ME405 Lab 0x04
    @details runs DC motor PID feedback from quadrature encoder and reflectance sensors
    @author Casey Pickett and Jack Miller
    @date 11/07/23
'''

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
gc.collect()
import QTRSensors
gc.collect()

gc.collect()
gc.collect()


# The Task Classes
from UpdateIMU import UpdateIMU
gc.collect()
from UpdateTOF import UpdateTOF
gc.collect()
from Control import Control
gc.collect()
from UpdateMotor import UpdateMotor
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
    
    s_e1_kp: Share           = Share('f')
    s_e1_ki: Share           = Share('f')
    s_e1_kd: Share           = Share('f')
    s_e1_speed: Share        = Share('f')
    s_m1_feedbackOn: Share   = Share('b')

    s_e2_kp: Share           = Share('f')
    s_e2_ki: Share           = Share('f')
    s_e2_kd: Share           = Share('f')
    s_e2_speed: Share        = Share('f')
    s_m2_feedbackOn: Share   = Share('b')

    # Size of all of the queues
    q_size = 10

    q_e1_positions:   Queue = Queue('f', q_size)
    s_e1_timeDeltas:  Share = Share('f')
    q_e1_times:       Queue = Queue('f', q_size)
    q_e1_deltas:      Queue = Queue('f', q_size)
    s_e1_velocities:  Share = Share('f')
    q_e1_velocities:  Queue = Queue('f', q_size)

    q_e2_positions:   Queue = Queue('f', q_size)
    s_e2_timeDeltas:  Share = Share('f')
    q_e2_times:       Queue = Queue('f', q_size) 
    q_e2_deltas:      Queue = Queue('f', q_size)
    s_e2_velocities:  Share = Share('f')
    q_e2_velocities:  Queue = Queue('f', q_size)

    #1 means reset requested, 0 means reset complete
    q_e1_resetEncoder: Share = Share('b')
    q_e1_resetEncoder.put(0)

    q_e2_resetEncoder: Share = Share('b')
    q_e2_resetEncoder.put(0)

    #These closed loop params wont get used in open loop
    s_e1_kp.put(2.7) # 2, 100, 0.01
    s_e1_ki.put(100)
    s_e1_kd.put(0.01)

    s_e2_kp.put(2.7)
    s_e2_ki.put(100)
    s_e2_kd.put(0.01)

    s_e1_velocities.put(0)
    s_e2_velocities.put(0)

    s_e1_speed.put(0)
    s_e2_speed.put(0)

    # initialize uart
    uart2 = UART(2, 115200)
    uart2.init(115200, bits=8, parity=None, stop=1)

    #flag to request collection of OL data for 30s
    s_m1_collect_OL_Data: Share = Share('b')
    s_m2_collect_OL_Data: Share = Share('b')

    s_m1_collect_step_data: Share = Share('b')
    s_m2_collect_step_data: Share = Share('b')

    m1_printList: Queue = Queue('b', 10)
    m2_printList: Queue = Queue('b', 10)

    s_m1_feedbackOn.put(0)
    s_m2_feedbackOn.put(0)

    sensor_pins_front = (Pin.cpu.C0, Pin.cpu.C1, Pin.cpu.B0, Pin.cpu.A4, Pin.cpu.A1, Pin.cpu.A0, Pin.cpu.C3)
    emitter_pin_front = Pin.cpu.C2
    qtr_front = QTRSensors.QTRSensors(sensor_pins_front, emitter_pin_front, type=QTRSensors.TYPE_A)
    sensor_pins_rear = (Pin.cpu.C5, Pin.cpu.A5, Pin.cpu.A6, Pin.cpu.A7, Pin.cpu.B1)
    emitter_pin_rear = Pin.cpu.A11
    qtr_rear = QTRSensors.QTRSensors(sensor_pins_rear, emitter_pin_rear, type=QTRSensors.TYPE_A)
    s_qtrPos_front: Share = Share('i')
    s_qtrPos_rear: Share = Share('i')

    lineCenterPosition_front = 3000
    lineCenterPosition_rear = 2000
    
    baseSpeed = 1.0 # rad/s

    s_leftWheelPosition:    Share = Share('d')
    s_rightWheelPosition:   Share = Share('d')

    period_all = 36 # ms

    s_field_heading: Share = Share('d')
    
    s_TOF_Distance: Share = Share('h')

    i2c1 = I2C(1, freq=400_000)

    e1_update_obj = UpdateMotor("A", q_e1_positions, s_e1_timeDeltas, s_e1_velocities, q_e1_velocities, q_e1_times, q_e1_resetEncoder, m1_printList, s_m1_collect_OL_Data, s_m1_collect_step_data, qtrfront=qtr_front, qtrrear=qtr_rear, s_qtrPositionfront=s_qtrPos_front, s_qtrPositionrear=s_qtrPos_rear, s_position=s_leftWheelPosition)
    m1_control_obj = Control("A", s_speed = s_e1_speed, s_kp=s_e1_kp, s_ki=s_e1_ki, s_kd=s_e1_kd, feedbackOn= s_m1_feedbackOn, s_timeDelta=s_e1_timeDeltas, s_velocity=s_e1_velocities)

    e2_update_obj = UpdateMotor("B", q_e2_positions, s_e2_timeDeltas, s_e2_velocities, q_e2_velocities, q_e2_times, q_e2_resetEncoder, m2_printList, s_m2_collect_OL_Data, s_m2_collect_step_data, qtrfront=qtr_front, qtrrear=qtr_rear, s_qtrPositionfront=s_qtrPos_front, s_qtrPositionrear=s_qtrPos_rear, s_position=s_rightWheelPosition)
    m2_control_obj = Control("B", s_speed = s_e2_speed, s_kp=s_e2_kp, s_ki=s_e2_ki, s_kd=s_e2_kd, feedbackOn= s_m2_feedbackOn, s_timeDelta=s_e2_timeDeltas, s_velocity=s_e2_velocities)

    line_follower_obj = LineFollower(qtrPos_front=s_qtrPos_front, qtrPos_rear=s_qtrPos_rear, e1_targetSpeed=s_e1_speed, e2_targetSpeed=s_e2_speed, linePosTarget_front=lineCenterPosition_front, linePosTarget_rear=lineCenterPosition_rear, s_timeDelta=s_e1_timeDeltas, centerSpeed=baseSpeed, e1FeedbackOn=s_m1_feedbackOn, e2FeedbackOn=s_m2_feedbackOn, leftDistanceTraveled=s_leftWheelPosition, rightDistanceTraveled=s_rightWheelPosition, heading=s_field_heading, s_tofdistance=s_TOF_Distance, time_scale=period_all, motor_1_object = m1_control_obj, motor_2_object= m2_control_obj)
    
    tof_updater_obj = UpdateTOF(s_tofdistance=s_TOF_Distance, i2c=i2c1)
    
    imu_updater_obj = UpdateIMU(s_field_heading=s_field_heading, i2c=i2c1)

    #garbage_collector_obj = GarbageCollector()


    update1 = cotask.Task(e1_update_obj.run, name = "Encoder 1", priority = 1, period = period_all*1.0, profile=True)
    update2 = cotask.Task(e2_update_obj.run, name = "Encoder 2", priority = 1, period = period_all*1.0, profile=True)
    control1 = cotask.Task(m1_control_obj.run, name = "Motor 1", priority = 1, period = period_all*1.0, profile=True)
    control2 = cotask.Task(m2_control_obj.run, name = "Motor 2", priority = 1, period = period_all*1.0, profile=True)
    tofUpdate = cotask.Task(tof_updater_obj.run, name = "ToF", priority = 1, period = period_all*1.0, profile=True)
    imuUpdate = cotask.Task(imu_updater_obj.run, name = "IMU", priority = 1, period = period_all*1.0, profile=True)

    lineFollower_task = cotask.Task(line_follower_obj.run, name="Line Follow", priority=1, period=period_all*1.0, profile=True)
    
    #garbageCollector = cotask.Task(garbage_collector_obj.run, name="Garbage Collector", priority=1, period=2*period_all)

    cotask.task_list.append(update1)
    cotask.task_list.append(update2)
    cotask.task_list.append(control1)
    cotask.task_list.append(control2)
    cotask.task_list.append(lineFollower_task)
    cotask.task_list.append(tofUpdate)
    cotask.task_list.append(imuUpdate)
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
        s_e1_speed.put(0)
        s_e2_speed.put(0)
        m1_control_obj._motor.set_duty(0)
        m2_control_obj._motor.set_duty(0)