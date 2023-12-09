'''!@file
    @brief Main file that runs ME405 Lab 0x04
    @details runs DC motor PID feedback from quadrature encoder and reflectance sensors
    @author Casey Pickett and Jack Miller
    @date 11/07/23
'''

import gc
from pyb import Timer, Pin, UART, repl_uart, USB_VCP
import micropython
from task_share import Queue, Share
import cotask
from nb_input import NB_Input
from time import ticks_ms, ticks_diff, sleep_ms
from math import pi, cos, sin

gc.collect()
from RomiMotor import RomiMotor
from RomiCoder import RomiCoder
gc.collect()
from ClosedLoop import ClosedLoop
gc.collect()
import QTRSensors
gc.collect()
from PID import PID
gc.collect()
from machine import I2C
import adafruit_vl53l1x
gc.collect()
from BNO055 import BNO055
gc.collect()

class Control:
    '''! A class to control a DC motor
    '''

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
                
                if self._feedbackOn.get() == 1:
                    self._state = 2
                elif  self._feedbackOn.get() == 0:
                    self._state = 1
                else:
                    print("stuck in init")


            elif self._state == 1:

                #print(self._desiredSpeed.get())

                #direct open loop speed setting
                #_speed = self._desiredSpeed.get()
                #print("Motor: {0}, speed: {1}".format(self._motorLetter, _speed))
                self._motor.set_duty(self._desiredSpeed.get())

                #change to closed loop if needed
                if self._feedbackOn.get() == 1:
                    self._desiredSpeed.put(0)
                    self._state = 2
            
            elif self._state == 2:

                self._closedLoop.setTarget(self._desiredSpeed.get())
                self._closedLoop.setKp(self._kp.get())
                self._closedLoop.setKi(self._ki.get())
                self._closedLoop.setKd(self._kd.get())

                self._efforts = self._closedLoop.calculateEfforts(self._velocities.get(), self._timeDeltas.get())
                #change to open loop if needed
                if self._feedbackOn.get() == 0:
                    self._desiredSpeed.put(0)
                    self._state = 1

                self._motor.set_duty(int(round(sum(self._efforts), 0)))

                #gc.collect()

            yield self._state

class UpdateMotor:
    '''!
        Class for task to update an encoder position at a different frequency compared to the 
        respective motor control loop
    '''

    def __init__(self, motor, q_positions: Queue, s_timeDelta: Share, s_velocity: Share, q_velocities: Queue, q_times: Queue, resetRequest: Share, printList: Queue, collectOLData: Share, collectStepData: Share, qtrfront: QTRSensors.QTRSensors, qtrrear: QTRSensors.QTRSensors, s_qtrPositionfront: Share, s_qtrPositionrear: Share, s_position: Share):
        '''
            @param encoder is the encoder object to get data from. This will already be initialized
            @param _reset is a flag where "1" means a request to reset the encoder
        '''

        self._positions = q_positions
        self._velocities = q_velocities
        self._timeDelta = s_timeDelta
        self._velocity = s_velocity
        self._times = q_times
        self._reset = resetRequest
        self._printList = printList
        self._collectOLData = collectOLData
        self._collectStepData = collectStepData
        self._qtr_front = qtrfront
        self._qtr_rear = qtrrear
        self._qtrPosition_front = s_qtrPositionfront
        self._qtrPosition_rear = s_qtrPositionrear

        self._motorLetter = motor

        self._position = s_position

        #init state
        self._state = 0

        # Motor object
        if motor == "A":
            self._encoder = RomiCoder(Pin.cpu.B6, Pin.cpu.B7, 65535, 0, 4)
        elif motor == "B":
            self._encoder = RomiCoder(Pin.cpu.C6, Pin.cpu.C7, 65535, 0, 8)
        else:
            raise ValueError("Invalid motor")

    def run(self):
        while True:
            #immediately go to run state after qtr initialization
            if self._state == 0:

                if self._motorLetter == "A":
                    self._qtr_front.dimmable(True)
                    self._qtr_front.dimmingLevel(15)
                    self._qtr_front.emittersOn()
                    sleep_ms(5)

                    #self._qtr_front.calibrationOn.load_json('{"initialized": true, "maximum": [4080.0, 4092.5, 3811.5, 3909.25, 3892.0, 3983.75, 3729.5], "minimum": [213.5, 226.75, 217.75, 216.75, 219.75, 225.75, 225.5]}') # 5V calibration
                    self._qtr_front.calibrationOn.load_json('{"initialized": true, "maximum": [2491.75, 2537.0, 2091.5, 2190.0, 2189.25, 2370.5, 2096.5], "minimum": [217.5, 222.25, 213.25, 211.5, 215.75, 215.5, 214.25]}') # 3.3V calibration
                    self._qtr_front.calibrationOff.load_json( '{"maximum": null, "minimum": null, "initialized": false}' )

                if self._motorLetter == "B":
                    self._qtr_rear.dimmable(True)
                    self._qtr_rear.dimmingLevel(15)
                    self._qtr_rear.emittersOn()
                    sleep_ms(5)
                    #self._qtr_rear.calibrationOn.load_json('{"initialized": true, "maximum": [4095.0, 2207.0, 4095.0, 4095.0, 4095.0], "minimum": [352.75, 361.0, 373.75, 373.75, 340.0]}') # 5V calibration
                    self._qtr_rear.calibrationOn.load_json('{"initialized": true, "maximum": [3329.5, 2032.25, 3212.5, 3232.5, 3112.25], "minimum": [393.0, 351.75, 309.0, 323.75, 276.75]}') # 3.3V calibration
                    self._qtr_rear.calibrationOff.load_json( '{"maximum": null, "minimum": null, "initialized": false}' )

                self._encoder.zero()
                self._state = 1

            elif self._state == 1:

                self._encoder.update()

                self._velocity.put(self._encoder.get_rate())

                self._position.put(self._encoder.get_position() * 69.5/2) #  wheel radius is 35mm nominal

                self._timeDelta.put(self._encoder.get_timeDelta())

                if self._motorLetter == "A":
                    self._qtrPosition_front.put(self._qtr_front.readLineBlack())
                elif self._motorLetter == "B":
                    self._qtrPosition_rear.put(self._qtr_rear.readLineBlack())

                #if a reset is requested, change state so that the reset will be handled and clear the flag
                if self._reset.get() == 1:
                    self._state = 0
                    self._reset.put(0)
                

            #always will be in state 1
            #gc.collect()
            yield self._state

class GarbageCollector:
    def __init__(self):
        self._state = 0
    def run(self):
        if self._state == 0:
            gc.collect()
        else:
            raise ValueError("Invalid state")
        yield self._state

class UpdateTOF:
    '''!
        Class for task to update a Time of Flight sensor position
    '''

    def __init__(self, s_tofdistance: Share):
        """Time of Flight Distance Sensor

        Args:
            s_tofposition (Share): Time of Flight position, reported in millimeters.

        """

        self._tofdistance = s_tofdistance
        self._tofdistance.put(400) # start with a long distance on initialization
        
        self._i2c = I2C(1, freq=400_000)

        self.vl53 = adafruit_vl53l1x.VL53L1X(self._i2c)

        #init state
        self._state = 0

    def run(self):
        while True:
            # immediately go to run state after tof initialization
            if self._state == 0:
                self.vl53.distance_mode = 1 # short range = 1, long range = 2
                self.vl53.timing_budget = 100 # ms
                self._state = 1

            # start ranging
            elif self._state == 1:
                self.vl53.start_ranging()
                #self._tofdistance.put(self.vl53.distance)
                self._state = 2

            # get the position and put it in the share
            elif self._state == 2:
                try:
                    self._distance = int(self.vl53.distance)
                    self._tofdistance.put(self._distance)
                except TypeError:
                    continue
                self.vl53.clear_interrupt()
                #gc.collect()

            yield self._state

class UpdateIMU:
    '''!
        Class for task to update a IMU euler angle heading
    '''
    def __init__(self, s_field_heading: Share):
        """IMU Heading Reader

        Args:
            s_field_heading (Share): Time of Flight position, reported in millimeters.

        """

        self._heading = s_field_heading

        self.sensor = BNO055()

        self._state = 0

    def run(self):
        while True:
            # IMU initialization
            if self._state == 0:
                self.sensor.begin()
                self._state = 1
            # set axis remap
            elif self._state == 1:
                self.sensor.set_axis_remap(x=BNO055.AXIS_REMAP_Y, y=BNO055.AXIS_REMAP_X, z=BNO055.AXIS_REMAP_Z, x_sign=BNO055.AXIS_REMAP_NEGATIVE, y_sign=BNO055.AXIS_REMAP_POSITIVE, z_sign=BNO055.AXIS_REMAP_POSITIVE)
                self._state = 2
            # set calibration
            elif self._state == 2:
                self.sensor.setCalibrationData([0xeb,0xff,0xfd,0xff,0xe1,0xff,0x1a,0x3,0x49,0xf7,0x6,0x7,0xfe,0xff,0xfe,0xff,0x0,0x0,0xe8,0x3,0x70,0x1])
                self._state = 3

            # get the position and put it in the share
            elif self._state == 3:
                self._heading.put(self.sensor.getVector(BNO055.VECTOR_EULER)[0])
                
            yield self._state

class LineFollower:
    """this is a line follower, it follows lines"""
    def __init__(self, qtrPos_front: Share, qtrPos_rear: Share, e1_targetSpeed: Share, e2_targetSpeed: Share, s_timeDelta: Share, e1FeedbackOn: Share, e2FeedbackOn: Share,  leftDistanceTraveled: Share, rightDistanceTraveled: Share, heading: Share,  s_tofdistance: Share, linePosTarget_front: int = 3000, linePosTarget_rear: int = 2000, centerSpeed: float = 2, time_scale: int = 20):
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

        self.kp = 1.15e-4
        self.ki = 1.05e-4
        self.kd = self.kp / 1000
        self._linePosPID = PID(self.kp, self.ki, self.kd, self._linePosTarget_front, sample_time=self.time_scale, scale="ms", proportional_on_measurement=False)

    def run(self):
        while True:
            if self._state == 0:
                self._e1FeedbackOn.put(1)
                self._e2FeedbackOn.put(1)
                self._state = 1

            elif self._state == 1:
                
                if self.user_button.value() == 0:
                    self.ki *= 0.98 # increase by 2% every button press
                    #self.kd = 0 * self.kp
                    #self._linePosPID.Kp = self.kp
                    self._linePosPID.Ki = self.ki

                # Scale rear position to have same full scale as front sensor array
                rearPos = (self._qtrPos_rear.get()) * (self._linePosTarget_front/self._linePosTarget_rear)
                frontPos = self._qtrPos_front.get()

                if rearPos==0 and frontPos==0:
                    pos = 0
                elif (rearPos<2700 or 3300<rearPos) and 2700<frontPos<3300:
                    pos = frontPos
                elif 2700<rearPos<3300 and (frontPos<2700 or 3300<frontPos):
                    pos = rearPos
                elif rearPos==6000 and frontPos==6000:
                    pos = 6000
                else:
                    pos = (frontPos + rearPos)/2
                    #pos = frontPos

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

                if self._tofdistance.get() <= 50:
                   self._state = 2

            elif self._state == 2:
                self._e1SpeedTarget.put(0)
                self._e2SpeedTarget.put(0)
                #print("DISTANCE MET")
            
            yield self._state

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
    s_e1_kp.put(2) # 2, 100, 0.01
    s_e1_ki.put(100)
    s_e1_kd.put(0.01)

    s_e2_kp.put(2)
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
    #baseSpeed = 4.2 rad/s
    baseSpeed = 1.5

    s_leftWheelPosition:    Share = Share('d')
    s_rightWheelPosition:   Share = Share('d')

    period_all = 50 # ms

    s_field_heading: Share = Share('d')
    
    s_TOF_Distance: Share = Share('h')

    e1_update_obj = UpdateMotor("A", q_e1_positions, s_e1_timeDeltas, s_e1_velocities, q_e1_velocities, q_e1_times, q_e1_resetEncoder, m1_printList, s_m1_collect_OL_Data, s_m1_collect_step_data, qtrfront=qtr_front, qtrrear=qtr_rear, s_qtrPositionfront=s_qtrPos_front, s_qtrPositionrear=s_qtrPos_rear, s_position=s_leftWheelPosition)
    m1_control_obj = Control("A", s_speed = s_e1_speed, s_kp=s_e1_kp, s_ki=s_e1_ki, s_kd=s_e1_kd, feedbackOn= s_m1_feedbackOn, s_timeDelta=s_e1_timeDeltas, s_velocity=s_e1_velocities)

    e2_update_obj = UpdateMotor("B", q_e2_positions, s_e2_timeDeltas, s_e2_velocities, q_e2_velocities, q_e2_times, q_e2_resetEncoder, m2_printList, s_m2_collect_OL_Data, s_m2_collect_step_data, qtrfront=qtr_front, qtrrear=qtr_rear, s_qtrPositionfront=s_qtrPos_front, s_qtrPositionrear=s_qtrPos_rear, s_position=s_rightWheelPosition)
    m2_control_obj = Control("B", s_speed = s_e2_speed, s_kp=s_e2_kp, s_ki=s_e2_ki, s_kd=s_e2_kd, feedbackOn= s_m2_feedbackOn, s_timeDelta=s_e2_timeDeltas, s_velocity=s_e2_velocities)

    line_follower_obj = LineFollower(qtrPos_front=s_qtrPos_front, qtrPos_rear=s_qtrPos_rear, e1_targetSpeed=s_e1_speed, e2_targetSpeed=s_e2_speed, linePosTarget_front=lineCenterPosition_front, linePosTarget_rear=lineCenterPosition_rear, s_timeDelta=s_e1_timeDeltas, centerSpeed=baseSpeed, e1FeedbackOn=s_m1_feedbackOn, e2FeedbackOn=s_m2_feedbackOn, leftDistanceTraveled=s_leftWheelPosition, rightDistanceTraveled=s_rightWheelPosition, heading=s_field_heading, s_tofdistance=s_TOF_Distance, time_scale=period_all)
    
    tof_updater_obj = UpdateTOF(s_tofdistance=s_TOF_Distance)
    
    imu_updater_obj = UpdateIMU(s_field_heading=s_field_heading)

    garbage_collector_obj = GarbageCollector()

    update1 = cotask.Task(e1_update_obj.run, name = "Encoder 1 Update", priority = 7, period = period_all)
    update2 = cotask.Task(e2_update_obj.run, name = "Encoder 2 Update", priority = 6, period = period_all)
    control1 = cotask.Task(m1_control_obj.run, name = "Motor 1 Control loop", priority = 5, period = period_all)
    control2 = cotask.Task(m2_control_obj.run, name = "Motor 2 Control loop", priority = 4, period = period_all)
    tofUpdate = cotask.Task(tof_updater_obj.run, name = "Time of Flight Updater", priority = 8, period = 3*period_all)
    imuUpdate = cotask.Task(imu_updater_obj.run, name = "IMU Updater", priority = 9, period = 2*period_all)

    lineFollower = cotask.Task(line_follower_obj.run, name="Line Follower", priority=3, period=period_all)
    
    garbageCollector = cotask.Task(garbage_collector_obj.run, name="Garbage Collector", priority=1, period=2*period_all)

    cotask.task_list.append(update1)
    cotask.task_list.append(update2)
    cotask.task_list.append(control1)
    cotask.task_list.append(control2)
    cotask.task_list.append(lineFollower)
    cotask.task_list.append(tofUpdate)
    cotask.task_list.append(imuUpdate)
    #cotask.task_list.append(garbageCollector)

    try:
        while True:
            cotask.task_list.pri_sched()
    
    # turn motors off on Ctrl-C
    except KeyboardInterrupt:
        s_e1_speed.put(0)
        s_e2_speed.put(0)
        m1_control_obj._motor.set_duty(0)
        m2_control_obj._motor.set_duty(0)