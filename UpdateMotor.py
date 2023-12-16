"""Houses the UpdateMotor class.
"""

from pyb import Timer, Pin
from task_share import Share, Queue
from RomiCoder import RomiCoder
from utime import sleep_ms
import QTRSensors

class UpdateMotor:
    """Communicates with a motor's respective encoder to provide updated position, and velocity value; also manages a QTR Sensor.

    

    """

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

            self._qtr_front.dimmable(True)
            self._qtr_front.dimmingLevel(20) # 5 for matte, 20 for glossy paper
            self._qtr_front.emittersOn()
            sleep_ms(5)

            self._qtr_front.calibrationOn.load_json('{"initialized": true, "maximum": [3918.0, 3917.5, 3311.75, 3478.75, 3476.25, 3607.75, 3396.75], "minimum": [192.0, 196.75, 188.75, 185.0, 197.0, 191.75, 194.0]}') # 5V calibration
            #self._qtr_front.calibrationOn.load_json('{"initialized": true, "maximum": [2491.75, 2537.0, 2091.5, 2190.0, 2189.25, 2370.5, 2096.5], "minimum": [217.5, 222.25, 213.25, 211.5, 215.75, 215.5, 214.25]}') # 3.3V calibration
            self._qtr_front.calibrationOn.load_json('{"initialized": true, "maximum": [4095.0, 4095.0, 4044.0, 4063.25, 4095.0, 4095.0, 4048.5], "minimum": [295.75, 299.75, 290.5, 286.75, 285.25, 281.0, 279.5]}') # 5V, glossy paper
            self._qtr_front.calibrationOff.load_json( '{"maximum": null, "minimum": null, "initialized": false}' )
        elif motor == "B":
            self._encoder = RomiCoder(Pin.cpu.C6, Pin.cpu.C7, 65535, 0, 8)

            self._qtr_rear.dimmable(True)
            self._qtr_rear.dimmingLevel(20) # 5 for matte, 20 for glossy paper
            self._qtr_rear.emittersOn()
            sleep_ms(5)
            #self._qtr_rear.calibrationOn.load_json('{"initialized": true, "maximum": [4095.0, 2242.25, 4095.0, 4095.0, 4095.0], "minimum": [337.75, 419.75, 405.75, 398.0, 377.0]}') # 5V calibration
            #self._qtr_rear.calibrationOn.load_json('{"initialized": true, "maximum": [3329.5, 2032.25, 3212.5, 3232.5, 3112.25], "minimum": [393.0, 351.75, 309.0, 323.75, 276.75]}') # 3.3V calibration
            self._qtr_rear.calibrationOn.load_json('{"initialized": true, "maximum": [4095.0, 2247.75, 4095.0, 4095.0, 4095.0], "minimum": [320.0, 319.5, 335.25, 333.75, 304.5]}') # 5V, glossy paper
            self._qtr_rear.calibrationOff.load_json( '{"maximum": null, "minimum": null, "initialized": false}' )
        else:
            raise ValueError("Invalid motor")

    def run(self):
        


        while True:
            #immediately go to run state after qtr initialization
            if self._state == 0:

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
