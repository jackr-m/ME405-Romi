"""Houses the UpdateQTRs task class."""

from RomiCoder import RomiCoder
from pyb import Pin
from utime import sleep_ms
import QTRSensors
from task_share import Share

class UpdateQTRs:

    def __init__(self, qtrfront: QTRSensors.QTRSensors, qtrrear: QTRSensors.QTRSensors, s_pos: Share, linePosTarget_front: int = 3000, linePosTarget_rear: int = 2000):
        """"Manages BOTH QTR sensors.
    
        Returns a singular s_pos value to replace the "pos" variable that used to be in LineFollower.

        Args: 
            qtrfront: QTRSensors.QTRSensors object, front sensor
            qtrrear: QTRSensors.QTRSensors object, rear sensor
            s_pos: Share object, composite guess of line position
            linePosTarget_front: integer target front line pos, only used for rear position scaling in this class
            linePosTarget_rear: integer target rear line pos, only used for rear position scaling in this class
        """

        self._state = 0

        self._qtr_front = qtrfront
        self._qtr_rear = qtrrear

        self._pos = s_pos

        self._frontLinePosTarget = linePosTarget_front
        self._rearLinePosTarget = linePosTarget_rear

        self._qtr_front.dimmable(True)
        self._qtr_front.dimmingLevel(20) # 5 for matte, 20 for glossy paper
        self._qtr_front.emittersOn()
        sleep_ms(5)

        #self._qtr_front.calibrationOn.load_json('{"initialized": true, "maximum": [3918.0, 3917.5, 3311.75, 3478.75, 3476.25, 3607.75, 3396.75], "minimum": [192.0, 196.75, 188.75, 185.0, 197.0, 191.75, 194.0]}') # 5V calibration
        #self._qtr_front.calibrationOn.load_json('{"initialized": true, "maximum": [2491.75, 2537.0, 2091.5, 2190.0, 2189.25, 2370.5, 2096.5], "minimum": [217.5, 222.25, 213.25, 211.5, 215.75, 215.5, 214.25]}') # 3.3V calibration
        
        self._qtr_front.calibrationOn.load_json('{"initialized": true, "maximum": [4095.0, 4095.0, 4044.0, 4063.25, 4095.0, 4095.0, 4048.5], "minimum": [295.75, 299.75, 290.5, 286.75, 285.25, 281.0, 279.5]}') # 5V, glossy paper
        self._qtr_front.calibrationOff.load_json( '{"maximum": null, "minimum": null, "initialized": false}' )

        self._qtr_rear.dimmable(True)
        self._qtr_rear.dimmingLevel(20) # 5 for matte, 20 for glossy paper
        self._qtr_rear.emittersOn()
        sleep_ms(5)
        #self._qtr_rear.calibrationOn.load_json('{"initialized": true, "maximum": [4095.0, 2242.25, 4095.0, 4095.0, 4095.0], "minimum": [337.75, 419.75, 405.75, 398.0, 377.0]}') # 5V calibration
        #self._qtr_rear.calibrationOn.load_json('{"initialized": true, "maximum": [3329.5, 2032.25, 3212.5, 3232.5, 3112.25], "minimum": [393.0, 351.75, 309.0, 323.75, 276.75]}') # 3.3V calibration
        self._qtr_rear.calibrationOn.load_json('{"initialized": true, "maximum": [4095.0, 2247.75, 4095.0, 4095.0, 4095.0], "minimum": [320.0, 319.5, 335.25, 333.75, 304.5]}') # 5V, glossy paper
        self._qtr_rear.calibrationOff.load_json( '{"maximum": null, "minimum": null, "initialized": false}' )
   
    def run(self):
            
            while True:
                #immediately go to run state after qtr initialization
                if self._state == 0:

                    self._state = 1

                elif self._state == 1:

                    _rearPos = self._qtr_rear.readLineBlack()
                    _frontPos = self._qtr_front.readLineBlack()


                    # Scale rear position to have same full scale as front sensor array
                    _rearPos = int(_rearPos*(self._frontLinePosTarget/self._rearLinePosTarget))
                    
                    print(_rearPos)

                    if _rearPos==0 and _frontPos==0:
                        self._pos.put(0)
                    elif (_rearPos<2500 or 3500<_rearPos) and 2500<_frontPos<3500:
                        self._pos.put(_frontPos)
                    elif 2500<_rearPos<3500 and (_frontPos<2500 or 3500<_frontPos):
                        self._pos.put(_rearPos)
                    elif _rearPos==6000 and _frontPos==6000:
                        self._pos.put(6000)
                    else:
                        #pos = (frontPos + rearPos)/2
                        self._pos.put(_frontPos)

                    # dead zone
                    if 2600<self._pos.get()<3400:
                        self._pos.put(3000)

                #always will be in state 1
                #gc.collect()
                yield self._state
