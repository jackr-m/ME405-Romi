"""Houses the UpdateTOF task class.
"""

from task_share import Share
import adafruit_vl53l1x
from machine import I2C

class UpdateTOF:
   
    def __init__(self, s_tofdistance: Share, i2c):
        """Updates an dafruit_vl53l1x Time of Flight sensor position.

        Args:
            s_tofposition: a task_share.Share object conveying the Time of Flight position, reported in millimeters.
            i2c: a machine.I2C object, pre-initialized, designating the i2c bus used.
        """

        self._tofdistance = s_tofdistance
        self._tofdistance.put(400) # start with a long distance on initialization
        self._distance = 400
        
        self._i2c = i2c

        self.vl53 = adafruit_vl53l1x.VL53L1X(self._i2c)

        self.vl53.distance_mode = 1 # short range = 1, long range = 2
        self.vl53.timing_budget = 33 # ms

        #init state
        self._state = 0

    def run(self):
        """Implementation as a generator function.
        """

        while True:
            # immediately go to run state after tof initialization
            if self._state == 0:
                self.vl53.start_ranging()
                self.vl53.clear_interrupt()
                self._state = 1

            # get the position and put it in the share
            elif self._state == 1:
                try:
                    self._distance = int(self.vl53.distance)
                    self._tofdistance.put(self._distance)
                except TypeError:
                    self._tofdistance.put(self._distance)
                self.vl53.clear_interrupt()
                #gc.collect()

            yield self._state