from task_share import Share
import adafruit_vl53l1x
from machine import I2C

class UpdateTOF:
    '''!
        Class for task to update a Time of Flight sensor position
    '''

    def __init__(self, s_tofdistance: Share, i2c):
        """Time of Flight Distance Sensor

        Args:
            s_tofposition (Share): Time of Flight position, reported in millimeters.

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