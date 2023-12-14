from task_share import Share
from BNO055 import BNO055

class UpdateIMU:
    '''!
        Class for task to update a IMU euler angle heading
    '''
    def __init__(self, s_field_heading: Share, i2c):
        """IMU Heading Reader

        Args:
            s_field_heading (Share): Time of Flight position, reported in millimeters.

        """

        self._heading = s_field_heading

        self._i2c = i2c

        self.sensor = BNO055(self._i2c)

        self.sensor.begin()
        self.sensor.set_axis_remap(x=BNO055.AXIS_REMAP_Y, y=BNO055.AXIS_REMAP_X, z=BNO055.AXIS_REMAP_Z, x_sign=BNO055.AXIS_REMAP_NEGATIVE, y_sign=BNO055.AXIS_REMAP_POSITIVE, z_sign=BNO055.AXIS_REMAP_POSITIVE)
        self.sensor.setCalibrationData([0xeb,0xff,0xfd,0xff,0xe1,0xff,0x1a,0x3,0x49,0xf7,0x6,0x7,0xfe,0xff,0xfe,0xff,0x0,0x0,0xe8,0x3,0x70,0x1])

        self.sensor.setMode(BNO055.OPERATION_MODE_NDOF)

        self._state = 0

    def run(self):
        while True:
            # get the position and put it in the share
            if self._state == 0:
                self._heading.put(self.sensor.getVector(BNO055.VECTOR_EULER)[0])
            else:
                raise ValueError("Invalid state")
                
            yield self._state