# See qtrsensors.py by MCHobby on Github

#
# The MIT License (MIT)
#
# Copyright (c) 2019 Meurisse D. for MC Hobby
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


# See qrt-sensors-arduino by Pololu on Github (MIT License)

from micropython import const
from machine import Pin
import machine
from pyb import ADC
from array import array
import utime as time
from utime import ticks_diff
#import gc

# QTRReadMode

# QTRReadMode
# https://github.com/pololu/qtr-sensors-arduino/blob/master/QTRSensors.h

READMODE_OFF = const(0)
"""Each reading is made without turning on the infrared (IR) emitters. The reading represents ambient light levels near the sensor."""

READMODE_ON = const(1)
"""Each reading is made with the emitters on. The reading is a measure of reflectance."""

READMODE_ON_AND_OFF = const(2)
"""For each sensor, a reading is made in both the on and off states. The
   value returned is **on + max &minus; off**, where **on** and **off** are
   the reading with the emitters on and off, respectively, and **max** is
   the maximum possible sensor reading. This mode can reduce the amount of
   interference from uneven ambient lighting."""

READMODE_ODD_EVEN = const(3)
"""The odd-numbered sensors are read with the odd-numbered emitters on, then
   the even-numbered sensors are read with the even-numbered emitters on.
   This mode can reduce interference between adjacent sensors, especially on
   QTRX sensor boards. It is only usable with second-generation QTR and QTRX
   sensor arrays that have two emitter control pins."""

READMODE_ODD_EVEN_AND_OFF = const(4)
"""The odd and even sensors are read separately with the respective emitters
   on, then all sensors are read with emitters off and **on + max &minus;
   off** is returned. (In other words, this mode combines OddEven and
   OnAndOff.)"""

READMODE_MANUAL = const(5)
"""Calling read() with this mode prevents it from automatically controlling
   the emitters: they are left in their existing states, which allows manual
   control of the emitters for testing and advanced use. Calibrating and
   obtaining calibrated readings are not supported with this mode."""

# QTREmitters
EMITTERS_ALL = const(0)
EMITTERS_ODD = const(1)
EMITTERS_EVEN= const(2)
EMITTERS_NONE= const(3)

NO_EMITTER_PIN = None

# RC
DEFAULT_TIMEOUT = 2500 # microseconds
TYPE_RC = const(0)
"""RC (Digital) Sensor Type"""

# ANALOG
MAX_VALUE = 4095 # 12 bit ADC max value
DEFAULT_SAMPLES_PER_SENSOR = 4
TYPE_A = const(1)
"""Analog Sensor Type"""

MAX_SENSORS = const(31)

class CalibrationData:
    def __init__(self) -> None:
        self.initialized = False
        self.minimum = None
        self.maximum = None
    def as_json( self ):
        import json
        _dict = {'initialized' : self.initialized, 'minimum' : self.minimum, 'maximum' : self.maximum }
        return json.dumps( _dict )
    def load_json( self, _str ):
        import json
        _dict = json.loads(_str)
        self.initialized = _dict['initialized']
        self.minimum = _dict['minimum']
        self.maximum = _dict['maximum']
    
class QTRSensors():
    def __init__(self, pins: list[machine.Pin], emitterPin: machine.Pin, evenEmitterPin: machine.Pin=None, timeout: int=DEFAULT_TIMEOUT, maxValue: int=MAX_VALUE, samples: int=DEFAULT_SAMPLES_PER_SENSOR, type=TYPE_RC) -> None:
        """Represents a QTR sensor array.
       
        An instance of this class represents a QTR sensor array, consisting of one or more sensors of the same type.
        This could be either a single QTR sensor board or multiple boards controlled as a group.

        Args:
            pins (list[machine.Pin]): List of machine.Pin objects representing the reflectance sensors to be read
            emitterPin (machine.Pin): machine.Pin object for the IR emitter control
            evenEmitterPin (machine.Pin, optional): machine.Pin object for the option IR emitter control of even numbered emitters. Defaults to None.
            timeout (int, optional): Timeout value in microseconds for RC (digital) sensors. Defaults to DEFAULT_TIMEOUT (2500).
            maxValue (int, optional): Maximum ADC reading value for analog sensors. Defaults to MAX_VALUE (4095).
            samples (int, optional): Number of samples to take for analog sensors. Defaults to DEFAULT_SAMPLES_PER_SENSOR (4).
            type : Type of sensor used, RC (digital) or A (analog). Defaults to TYPE_RC (digital).

        Raises:
            ValueError: Number of pins exceeds maximum allowable
            ValueError: Invalid sensor type
        """
        if len(pins) >= MAX_SENSORS:
            raise ValueError("Number of pins exceeds maximum allowable")

        self._sensorPins = pins
        self._sensorCount = len(pins)
        self.__sensorValues = array('f', (len(pins))*[0])
        self.__maxSensorValues = array('f', (len(pins))*[0])
        self.__minSensorValues = array('f', (len(pins))*[0])

        self.__type = type

        if self.__type == 0:
            self._maxValue = timeout
        elif self.__type == 1:
            self._maxValue = maxValue
            for i in self._sensorPins:
                i.init(Pin.ANALOG)
            self._sensorADCs = [ADC(i) for i in self._sensorPins]
        else:
            raise ValueError('Invalid sensor type')

        self._samplesPerSensor = samples

        self.calibrationOn = CalibrationData()
        self.calibrationOff = CalibrationData()

        self._oddEmitterPin = emitterPin # also used for single emitter pin
        self._evenEmitterPin = evenEmitterPin
        self._emitterPinCount = 1 + (1 if evenEmitterPin != None else 0) # at least 1
        self._oddEmitterPin.init(Pin.OUT)
        if self._evenEmitterPin != None:
            self._evenEmitterPin.init(Pin.OUT)
        
        self._dimmable = True
        self._dimmingLevel = 0

        self._lastPosition = 0
    
    def values(self):
        """Get the last read sensor values

        Returns:
            array[float]: Raw or calibrated (depending if the most recently done read was `read()` or `readCalibrated()` ) sensor values.
        """
        return self.__sensorValues
    
    def setTypeAnalog(self, maxValue: int=MAX_VALUE):
        """Set the sensor type to analog.

        Args:
            maxValue (int, optional): _description_. Defaults to MAX_VALUE (4095).
        """
        self.__type = 1
        self._maxValue = maxValue
    
    def setTypeRC(self, timeout: int=DEFAULT_TIMEOUT):
        """Set the sensor type to RC (digital).

        Args:
            timeout (int, optional): Timeout value in microseconds. Defaults to DEFAULT_TIMEOUT (2500).
        """
        self.__type = 0
        self._maxValue = timeout
    
    def setTimeout(self, timeout: int=DEFAULT_TIMEOUT):
        """Set the timeout value for RC (digital) sensors.

        Args:
            timeout (int, optional): Timeout value in microseconds. Defaults to DEFAULT_TIMEOUT (2500).
        """
        if timeout > 32767:
            self._maxValue = 32767
        else:
            self._maxValue = timeout
    
    def setSamplesPerSensor(self, samples: int=DEFAULT_SAMPLES_PER_SENSOR):
        """Set the samples per sensor for analog sensors.

        Args:
            samples (int, optional): Number of samples to be taken per read per sensor. Defaults to DEFAULT_SAMPLES_PER_SENSOR (4).
        """
        if samples > 64:
            self._samplesPerSensor = 64
        else:
            self._samplesPerSensor = samples

    def __emittersOnWithPin( self, pin ): #pin:uint8_t return:uint16_t
        if self._dimmable and (pin.value() == 1) :
            # We are turning on dimmable emitters that are already on. To avoid messing
            # up the dimming level, we have to turn the emitters off and back on. This
            # means the turn-off delay will happen even if wait = false was passed to
            # emittersOn(). (Driver min is 1 ms.)
            pin.value( 0 )
            time.sleep_us(1200)

        pin.value(1)
        emittersOnStart = time.ticks_us()

        if self._dimmable and (self._dimmingLevel > 0):
            for i in range( self._dimmingLevel ): #(uint8_t i = 0; i < _dimmingLevel; i++)
                time.sleep_us(1)
                pin.value(0)
                time.sleep_us(1)
                pin.value(1)

        return emittersOnStart
    
    def __calibrateOnOrOff(self, calOn, mode): # CalibrationData & calibration, QTRReadMode mode):
        # Handles the actual calibration, including (re)allocating and
        # initializing the storage for the calibration values if necessary.
        #DEBUG print("calibrateOnOrOff.in")
        for i in range( self._sensorCount ):
            self.__sensorValues[i] = 0
            self.__maxSensorValues[i] = 0
            self.__minSensorValues[i] = 0
        if calOn: # If we do calibrateOn
            calibration = self.calibrationOn
        else:
            calibration = self.calibrationOff

        # (Re)allocate and initialize the arrays if necessary.
        if not calibration.initialized:
            #DEBUG print("calibrateOnOrOff.calibration-init")
            # Looks not used! oldMaximum = list( calibration.maximum ) # Make a copy
            calibration.maximum = [0]*self._sensorCount

            # Looks not used! oldMinimum = list(calibration.minimum) # Make a copy
            calibration.minimum = [self._maxValue]*self._sensorCount
            calibration.initialized = True

        #DEBUG print("calibrateOnOrOff.10*read")
        for j in range( 10 ): # (uint8_t j = 0; j < 10; j++)
            # self.read( sensorValues, mode)# DEBUG NOK
            self.read( mode ) # DEBUG_NOK
            for i in range( self._sensorCount ): # (uint8_t i = 0; i < _sensorCount; i++)
                # set the max we found THIS time
                if (j == 0) or (self.__sensorValues[i] > self.__maxSensorValues[i]) :
                    self.__maxSensorValues[i] = self.__sensorValues[i]
                # set the min we found THIS time
                if (j == 0) or (self.__sensorValues[i] < self.__minSensorValues[i]) :
                    self.__minSensorValues[i] = self.__sensorValues[i]
        #DEBUG print("calibrateOnOrOff.record-min-max")
        # record the min and max calibration values
        for i in range( self._sensorCount ):
            # Update maximum only if the min of 10 readings was still higher than it
            # (we got 10 readings in a row higher than the existing maximum).
            if self.__minSensorValues[i] > calibration.maximum[i]:
                calibration.maximum[i] = self.__minSensorValues[i]

            # Update minimum only if the max of 10 readings was still lower than it
            # (we got 10 readings in a row lower than the existing minimum).
            if self.__maxSensorValues[i] < calibration.minimum[i]:
                calibration.minimum[i] = self.__maxSensorValues[i]
        #gc.collect()
        #DEBUG print("calibrateOnOrOff.Collect+Out")
    
    def emittersOff(self, emitters=EMITTERS_ALL, wait: bool=True):
        """Turns the IR LEDs off.
        This function is mainly for use by the read() method. Since read()
        normally turns the emitters on and off automatically for each reading,
        calling this function yourself will not affect the readings unless the
        read mode is READMODE_MANUAL, which tells read() to leave the
        emitters alone.

        Args:
            emitters (optional): Which emitters to turn off. Defaults to EMITTERS_ALL.
            wait (bool, optional): If true, this function delays to give the sensors time to turn off before returning. Otherwise, it returns immediately. Defaults to True.
        """
        
        assert emitters in (EMITTERS_ALL,EMITTERS_ODD,EMITTERS_EVEN)

        pinChanged = False
        # Use odd emitter pin in these cases:
        # - 1 emitter pin, emitters = all
        # - 2 emitter pins, emitters = all
        # - 2 emitter pins, emitters = odd
        if (emitters==EMITTERS_ALL) or ((self._emitterPinCount==2) and (emitters==EMITTERS_ODD)):
            # Check if pin is defined and only turn off if not already off
            if (self._oddEmitterPin != None) and ( self._oddEmitterPin.value() == 1 ):
                self._oddEmitterPin.value( 0 )
                pinChanged = True


        # Use even emitter pin in these cases:
        # - 2 emitter pins, emitters = all
        # - 2 emitter pins, emitters = even
        if (self._emitterPinCount == 2) and (emitters in (EMITTERS_ALL, EMITTERS_EVEN)) :
             # Check if pin is defined and only turn off if not already off
            if ( self._evenEmitterPin != None) and ( self._evenEmitterPin.value() == 1 ):
              self._evenEmitterPin.value( 0 )
              pinChanged = True

        if wait and pinChanged :
            if self._dimmable:
                # driver min is 1 ms
                time.sleep_us(1200)
            else:
                time.sleep_us(200)

    def emittersOn(self, emitters=EMITTERS_ALL, wait: bool=True):
        """Turns the IR LEDs on.

        If the sensors are dimmable and a dimming level is set, this function
        will apply the dimming level after turning the emitters on.

        This function is mainly for use by the read() method. Since read()
        normally turns the emitters on and off automatically for each reading,
        calling this function yourself will not affect the readings unless the
        read mode is READMODE_MANUAL, which tells read() to leave the
        emitters alone.

        Args:
            emitters (optional): Which emitters to turn off. Defaults to EMITTERS_ALL.
            wait (bool, optional): If true, this function delays to give the sensors time to turn off before returning. Otherwise, it returns immediately. Defaults to True.
        """
        assert emitters in (EMITTERS_ALL,EMITTERS_ODD,EMITTERS_EVEN)
        assert self._emitterPinCount > 0
        pinChanged = False
        emittersOnStart = 0 # uint16_t ;
        #DEBUG print( 'eOn.enter' )

        # Use odd emitter pin in these cases:
        # - 1 emitter pin, emitters = all
        # - 2 emitter pins, emitters = all
        # - 2 emitter pins, emitters = odd
        if (emitters == EMITTERS_ALL) or ((self._emitterPinCount == 2) and (emitters == EMITTERS_ODD)):
            # Check if pin is defined, and only turn on non-dimmable sensors if not
            # already on, but always turn dimmable sensors off and back on because
            # we might be changing the dimming level (emittersOnWithPin() should take
            # care of this)
            if (self._oddEmitterPin != None) and ( self._dimmable or (self._oddEmitterPin.value() == 0)):
                emittersOnStart = self.__emittersOnWithPin(self._oddEmitterPin)
                pinChanged = True

        # Use even emitter pin in these cases:
        # - 2 emitter pins, emitters = all
        # - 2 emitter pins, emitters = even
        if (self._emitterPinCount == 2) and ((emitters == EMITTERS_ALL) or (emitters == EMITTERS_EVEN)):
            # Check if pin is defined, and only turn on non-dimmable sensors if not
            # already on, but always turn dimmable sensors off and back on because
            # we might be changing the dimming level (emittersOnWithPin() should take care of this)
            if (self._evenEmitterPin != None) and ((self._dimmable) or (self._evenEmitterPin.value() == 0)):
                emittersOnStart = self.__emittersOnWithPin(self._evenEmitterPin)
                pinChanged = True

        if wait and pinChanged:
            if (emittersOnStart==0) or (emittersOnStart==None): # Debug testing
                emittersOnStart = time.ticks_us()
            if self._dimmable:
                # Make sure it's been at least 300 us since the emitter pin was first set
                # high before returning. (Driver min is 250 us.) Some time might have
                # already passed while we set the dimming level
                _diff = time.ticks_diff( time.ticks_us(), emittersOnStart)
                while _diff < 300 :
                    time.sleep_us(10)
                    _diff = time.ticks_diff( time.ticks_us(), emittersOnStart)
            else: # not dimmable
                time.delay_us(200)
        #DEBUG print( 'eOn.exit' )

    def emittersSelect(self, emitters):
        """Turn on selected emitters and turns off the other emitters
        
        This function turns on the selected emitters while it waits for the
        other emitters to turn off. For example,
        `emittersSelect(EMITTERS_ODD)` turns on the odd-numbered emitters
        while turning off the even-numbered emitters. Using this method avoids
        unnecessary delays compared to calling emittersOff() and emittersOn()
        separately, but it still waits for all emitters to be in the right
        states before returning.

        Args:
            emitters (optional): Which emitters to turn off. Defaults to EMITTERS_ALL.
        """
        assert emitters in (EMITTERS_ALL,EMITTERS_ODD,EMITTERS_EVEN,EMITTERS_NONE)
        offEmitters = None

        if emitters==EMITTERS_ODD:
            offEmitters = EMITTERS_EVEN
        elif emitters==EMITTERS_EVEN:
            offEmitters = EMITTERS_ODD
        elif emitters==EMITTERS_ALL:
            self.emittersOn()
            return
        elif emitters==EMITTERS_NONE:
            self.emittersOff()
            return
        else: # invalid
            return

        # Turn off the off-emitters; don't wait before proceeding, but record the time.
        self.emittersOff( offEmitters, False )
        turnOffStart = time.ticks_us()

        # Turn on the on-emitters and wait.
        self.emittersOn( emitters )

        if self._dimmable:
            # Finish waiting for the off-emitters emitters to turn off: make sure it's been
            # at least 1200 us since the off-emitters was turned off before returning.
            # (Driver min is 1 ms.) Some time has already passed while we waited for
            # the on-emitters to turn on.
            while ticks_diff( time.ticks_us(), turnOffStart ) < 1200:
                time.delay_us(10)
    
    def __calibrateOnOrOff(self, calOn, mode): # CalibrationData & calibration, QTRReadMode mode):
        # Handles the actual calibration, including (re)allocating and
        # initializing the storage for the calibration values if necessary.
        #DEBUG print("calibrateOnOrOff.in")
        for i in range( self._sensorCount ):
            self.__sensorValues[i] = 0
            self.__maxSensorValues[i] = 0
            self.__minSensorValues[i] = 0
        if calOn: # If we do calibrateOn
            calibration = self.calibrationOn
        else:
            calibration = self.calibrationOff

        # (Re)allocate and initialize the arrays if necessary.
        if not calibration.initialized:
            #DEBUG print("calibrateOnOrOff.calibration-init")
            # Looks not used! oldMaximum = list( calibration.maximum ) # Make a copy
            calibration.maximum = [0]*self._sensorCount

            # Looks not used! oldMinimum = list(calibration.minimum) # Make a copy
            calibration.minimum = [self._maxValue]*self._sensorCount
            calibration.initialized = True

        #DEBUG print("calibrateOnOrOff.10*read")
        for j in range( 10 ): # (uint8_t j = 0; j < 10; j++)
            # self.read( sensorValues, mode)# DEBUG NOK
            self.read( mode ) # DEBUG_NOK
            for i in range( self._sensorCount ): # (uint8_t i = 0; i < _sensorCount; i++)
                # set the max we found THIS time
                if (j == 0) or (self.__sensorValues[i] > self.__maxSensorValues[i]) :
                    self.__maxSensorValues[i] = self.__sensorValues[i]
                # set the min we found THIS time
                if (j == 0) or (self.__sensorValues[i] < self.__minSensorValues[i]) :
                    self.__minSensorValues[i] = self.__sensorValues[i]
        #DEBUG print("calibrateOnOrOff.record-min-max")
        # record the min and max calibration values
        for i in range( self._sensorCount ):
            # Update maximum only if the min of 10 readings was still higher than it
            # (we got 10 readings in a row higher than the existing maximum).
            if self.__minSensorValues[i] > calibration.maximum[i]:
                calibration.maximum[i] = self.__minSensorValues[i]

            # Update minimum only if the max of 10 readings was still lower than it
            # (we got 10 readings in a row lower than the existing minimum).
            if self.__maxSensorValues[i] < calibration.minimum[i]:
                calibration.minimum[i] = self.__maxSensorValues[i]
        
        return (calibration.minimum, calibration.maximum)
        
        #gc.collect()
        #DEBUG print("calibrateOnOrOff.Collect+Out")

    def calibrate(self, mode=READMODE_ON):
        """Read the sensor for calibration.
        
        READMODE_MANUAL is not supported

        Args:
            mode (optional): Indicates the emitter behavior during the calibration. Defaults to READMODE_ON.
        """
        assert mode in (READMODE_OFF, READMODE_ON, READMODE_ON_AND_OFF, READMODE_ODD_EVEN, READMODE_ODD_EVEN_AND_OFF, READMODE_MANUAL )

        if mode == READMODE_MANUAL:
            return

        if mode in (READMODE_ON, READMODE_ON_AND_OFF ):
            self.__calibrateOnOrOff( True, READMODE_ON )
        elif mode in (READMODE_ODD_EVEN, READMODE_ODD_EVEN_AND_OFF):
            self.__calibrateOnOrOff( True, READMODE_ODD_EVEN )

        if mode in (READMODE_ON_AND_OFF, READMODE_ODD_EVEN_AND_OFF, READMODE_OFF):
            self.__calibrateOnOrOff( False, READMODE_OFF)

    def resetCalibration(self):
        """Resets all calibration that has been done.
        """
        for i in range(self._sensorCount): #(uint8_t i = 0; i < _sensorCount; i++)
            if self.calibrationOn.maximum != None:
                self.calibrationOn.maximum[i] = 0
            if self.calibrationOff.maximum!= None:
                self.calibrationOff.maximum[i] = 0
            if self.calibrationOn.minimum != None:
                self.calibrationOn.minimum[i] = self._maxValue
            if self.calibrationOff.minimum!= None:
                self.calibrationOff.minimum[i] = self._maxValue
    
    def setCalValues(self, minList: list, maxList: list, calibrationInitialized: bool, CalOn: bool=True):
        """Sets calibration values to user-supplied presets
            minList: list of minimum values per sensor
            maxList: list of maximum values per sensor
            calibrationInitialized: boolean, should always be true
            CalOn: boolean, signifies that calibration was performed with the lights on, not off.
            Returns: nothing
        """
        
        #CalOn tells us if we performed the calibration with the lights on or not
        if CalOn:
            calibration = self.calibrationOn
        else:
            calibration = self.calibrationOff

        #length check - list of cal values should equal the number of sensors in this array
        if not ((len(minList) == self._sensorCount) and (len(maxList) == self._sensorCount)):
            calibration.initialized = False
            raise TypeError("Lengths of sensor calibration value arrays don't match the count of sensors in this array.")
        else:
            for i in range(self._sensorCount):
                calibration.minimum[i] = minList[i]
                calibration.maximum[i] = maxList[i]

            calibration.initialized = calibrationInitialized


    def __readPrivate(self, start = 0, step = 1):
        """Reads the first of every [step] sensors, starting with [start] (0-indexed, so start = 0 means start with the first sensor).
        For example, step = 2, start = 1 means read the *even-numbered* sensors. start defaults to 0, step defaults to 1"""
        if self._sensorPins == None:
            return

        # RC sensor type
        if self.__type == 0:

            for i in range(start, self._sensorCount, step): #(uint8_t i = start; i < _sensorCount; i += step)
                self.__sensorValues[i] = self._maxValue
                # make sensor line an output (drives low briefly, but doesn't matter)
                self._sensorPins[i].init(Pin.OUT)
                # drive sensor line high
                self._sensorPins[i].value( 1 )

            time.sleep_us(10) # charge lines for 10 us

            # disable interrupts so we can switch all the pins as close to the same time as possible
            # noInterrupts()

            # record start time before the first sensor is switched to input
            # (similarly, time is checked before the first sensor is read in the loop below)
            startTime = time.ticks_us()

            for i in range( start, self._sensorCount, step ): # (uint8_t i = start; i < _sensorCount; i += step)
                #make sensor line an input (should also ensure pull-up is disabled)
                self._sensorPins[i].init(Pin.IN)

            # interrupts() # re-enable
            _diff = time.ticks_diff(time.ticks_us(), startTime)
            while _diff < self._maxValue:
                # disable interrupts so we can read all the pins as close to the same
                # time as possible
                #noInterrupts()
                for i in range(start, self._sensorCount, step): # (uint8_t i = start; i < _sensorCount; i += step)
                    if (self._sensorPins[i].value() == 0) and (_diff < self.__sensorValues[i]):
                        # record the first time the line reads low
                        self.__sensorValues[i] = _diff
                _diff = time.ticks_diff(time.ticks_us(), startTime)

            # interrupts() # re-enable
        
        # Analog sensor type
        elif self.__type == 1:

            # set the input pins to analog input and zero the array

            for i in range(start, self._sensorCount, step):
                #self._sensorPins[i].init(Pin.ANALOG)
                self.__sensorValues[i] = 0
            
            for j in range(self._samplesPerSensor):
                for i in range(start, self._sensorCount, step):
                    # add the conversion result
                    #print(self._sensorPins[i])
                    self.__sensorValues[i] += self._sensorADCs[i].read()
            
            # get the rounded average of the readings for each sensor
            for i in range(start, self._sensorCount, step):
                self.__sensorValues[i] = (self.__sensorValues[i]/self._samplesPerSensor)

        else:
            raise ValueError('Invalid sensor type')

    def __readLinePrivate( self, mode, invertReadings): # returns uint16_t, uint16_t * sensorValues, QTRReadMode mode, bool invertReadings
        onLine = False # Sensor is on the line
        avg = 0 # this is for the weighted total
        sum = 0 # this is for the denominator, which is <= 64000

        # manual emitter control is not supported
        if mode == READMODE_MANUAL:
            return 0

        self.readCalibrated(mode)

        for i in range( self._sensorCount ): # (uint8_t i = 0; i < _sensorCount; i++)
            value = self.__sensorValues[i]
            if invertReadings:
                value = 1000 - value

            # keep track of whether we see the line at all
            if value > 200:
                onLine = True

            # only average in values that are above a noise threshold
            if value > 50:
                avg += int(value * i * 1000)
                sum += value

        if not onLine:
            # If it last read to the left of center, return 0.
            if self._lastPosition < (self._sensorCount - 1) * 1000 / 2:
                return 0
            # If it last read to the right of center, return the max.
            else:
                return (self._sensorCount - 1) * 1000

        self._lastPosition = int(avg / sum)
        return self._lastPosition

    def read(self, mode=READMODE_ON):
        """Reads the raw sensor values.

        READMODE_MANUAL is not supported.

        Args:
            mode (optional): Indicates the emitter behavior during the calibration. Defaults to READMODE_ON.
        """
        assert mode in (READMODE_OFF, READMODE_ON, READMODE_ON_AND_OFF, READMODE_ODD_EVEN, READMODE_ODD_EVEN_AND_OFF, READMODE_MANUAL )
        sensorValues = self.__sensorValues
        if mode==READMODE_OFF:
            self.emittersOff()
            self.__readPrivate()
            return
        elif mode==READMODE_MANUAL:
            self.__readPrivate()
            return
        elif mode in ( READMODE_ON, READMODE_ON_AND_OFF ):
            self.emittersOn();
            #DEBUG print( '__readPrivate.enter' )
            self.__readPrivate() # DEBUG NOK
            #DEBUG print( '__readPrivate.exit' )
            self.emittersOff() # DEBUG NOK
        elif mode in ( READMODE_ODD_EVEN , READMODE_ODD_EVEN_AND_OFF ):
            # Turn on odd emitters and read the odd-numbered sensors.
            # (readPrivate takes a 0-based array index, so start = 0 to start with the first sensor)
            self.emittersSelect(EMITTERS_ODD)
            self.__readPrivate( 0, 2)
            # Turn on even emitters and read the even-numbered sensors.
            # (readPrivate takes a 0-based array index, so start = 1 to start with  the second sensor)
            self.emittersSelect(EMITTERS_EVEN)
            self.__readPrivate( 1, 2)
            self.emittersOff()
        else:
            # invalid - do nothing
            return

        if mode in (READMODE_ON_AND_OFF, READMODE_ODD_EVEN_AND_OFF):
            #Take a second set of readings and return the values (on + max - off).
            offValues = [0]*self._sensorCount
            self.__readPrivate(offValues)
            for i in range( self._sensorCount ):
                 sensorValues[i] += self._maxValue - offValues[i]
                 if sensorValues[i] > self._maxValue:
                    # This usually doesn't happen, because the sensor reading should
                     # go up when the emitters are turned off.
                    sensorValues[i] = self._maxValue

    def readCalibrated( self, mode=READMODE_ON ):
        """Reads the sensors and provides calibrated values between 0 and 1000.

        READMODE_MANUAL is not supported.

        Args:
            mode (optional): Indicates the emitter behavior during the calibration. Defaults to READMODE_ON.
        """
        assert mode in (READMODE_OFF, READMODE_ON, READMODE_ON_AND_OFF, READMODE_ODD_EVEN, READMODE_ODD_EVEN_AND_OFF, READMODE_MANUAL )
        #DEBUG print( "readCalibrated.in" )
        sensorValues = self.__sensorValues
        # manual emitter control is not supported
        if mode == READMODE_MANUAL:
            return

        # if not calibrated, do nothing
        if mode in (READMODE_ON,READMODE_ON_AND_OFF,READMODE_ODD_EVEN_AND_OFF):
            if not self.calibrationOn.initialized:
                return

        if mode in (READMODE_OFF, READMODE_ON_AND_OFF, READMODE_ODD_EVEN_AND_OFF):
            if not self.calibrationOff.initialized:
                return

        # read the needed values
        self.read( mode )

        for i in range( self._sensorCount ):
            calmin = self._maxValue
            calmax = 0

            # find the correct calibration
            if mode in (READMODE_ON,READMODE_ODD_EVEN):
                calmax = self.calibrationOn.maximum[i]
                calmin = self.calibrationOn.minimum[i]
            elif (mode == READMODE_OFF):
                calmax = self.calibrationOff.maximum[i]
                calmin = self.calibrationOff.minimum[i]
            else: # READMODE_ON_AND_OFF, READMODE_ODD_EVEN_AND_OFF
                if self.calibrationOff.minimum[i] < self.calibrationOn.minimum[i]:
                    # no meaningful signal
                    calmin = self._maxValue
                else:
                    # this won't go past _maxValue
                    calmin = self.calibrationOn.minimum[i] + self._maxValue - self.calibrationOff.minimum[i]

                if self.calibrationOff.maximum[i] < self.calibrationOn.maximum[i]:
                    # no meaningful signal
                    calmax = self._maxValue
                else:
                    #this won't go past _maxValue
                    calmax = self.calibrationOn.maximum[i] + self._maxValue - self.calibrationOff.maximum[i];

            denominator = calmax - calmin
            value = 0
            if denominator != 0:
                value = int((sensorValues[i] - calmin) * 1000 / denominator)

            if value < 0:
                value = 0
            elif value > 1000:
                value = 1000

            sensorValues[i] = value
        #gc.collect()
        #DEBUG print( "readCalibrated.out" )

    def readLineBlack( self, mode=READMODE_ON ):
        """Reads the sensors, provides calibrated values, and returns an
        estimated black line position.

        The estimate is made using a weighted average of the sensor indices
        multiplied by 1000, so that a return value of 0 indicates that the line
        is directly below sensor 0, a return value of 1000 indicates that the
        line is directly below sensor 1, 2000 indicates that it's below sensor
        2000, etc. Intermediate values indicate that the line is between two
        sensors. The formula is (where v_0 represents the value from the
        first sensor):
    
        ((0*v_0) + (1000*v_1) + (2000*v_2) + ...) / (v_0 + v_1 + v_2 + ...)
    
        As long as your sensors aren't spaced too far apart relative to the
        line, this returned value is designed to be monotonic, which makes it
        great for use in closed-loop PID control. Additionally, this method
        remembers where it last saw the line, so if you ever lose the line to
        the left or the right, its line position will continue to indicate the
        direction you need to go to reacquire the line. For example, if sensor
        4 is your rightmost sensor and you end up completely off the line to
        the left, this function will continue to return 4000.
    
        This function is intended to detect a black (or dark-colored) line on a
        white (or light-colored) background. For a white line, see
        readLineWhite().

        Args:
            mode (optional): Indicates the emitter behavior during the calibration. Defaults to READMODE_ON.

        Returns:
            int: An estimate of the position of a black line under the sensors.
        """
        return self.__readLinePrivate( mode, False )
    
    def readLineWhite( self, mode=READMODE_ON ):
        """Reads the sensors, provides calibrated values, and returns an
        estimated white line position.

        The estimate is made using a weighted average of the sensor indices
        multiplied by 1000, so that a return value of 0 indicates that the line
        is directly below sensor 0, a return value of 1000 indicates that the
        line is directly below sensor 1, 2000 indicates that it's below sensor
        2000, etc. Intermediate values indicate that the line is between two
        sensors. The formula is (where v_0 represents the value from the
        first sensor):
    
        ((0*v_0) + (1000*v_1) + (2000*v_2) + ...) / (v_0 + v_1 + v_2 + ...)
    
        As long as your sensors aren't spaced too far apart relative to the
        line, this returned value is designed to be monotonic, which makes it
        great for use in closed-loop PID control. Additionally, this method
        remembers where it last saw the line, so if you ever lose the line to
        the left or the right, its line position will continue to indicate the
        direction you need to go to reacquire the line. For example, if sensor
        4 is your rightmost sensor and you end up completely off the line to
        the left, this function will continue to return 4000.
    
        This function is intended to detect a white (or light-colored) line on a
        black (or dark-colored) background. For a black line, see
        readLineBlack().

        Args:
            mode (optional): Indicates the emitter behavior during the calibration. Defaults to READMODE_ON.

        Returns:
            int: An estimate of the position of a black line under the sensors.
        """
        self.__readLinePrivate( mode, True )

    def sensorPins( self ):
        return self._sensorPins

    def emitterPin( self ): # When only one Pin to control them all (it is the odd reference)
        # Assigned only at creation time
        return self._oddEmitterPin

    def oddEmitterPin( self ):
        # Assigned only at creation time
        return self._oddEmitterPin
    
    def evenEmitterPin( self ):
        # Assigned only at creation time
        return self._evenEmitterPin
    
    def emitterPinCount( self ):
        return self._emitterPinCount

    
    def timeout( self ):
        # recommended value between 1000 & 3000 uSec
        return self._maxValue
    def timeout( self, value ):
        assert 0<=value<=5000
        self._maxValue = value

    def dimmable(self):
        return self._dimmable
    def dimmable(self, value):
        self._dimmable = value

    
    def dimmingLevel(self):
        return self._dimmingLevel
    def dimmingLevel( self, value ):
        assert 0<=value<=31
        self._dimmingLevel = value
    
