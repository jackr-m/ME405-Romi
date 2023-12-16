"""Houses the LineFollower task class.
"""

from pyb import Pin
from task_share import Share
from ClosedLoop import ClosedLoop
from math import atan, degrees

class LineFollower:
    
    def __init__(self, s_pos: Share, leftTargetSpeed: Share, rightTargetSpeed: Share, s_xCoord: Share, s_yCoord: Share, heading: Share, s_tofdistance: Share, linePosTarget_front: int = 3000, linePosTarget_rear: int = 2000, centerSpeed: float = 2, taskperiod: int = 30):
        """this is a line follower, it follows lines.
        
        Takes sensor data from 3 adjacent tasks: QTRs, IMU, and TOF
        Also takes distance traveled data from the now singular motor task and combines that with the IMU data to integrate position
        Commands motor task through requested speed shares

        Args:
            s_pos: Share object holding composite QTR position values
            leftTargetSpeed: Share object conveying desired left motor speed
            rightTargetSpeed: Share object conveying desired right motor speed
            s_xCoord (Share): Share object conveying mm left wheel NET distance traveled
            s_yCoord (Share): Share object conveying mm right wheel NET distance traveled
            heading: Share object conveying IMU heading
            s_tofdistance: Share object conveying TOF distance for wall detection
            linePosTarget_front: integer QTR position target for front sensor
            linePosTarget_rear: integer QTR position target for rear sensor
            centerSpeed (float): sets the desired base speed for the motors, determines how fast romi travels (rad/s)
            taskperiod (int): period the task runs at (milliseconds)
        """

        self._pos = s_pos

        self._leftSpeedTarget = leftTargetSpeed
        self._rightSpeedTarget = rightTargetSpeed

        self._linePosTarget_front = linePosTarget_front
        self._linePosTarget_rear = linePosTarget_rear
        
        self._centerSpeed = centerSpeed
        self._efforts = []
        self._motorSpeedOffset = 0
        self._taskperiod = taskperiod
        self._state = 0
        
        self._tofdistance = s_tofdistance

        self._circleArcLength = 24 * 25.4 + 20 # D*pi, 25.4 to convert to mm from in
        self._xCoord = s_xCoord
        self._yCoord = s_yCoord
        self._heading = heading
        self.user_button = Pin(Pin.cpu.C13, mode=Pin.IN)

        #positions recorded immediately when romi gets to wall
        self._wallStartX = 0
        self._wallStartY = 0

        self._headingPID = ClosedLoop(0.03, 0.012, 0.005, -((2^32)-1), ((2^32)-1))

    def run(self):
        """Implementation as a generator functions

        Yields:
            int: machine state
        """

        while True:
            if self._state == 0:

                # Get starting heading (this assumes the robot starts pointed straight)
                self._straightHeading = self._heading.get()
                
                if self._straightHeading > 180:
                    self._straightHeading -= 360

                self._state = 1

            # Normal line following, has not yet reached a wall
            elif self._state == 1:                

                #print(pos)
                #print("Distance: {:.0f} mm".format(self._tofdistance.get()))
                #print("Heading:  {:.2f} deg".format(self._heading.get()))

                # If a lot to the right of the line
                if self._pos.get() > 5300:
                    self._motorSpeedOffset = -self._centerSpeed/1.5
                # If a little to the right of the line
                elif self._pos.get() > 4000:
                    self._motorSpeedOffset = -self._centerSpeed/1.85
                # If a lot to the left of the line
                elif self._pos.get() < 700:
                    self._motorSpeedOffset = self._centerSpeed/1.5
                # If a little to the left of the line
                elif self._pos.get() < 2000:
                    self._motorSpeedOffset = self._centerSpeed/1.85
                # If around the center of the line
                else:
                    self._motorSpeedOffset = 0

                self._leftSpeedTarget.put(self._centerSpeed+self._motorSpeedOffset)
                self._rightSpeedTarget.put(self._centerSpeed-self._motorSpeedOffset)

                #print("X{:.2f}, Y{:.2f}".format(self._xCoord.get(), self._yCoord.get()))

                if self._tofdistance.get() <= 100: # mm
                    self._leftSpeedTarget.put(0)
                    self._rightSpeedTarget.put(0)
                    print("DISTANCE MET")
                    print("X{:.2f}, Y{:.2f}".format(self._xCoord.get(), self._yCoord.get()))
                    self._state = 2
                    self._wallStartX = self._xCoord.get()
                    self._wallStartY = self._yCoord.get()

            # Has hit wall
            elif self._state == 2:

                #print("STATE 2")
                heading_corrected = self._heading.get()

                if self._heading.get() > 180:
                    heading_corrected = self._heading.get() - 360

                # if heading is within +/- 10 deg of target, move to next state
                if (self._straightHeading + 80) <= heading_corrected <= (self._straightHeading + 100):
                    self._state = 3
                
                self._headingPID.setTarget(self._straightHeading + 90)

                self._efforts = self._headingPID.calculateEfforts(heading_corrected, self._taskperiod)
            
                self._motorSpeedOffset = round(sum(self._efforts), 0)

                self._leftSpeedTarget.put(self._motorSpeedOffset)
                self._rightSpeedTarget.put(-self._motorSpeedOffset)


                #print("Heading: {} deg".format(self._heading.get()))
                    
            elif self._state == 3:
                #print("STATE 3")
                heading_corrected = self._heading.get()

                if self._heading.get() > 180:
                    heading_corrected = self._heading.get() - 360
                
                if self._yCoord.get() <= (self._wallStartY - 230):
                    self._state = 4
                
                self._headingPID.setTarget(self._straightHeading + 90)

                self._efforts = self._headingPID.calculateEfforts(heading_corrected, self._taskperiod)
            
                self._motorSpeedOffset = round(sum(self._efforts), 0)

                self._leftSpeedTarget.put(0.8+self._motorSpeedOffset)
                self._rightSpeedTarget.put(0.8-self._motorSpeedOffset)

            elif self._state == 4:
                #print("STATE 4")
                heading_corrected = self._heading.get()

                if self._heading.get() > 180:
                    heading_corrected = self._heading.get() - 360
                
                if self._xCoord.get() >= (self._wallStartX + 700):
                    self._state = 5
                
                self._headingPID.setTarget(self._straightHeading)

                self._efforts = self._headingPID.calculateEfforts(heading_corrected, self._taskperiod)
            
                self._motorSpeedOffset = round(sum(self._efforts), 0)

                self._leftSpeedTarget.put(0.8+self._motorSpeedOffset)
                self._rightSpeedTarget.put(0.8-self._motorSpeedOffset)

            # Getting back on the line
            elif self._state == 5:
                #print("STATE 5")
                heading_corrected = self._heading.get()

                if self._heading.get() > 180:
                    heading_corrected = self._heading.get() - 360
                
                if self._yCoord.get() >= (self._wallStartY - 170):
                    self._state = 7
                
                self._headingPID.setTarget(self._straightHeading - 90)

                self._efforts = self._headingPID.calculateEfforts(heading_corrected, self._taskperiod)
            
                self._motorSpeedOffset = round(sum(self._efforts), 0)

                self._leftSpeedTarget.put(0.8+self._motorSpeedOffset)
                self._rightSpeedTarget.put(0.8-self._motorSpeedOffset)

            # Normal line following, after it has reached a wall
            elif self._state == 7:                

                #print(pos)
                #print("Distance: {:.0f} mm".format(self._tofdistance.get()))
                #print("Heading:  {:.2f} deg".format(self._heading.get()))

                # If a lot to the right of the line
                if self._pos.get() > 5300:
                    self._motorSpeedOffset = -self._centerSpeed/1.5
                # If a little to the right of the line
                elif self._pos.get() > 4000:
                    self._motorSpeedOffset = -self._centerSpeed/1.85
                # If a lot to the left of the line
                elif self._pos.get() < 700:
                    self._motorSpeedOffset = self._centerSpeed/1.5
                # If a little to the left of the line
                elif self._pos.get() < 2000:
                    self._motorSpeedOffset = self._centerSpeed/1.85
                # If around the center of the line
                else:
                    self._motorSpeedOffset = 0

                self._leftSpeedTarget.put(self._centerSpeed+self._motorSpeedOffset)
                self._rightSpeedTarget.put(self._centerSpeed-self._motorSpeedOffset)

                #print("X{:.2f}, Y{:.2f}".format(self._xCoord.get(), self._yCoord.get()))

                if self._xCoord.get() <= -1100: # mm
                    self._leftSpeedTarget.put(0)
                    self._rightSpeedTarget.put(0)
                    print("DONE WITH WALL")
                    print("X{:.2f}, Y{:.2f}".format(self._xCoord.get(), self._yCoord.get()))
                    self._state = 8

            # Hits the finish line
            elif self._state == 8:
                self._leftSpeedTarget.put(0)
                self._rightSpeedTarget.put(0)
                self._state = 9
            
            # Turn around to face the start
            elif self._state == 9:
                heading_corrected = self._heading.get()

                if self._heading.get() > 180:
                    heading_corrected = self._heading.get() - 360
                
                # if heading is within +/- 10 deg of target, move to next state
                if (self._straightHeading - 10) <= heading_corrected <= (self._straightHeading + 10):
                    self._state = 10
                
                self._headingPID.setTarget(self._straightHeading)

                self._efforts = self._headingPID.calculateEfforts(heading_corrected, self._taskperiod)
            
                self._motorSpeedOffset = round(sum(self._efforts), 0)

                self._leftSpeedTarget.put(self._motorSpeedOffset)
                self._rightSpeedTarget.put(-self._motorSpeedOffset)
            
            # Going back to the start line
            elif self._state == 10:
                heading_corrected = self._heading.get()

                if self._heading.get() > 180:
                    heading_corrected = self._heading.get() - 360
                
                if self._xCoord.get() >= (-15):
                    print("DONE!!!")
                    print("X{:.2f}, Y{:.2f}".format(self._xCoord.get(), self._yCoord.get()))
                    self._state = 11
                
                #self._headingPID.setTarget(degrees(atan((-self._yCoord)/(-self._xCoord))))
                self._headingPID.setTarget(self._straightHeading)

                self._efforts = self._headingPID.calculateEfforts(heading_corrected, self._taskperiod)
            
                self._motorSpeedOffset = round(sum(self._efforts), 0)

                self._leftSpeedTarget.put(0.8+self._motorSpeedOffset)
                self._rightSpeedTarget.put(0.8-self._motorSpeedOffset)

            elif self._state == 11:
                self._leftSpeedTarget.put(0)
                self._rightSpeedTarget.put(0)               

            yield self._state