Results
=======

.. contents:: Contents
   :local:
   :depth: 3

Overview
********
Our Romi was able to complete the course multiple times but always. The times it failed mostly we’re due to line mis-detections, or cases when the QTR chassis got close enough to the field edge to interpret it as a line.

Discussion
**********
The line sensor array was originally designed for high-speed PID line following, so it was far in front of the center of rotation of the chassis so it would be sensitive to rotation. When switching to an effective P-only control scheme, this meant that the line sensors would nominally fall off the line occasionally, which would make them vulnerable to detecting small blemishes on the game field as lines. The surface finish of the field was glossy, too, which made this problem more common than when on the original testing fields. To fix this issue, we could further investigate the exact settings and behavior of the line sensors, or implement an algorithm that changes the robot behavior when it is off the line for an extended period of time. The implementation of two sensor arrays inline, however, was very successful. The dialogue between the two sensor array readings allowed Romi to handle the courses dashed and hashed lines. The IMU and ToF sensors were also successful, though it was crucial that no obstacles were level with the robot’s field of view when testing due to the ToF’s performance.

Video
*****

.. youtube:: JfRuApvF66k