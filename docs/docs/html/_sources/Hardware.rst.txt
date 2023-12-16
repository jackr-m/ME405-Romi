Hardware
========

Description of the hardware of the robot.

.. contents:: Contents
   :local:
   :depth: 3

Base Chassis
************
The chassis is a Pololu Romi (`<https://www.pololu.com/category/202/romi-chassis-and-accessories>`_).
We are using the Motor Driver and Power Distribution Board for the Romi Chassis (`<https://www.pololu.com/product/3543>`_) to get regulated voltage from 6xLR6 batteries, and drive the 2 DC motors with the included TI DRV8838 motor drivers.

.. image:: https://a.pololu-files.com/picture/0J7351.1200.jpg
    :alt: Romi chassis with motor driver/power distribution board and encoders
    :width: 400

Processor
*********
This robot uses a Nucleo-L476RG, which has an STM32L476RG from ST Microelectronics.  It is running Micropython, using the ``firmware.bin`` binary provided at `<https://github.com/spluttflob/ME405-Support/>`_.
It is being used in conjuction with a Shoe of Brian (`<https://spluttflob.github.io/ME405-Support/shoe_info.html>`_) to provide a native USB port for use with Micropython code.

Sensors
*******
We are using the rotary encoder kit for the chassis (`<https://www.pololu.com/product/3542>`_) for our positional feedback.
In combination with a Bosch BNO055 Inertial Measurement Unit (IMU), we can get a relatively accurate (+/-1mm/minute) position using dead-reckoning of encoder distance traveled in the current IMU heading.

For line following, we are using Pololu QTR infrared reflectance sensors (`<https://www.pololu.com/category/123/pololu-qtr-reflectance-sensors>`_).
Specifically, we are using one QTR-MD-07A and QTR-MD-05A.  They are aligned (see Bottom view photo below) horizontally respect to the Y axis, but placed spaced out in X at a distance such that one of the sensor arrays will always be on a dashed line.  The mount for this is 3D printed from PLA, and is viewable on the CAD file link below.

To avoid the wall in the middle of the photo, we are using a ST Microelectronics VL53L1X time-of-flight sensor on a carrier board from Pololu (`<https://www.pololu.com/product/3415>`_).  This is attached to the front of the robot.

Wiring/Pinouts
**************
Here is a diagram and table that explains how each of the sensors are connected to the Nucleo development board.

.. image:: photos/Pin_Labels.jpg


And this explains the pin labels:

.. image:: photos/Pin_Descriptions.jpg

Coordinate System
*****************
We are using a standard 3-axis coordinate system, where the front of the robot points in the positive X direction, and the central pivot axis is the positive Z axis.
Here is a descriptive photo of the coordinate system of the track:

.. image:: photos/Field_Coordinates.svg
    :alt: Field Coordinates

CAD
***
For CAD reference (ex. reflectance sensor mount), please use the Onshape link:

`Onshape <https://cad.onshape.com/documents/fe4992c9b8e7b4310ed844c4/w/868ce3460770ffc11594e3ba/e/b2cd443f6a4a955c885b54f9?renderMode=0&uiState=657d1e90ccc43861a4264aac>`_

Photos
******

.. image:: photos/DSC02064.jpg
    :alt: Isometric view

.. image:: photos/DSC02069.jpg
    :alt: Top view

.. image:: photos/DSC02067.jpg
    :alt: Bottom view

    
.. image:: photos/DSC02070.jpg
    :alt: Top view in starting position


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
