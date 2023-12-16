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

Processor
*********
This robot uses a Nucleo-L476RG, which has an STM32L476RG from ST Microelectronics.  It is running Micropython, using the ``firmware.bin`` binary provided at `<https://github.com/spluttflob/ME405-Support/>`_.
It is being used in conjuction with a Shoe of Brian (`<https://spluttflob.github.io/ME405-Support/shoe_info.html>`_) to provide a native USB port for use with Micropython code.

Sensors
*******
We are also using the rotary encoder kit for the chassis (`<https://www.pololu.com/product/3542>`_) for our positional feedback.

Photos
******
Insert here

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
