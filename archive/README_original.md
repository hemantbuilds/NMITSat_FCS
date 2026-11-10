# NMITSat_FCS
 flight control software for the cansat was uploaded in the teensy4.1 board as flight controller
Overview
CANSAT OBC ( Teensy 4.1 ) gathers all the sensor data , saves to SD card and communicates through Ground Station via the XBEE module.
A bonus camera is also included on the OBC which will record the flight and save it to SD card.

Programming language :- C++ 
Development Environment :- Arduino IDE
Algorithms
Electronic System is activated via a switch
Container Calibrate Command is received from GCS
Camera activation Command is received from GCS.
Container Transmit Command is received from the GCS.
Container saves sensor data to a sd card and sends it to GCS via Xbee.
Container saves camera data to a sd card.
Second Parachute is triggered when apogee reaches 600mt and is deployed at 500mt.
The buzzer will keep beeping after landing until the power button is turned off.
