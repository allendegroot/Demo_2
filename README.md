# Demo 2

## Directory Structure

The CV code can be found quite easily in the `cv_class.py` file. This cna be found in the home directory. The code used for the Arduino is composed of a few files all of which can be found in the `Arduino Code` folder which is alos in the home directory. Each of these will be spoken about a little more below.

### cv_class.py

This file contians a class that runs a loop which constantly takes a picture and searches for an Aruco. Given that it finds an Aruco, it then calculates the angle and the distance to that Aruco. At the end of each loop iteration it also sends a signal down an I2C channel to the Arduino. There are a few signals it can send.
* It will send 255 if it does not sense an Arduino. This is to tell the Arduino to keep rotating until it finds one.
* It will send the angle if it sees the Aruco.
* Once the Arduino starts sending back a 2, the Pi will start sending the distance.

Each of these are converted to an 8 bit value which is then converted back on the Arduino side. This is the core functionality of the CV code.

### Arduino Code

This code can be found in the `Arduino Code` folder. Descriptions as to each file's purpose adn usage can be found there.

### Grho_transferFn_graphing.m

This is the MATLAB script that is used to determine a transfer function for the linear velocity of the robot. It compares data (in graphical form) to a simple Simulink diagram to produce the transfer function for the robot. This file was initially designed for demo 1 but there were some small tweaks made for this demonstration.

### linvelocity_closedLoop.slx

This is the Simulink diagram that was used to tune the closed loop response for the linear distance controller. This involves two individual controllers. The inner one takes the desired velocity for the robot and converts it to a PWM value that should be sent to the motors. The other controller takes the difference between the current and desired positions to determine what the velocity of the robot should be. This file was originally developed for demo 1, but some small tweaks and retuning had to be done for this demo.


