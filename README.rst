BMI160 6-Axis Fusion
##########################

.. contents::
   :local:
   :depth: 2

This code uses BoschSensortech's BMI160 or BMX160 sensors to calculate quaternion vectors 
It uses accelerometer and gyroscope data to calculate roll and pitch angles

Requirements
************

* nRF52832 Sparkfun Breakout Board
* Segger J-Link Edu-Mini
* BoschSensortech BMI160 or BMX160
* Power Source

Overview
********
The orientation is calculated as a quaternion that rotates the gravity vector from earth frame to sensor frame. The gravity vector in the sensor frame is the accelerometer readings and the gravity vector in earth frame is (0,0,-1).

The accelerometer values are sensitive to vibrations. The gyroscope is used to keep track of the gravity vector and correct the accelerometer readings.



Usage
********
To use the library, read the accelerometer, gyro values, and calculate the time taken to complete a loop.

```C
/*
fused_vector - corrected accelerometer readings
delta - delay or time taken to complete a loop
wx,wy,wz - gyro values in rad/s
ax, ay, az - raw accelerometer values
q_acc - quaternion representing orientation
angles - euler angles
*/

delta = 0.001*(millis()-Start);
fused_vector = update_fused_vector(fused_vector,ax,ay,az,wx,wy,wz,delta);
  
q_acc = quaternion_from_accelerometer(fused_vector.a,fused_vector.b,fused_vector.c);
angles = quaternion_to_euler_angles(q_acc);
```
Note: To calculate correct delta please synchronize sensor readings with the software.

Development kits
================



Building and running
********************



Testing
=======





