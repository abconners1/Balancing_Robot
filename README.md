# Balancing Robot with MPU6050 and PID Control
Demonstration: 
https://youtu.be/XPemdZGGN2E

This project implements a balancing robot using an MPU6050 6-axis gyroscope and accelerometer and a PID control algorithm. The robot uses the accelerometer data to calculate the tilt angle and adjusts the motor speed and direction to maintain balance.

## Table of Contents
1. [Overview](#overview)
2. [Hardware Requirements](#hardware-requirements)
3. [Software Requirements](#software-requirements)
4. [Setup and Configuration](#setup-and-configuration)
5. [Code Explanation](#code-explanation)
6. [PID Tuning](#pid-tuning)
7. [Future Improvements](#future-improvements)
8. [Acknowledgments](#acknowledgments)

## Overview

Balancing robots demonstrate the principles of inverse pendulum mechanics and PID control. This project reads the tilt angle from the MPU6050 sensor and uses a PID algorithm to drive a motor to correct the tilt and keep the robot upright.

### Features
- **PID Control:** Adjust proportional (P), integral (I), and derivative (D) gains to achieve stability.
- **Real-Time Feedback:** Debugging information displayed via serial communication.
- **Modular Design:** Easily adaptable for additional sensors or mobility.

## Hardware Requirements
- MPU6050 IMU sensor
- Arduino-compatible microcontroller
- DC motor with PWM control
- Motor driver circuit
- Power source for motors and microcontroller
- Supporting mechanical structure (e.g., chassis, wheels)

## Software Requirements
- Arduino IDE
- Required Libraries:
  - `Wire.h` (for I2C communication)
  - `I2Cdev.h` (for MPU6050 communication)
  - `MPU6050.h` (for MPU6050 sensor data processing)

## Setup and Configuration

1. **Connect the MPU6050 Sensor:**
   - VCC -> 3.3V or 5V (depending on your MPU6050 module)
   - GND -> GND
   - SCL -> Arduino A5 (on most boards)
   - SDA -> Arduino A4 (on most boards)

2. **Connect the Motor Driver:**
   - Motor control pins should connect to the defined `motorRightDirection` and `motorRightPWM` pins in the code.

3. **Install Required Libraries:**
   - Add `I2Cdev` and `MPU6050` libraries to your Arduino IDE.

4. **Upload the Code:**
   - Compile and upload the code to your Arduino.

5. **Power Up:**
   - Connect power to the motors and Arduino.

## Code Explanation

### Key Components
- **MPU6050 Initialization:**
  Reads accelerometer and gyroscope data to determine the tilt angle.

- **PID Control:**
  Calculates the error between the desired angle and the measured angle and adjusts motor speed and direction.

- **Motor Control:**
  Uses PWM to control motor speed based on the PID output.

### Constants
- `Kp`, `Ki`, `Kd`: PID tuning parameters.
- `UMAX`: Maximum allowable PID output to prevent excessive correction.
- `desiredAngle`: The target angle for the robot to maintain balance.

## PID Tuning

To achieve balance:
1. Start with small `Kp` values and incrementally increase until the robot starts correcting itself.
2. Gradually increase `Ki` to address steady-state errors.
3. Add small values for `Kd` to dampen oscillations.
4. Use the serial output to monitor errors and PID values for adjustments.

### Example Tuning Values
- Proportional (Kp): 1
- Integral (Ki): 0.001
- Derivative (Kd): 0.001

You may need to experiment with these values to achieve optimal stability.

## Future Improvements

- **Add Mobility:** Implement control for forward and backward motion.
- **Sensor Fusion:** Use both accelerometer and gyroscope data to calculate tilt angle more accurately.
- **Real-Time Adjustments:** Add potentiometers for dynamic PID tuning during operation.
- **Stabilization Against Disturbances:** Improve robustness against pushes or uneven surfaces.

## Acknowledgments

This project was inspired by the principles of inverse pendulum balancing and utilized libraries and insights from the Arduino community. The challenges and lessons learned were documented in the memo for Lab #7 by Andrew Conners and Yer Yang, Team 408.

