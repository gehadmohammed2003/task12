# task12
# PID Motor Speed Control
This project demonstrates how to control the speed of a DC motor using a PID controller in an Arduino environment. The motor's speed is measured using an encoder, and the PID controller adjusts the motor's PWM to maintain the desired speed.

## Components:
- Arduino
- DC Motor
- Encoder (600 PPR)
- Motor Driver

## Code Overview:
- `PID_v1.h`: A library used to implement PID control.
- Motor speed is adjusted based on feedback from an encoder.
- The PID parameters (Kp, Ki, Kd) can be tuned for optimal performance.
- Exponential smoothing is applied to the setpoint for smoother control.

## Wiring:
- Motor PWM output pin: `motorPin = 9`
- Encoder signal input pin: `encoderPin = 2`

## Setup Instructions:
1. Upload the code to your Arduino.
2. Ensure correct wiring between the motor, encoder, and Arduino.
3. Adjust the PID parameters (`Kp`, `Ki`, `Kd`) as needed for your specific motor.

## Usage:
- The setpoint for the motor speed is set to `100 RPM` initially.
- The code calculates the motor speed every second and adjusts the PWM output accordingly.
- Open the Serial Monitor to view real-time debugging information.

## Libraries:
- Install the [PID_v1](https://playground.arduino.cc/Code/PIDLibrary/) library for Arduino.



