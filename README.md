# Rover Motor Control

This program controls an ESP32 for use in controlling a rover. The ESP should be connected to a computer (such as an Nvidia Jetson Nano) via micro-USB. It uses incremental control to maintain the set point velocity for each individual motor.

## Hardware Required

- Espressif ESP32-WROOM-32D DevkitC
- Six (6) independent motor drivers
- Six (6) independent optical rotary encoders (powered by an external battery)

## Inputs

- Twelve (12) rotary encoder inputs (pulse A and B for each encoder) via I/O pins
- Two (2) revolution per minute (RPM) set points, one for each side, via UART of float type

## Outputs

- Six (6) pulse width modulation (PWM) control signals via I/O pins
- Six (6) direction control pins via I/O pins
- Six (6) encoder averages in revolutions per minute (RPM) separated by commas and terminated with a '\n' via UART of char type
