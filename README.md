# Rover Motor Control

This program controls an ESP32 for use in controlling a rover. The ESP should be connected to a computer (such as an Nvidia Jetson Nano) via micro-USB. It uses incremental control to maintain the set point velocity for each individual motor. The ESP uses UART to commuinicate with the Jetson Nano.

## How to Use Visual Studio Code for ESP32 Development
The Visual Studio Code environment can be set up for ESP32 devlopment by following this tutorial: https://www.youtube.com/watch?v=Lc6ausiKvQM

When configuring the ESP-IDF extension, make sure to select a version of the ESP-IDF that includes ```pulse_cnt.h``` and ```gptimer.h```. At the time of writing, these files are only included in the "master branch" installation option. If any releases are published beyond 4.4, you can check for the two files using the "Go to file" tool in the [repository](https://github.com/espressif/esp-idf)

If you are creating a new project, you can use the examples to set up the project as mentioned in the video. 

If you are going to use/modify the Rover Motor Control program (https://github.com/willish32/roverMotorTesting) then follow the following steps once you have the Visual Studio Code environment set up
- Press 'CTRL + `' to pull up the powershell terminal in VSCode
- Type in "git clone https://github.com/willish32/roverMotorTesting.git"
- As mentioned in the video, you can build, flash, and monitor the ESP using the command pallete or the buttons at the bottom of VSCode


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
- 
