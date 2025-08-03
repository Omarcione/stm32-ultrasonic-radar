# Radar Sweep Visualization System

This project uses an STM32 Nucleo board to control a radar-style system that sweeps an ultrasonic sensor with a servo motor and visualizes distance data. Output can be streamed via UART to a Python-based interface.

## Features

- Servo-controlled sweep from 0° to 180° and back
- Distance measurement with HC-SR04 ultrasonic sensor
- UART serial stream to PC (Python visualization)
- Modular code design for sensing, control, and display

## Hardware Components

- STM32 Nucleo-L476RG
- SG90 Servo Motor
- HC-SR04 Ultrasonic Sensor
- 5V external power supply for servo
- Breadboard and jumper wires

## Project Structure

stm32-ultrasonic-radar/
├── README.md
├── firmware/
│   ├── ... (build files)
│   ├── UltraSonic_Radar-316Final.ioc
│   ├── Debug/
│   ├── Release/
│   ├── Core/
│   │   ├── Inc/
│   │   │   └── ... (header files)
│   │   ├── Src/
│   │   │   └── ... (source files)
│   │   └── Startup/
│   ├── Drivers/
│   │   └── CMSIS/
│   │       └── ... (CMSIS files)
│   └── .settings/
│       └── ... (IDE settings)
├── python-ui/
│   └── Radar.py
