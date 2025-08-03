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

- `firmware/` – Embedded code for STM32
- `python-ui/` – Python script for radar-style display
- `README.md` – Project description and instructions
