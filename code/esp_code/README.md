# MicroMouse ESP Code

## Table of Contents
- [Introduction](#introduction)
- [Getting Started](#getting-started)
    - [Turning on the Device](#turning-on-the-device)
    - [Programming the Device](#programming-the-device)

## Introduction
Welcome to the MicroMouse ESP Code repository! This repository contains the code for controlling the MicroMouse robot using an ESP microcontroller.

## Getting Started
To get started with the MicroMouse robot, follow the steps below.

### Turning on the Device
To turn on the MicroMouse robot, follow these steps:
1. Connect the power source to the microcontroller.
2. DON'T connect battery and programmer in same time.
3. Press the button to turn on the device.
4. Wait for the device to boot up and initialize.

### Programming the Device
To program the MicroMouse robot, follow these steps:
1. Connect the microcontroller to your computer using a USB cable.
2. Write or modify the code as needed (in task_control call function for initing controller, additional filters..).
3. Upload the code to the microcontroller(on linux idf.py -p /dev/ttyUSBx flash) after build press button on mouse to get into download mode (same on windows) or download esptool-ftdi.py wrapper.
4. Disconnect the microcontroller from the computer.

That's it! You're now ready to start using and programming the MicroMouse robot.

## Contributing
If you'd like to contribute to this project, please follow the guidelines outlined in the [CONTRIBUTING.md](CONTRIBUTING.md) file.

## License
This project is licensed under the [MIT License](LICENSE).