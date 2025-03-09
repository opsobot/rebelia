Firmware for Rebelia - An Open Source Robotic Hand for Humans and Robots
===========================================================


Overview
========

This folder contains the firmware for the Robotic Hand Project, responsible for controlling the servomotors and processing commands for hand movements. The firmware is designed to work seamlessly with the open-source hardware of the robotic hand.

The firmware is licensed under the GNU General Public License v3 (GPLv3), ensuring that any modifications or derivatives remain open-source and freely available to the community.

Structure
=========

```
/firmware
  ├── src/               # Source code for the firmware
  ├── include/           # Header files (.h)
  ├── LICENSE            # Full text of the GPLv3 license
  ├── README.md          # This document
  └── Makefile           # Build instructions (optional, if applicable)
```

Licensing
=========

The firmware in this folder is released under the GNU General Public License v3 (GPLv3).

By using or modifying this firmware, you agree to:

Attribute the original authors.

Share any modified versions of this firmware under the same GPLv3 license.

Include a copy of the GPLv3 license with any distribution of this firmware.

You can find the full license text in the LICENSE.txt file.


Copyright Notice
================

Each source file contains a copyright notice similar to the following:

```
/*
 * Copyright (C) 2025 Your Name / Your Company
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */
```

Dependencies
============

To use this firmware with the Arduino IDE, you must install the following libraries:

* SCServo by FT&WS
* BluetoothSerial by Henry Abrahansen
* FFT by Robin Scheibler 

TODO
====
1. Install  Arduino IDE 2
2. Open Arduino IDE 2
3. Open RebeliaHuman sketch
4. Open Boards Manager
	1. Install esp32 by Expressif Systems
5. Open Library Manager
	1. Search and install SCServo by FT&WS
	2. Download https://github.com/yash-sanghvi/ESP32/files/6474828/ESPfft.zip and save into Arduino 'libraries' folder.
	3. TODO Search and install FFT by Robin Scheibler
	4. Search and install Bluetooth by FT&WS


How to Install Libraries
------------------------
1. Open Arduino IDE.
2. Go to Sketch → Include Library → Manage Libraries.
3. In the Library Manager, search for the required libraries (e.g., SCServo) and click Install.
4. Ensure you're using the correct versions specified here.

Alternatively, you can manually install libraries by downloading them and placing them in your `Documents/Arduino/libraries folder`.


External Libraries
==================

This firmware may reference external libraries via `#include <library.h>`. These libraries are not included in this repository, and it is the user's responsibility to ensure they are installed and correctly licensed.

Please refer to the following external dependencies:

* SCServo by FT&WS: (GNU GPL) — [https://github.com/workloads/scservo]
* BluetoothSerial by Henry Abrahansen: [License type] — [https://github.com/hen1227/bluetooth-serial]
* FFT by Robin Scheibler: (Custom License) — [https://github.com/Tinyu-Zhao/FFT]
  

How to Compile and Upload
=========================

Ensure you have all dependencies installed.

Navigate to the firmware directory and open the .ino file using Arduino IDE.

Use Arduino IDE to build and upload the binary to the microcontroller board.


TODO: Arduino configuration


Contributions
=============

Contributions are welcome! To contribute:

1. Fork the repo.
2. Create a branch (feature-xyz).
3. Submit a pull request.


Contact
=======

For any questions or collaboration, reach out at: vittorio.lumare@robotgarage.org

Feel free to open issues for bugs, feature requests, or discussions!

