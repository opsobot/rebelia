Rebelia Hand Firmware
=====================

Overview
========

This folder contains the firmware for the Rebelia Hand, responsible for controlling the servomotors and processing commands for hand movements. The firmware is designed to work seamlessly with the open-source hardware of the robotic hand.

The firmware is licensed under the MIT License, ensuring that any modifications or derivatives remain open-source and freely available to the community.

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

The firmware in this folder is released under the MIT License.

By using or modifying this firmware, you agree to:

Attribute the original authors.

If you distribute any modified versions of this firmware, they must be released under the same MIT license.

Include a copy of the MIT license with any distribution of this firmware.

You can find the full license text in the LICENSE.txt file.


Copyright Notice
================

Each source file contains a copyright notice similar to the following:

```
/*
Rebelia-Hand-Firmware is the control software for the Rebelia Hand

https://www.robotgarage.org

MIT License

Copyright (c) 2025 Vittorio Lumare

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
```

Dependencies
==================
This firmware may reference external libraries via `#include <library.h>`. These libraries are not included in this repository, and it is the user's responsibility to ensure they are installed and correctly licensed.

Please refer to the following external dependencies:

* SCServo by FT&WS: (GNU GPL) — [https://github.com/workloads/scservo]
* BluetoothSerial by Henry Abrahamsen: [License type] — [https://github.com/hen1227/bluetooth-serial]
* ESP32FFT modified version by Vittorio Lumare: (Custom License) — [https://github.com/vittorio-lumare/ESP32FFT]
	* Note:  The original version is by Robin Scheibler [https://github.com/fakufaku/esp32-fft]


Prepare the Arduino IDE for ESP32
=================================
1. Install the Arduino IDE 2
2. Open the Arduino IDE 2
3. Open the Arduino Boards Manager
	1. Install 'esp32' by Expressif Systems


Install the Libraries
=====================
Note: Some libraries are manually installed, while others use Arduino's Library Manager. 

Libraries with manual installation
----------------------------------
1. Install the ESP32FFT library:
2. Download the ESP32FFT Arduino library from [https://github.com/vittorio-lumare/ESP32FFT]
3. Save the 'ESPfft' folder in your `Documents/Arduino/libraries` folder.

Libraries with automatic installation
-------------------------------------
1. Open Arduino IDE.
2. Go to Sketch → Include Library → Manage Libraries.
3. Open the Arduino's Library Manager:
	1. Search and install 'SCServo' by FT&WS
 	2. Search and install 'BluetoothSerial' by Henry Abrahamsen

Alternatively, you can manually install these libraries by downloading them and placing them in your `Documents/Arduino/libraries` folder.

Prepare the 'Rebelia Human' sketch
==================================
1. Open the Arduino IDE 2
2. Open 'RebeliaHuman' sketch

How to Compile and Upload
=========================
1. Ensure you have all dependencies installed.
2. Navigate to the firmware directory and open the .ino file using Arduino IDE.
3. Use Arduino IDE to build and upload the binary to the microcontroller board.

Contributions
=============
Contributions are welcome! To contribute:

1. Fork the repo.
2. Create a branch (feature-xyz).
3. Submit a pull request.


Contact
=======
For any questions or collaboration, reach out at vittorio.lumare@robotgarage.org
Please stay tuned on https://www.robotgarage.org
Feel free to open issues for bugs, feature requests, or discussions!

