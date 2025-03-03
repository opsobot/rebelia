Rebelia - An Open Source Robotic Hand for Humans and Robots
===========================================================


Overview
========

Welcome to the Robotic Hand Project — an open-source, humanlike robotic hand designed for researchers, students, and robotics enthusiasts.

Our goal is to provide an affordable, high-performance robotic hand under 2000 EUR, open to community contributions and continuous improvement.

This project is released under the CERN Open Hardware License (OHL-S), ensuring the design remains open-source and preventing patent claims on its core innovations.

Features
* Fully 3D-printable design using affordable materials.
* Commercial off-the-shelf components for servos and electronics.
* Custom firmware for precise motor control.
* Modularity for easy upgrades and customization.

Repository Structure
====================

```
/hardware
  ├── CAD/ (STEP, IGES, and native formats like Blender or SolidWorks)
  ├── renders/ (images of the robotic hand)
  ├── BOM.md (Bill of Materials — list of components and suppliers)
/firmware
  ├── src/ (firmware code)
  ├── README.md (how to compile and upload the firmware)
/docs
  ├── LICENSE (CERN OHL-S)
  ├── README.md (project overview)
```


CAD Files
=========

Our CAD files are hosted on both GitHub and GrabCAD:
* GitHub (for version control and collaboration): View CAD files here
* GrabCAD (for easy preview and download): View on GrabCAD

Bill of Materials (BOM)
=======================
Find the full list of commercial components required to build the robotic hand in BOM.md.


Firmware
========
The custom firmware is located in the /firmware directory. Instructions for compiling and uploading are available in the Firmware README.

Licensing
=========
This project is licensed under multiple licenses:
* Hardware : released under the CERN Open Hardware License Strongly Reciprocal v2 (CERN OHL-S). Full license text: https://ohwr.org/project/cernohl/-/wikis/uploads/819d71bea3458f71fba6cf4fb0f2de6b/cern_ohl_s_v2.txt
* Firmware : released under the GNU CPL License (GNU GPL). Full license text: https://www.gnu.org/licenses/gpl-3.0.txt
* Multimedia : released under the Creative Commons Share-Alike License (CC-BY-SA). Full license text: https://creativecommons.org/licenses/by-sa/4.0/deed.en

By using or modifying this hardware, you agree to:
* Credit the original authors.
* Share any modifications or derivatives under the same CERN OHL-S license.
* Prevent patent claims on the design.



Contributing
============
We welcome contributions! Please:
1. Fork the repo.
2. Create a branch (feature-xyz).
3. Submit a pull request.

Defensive Publication
=====================
This design is publicly released to establish prior art and prevent patenting by third parties. 
Our goal is to ensure that the robotic hand remains open and accessible to all.

For further information, visit our website: http://www.robotgarage.org

Contact
=======
For any questions or collaborations, reach out to us at: vittorio.lumare@robotgarage.org.


