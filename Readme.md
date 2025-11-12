Rebelia - An Open Source Robotic Hand for Humans and Robots
===========================================================

Overview
========

Welcome to the Rebelia Hand Project — an open-source, humanlike robotic hand designed for researchers, students, and robotics enthusiasts.

Our goal is to provide an affordable, high-performance robotic hand under 2000 EUR, open to community contributions and continuous improvement.

This project hardware is released under the CERN Open Hardware License (OHL-S), ensuring the design remains open-source and preventing patent claims on its core innovations.

The firmware is released under the GNU-GPL License.

The documentation is released under the CC-BY-SA License.


Features
* Fully 3D-printable design using affordable materials.
* Commercial off-the-shelf components for servos and electronics.
* Custom firmware for precise motor control.
* Modularity for easy upgrades and customization.

Our Approach to Open-Source Release
===================================
We use a copyleft open-source license to protect user freedoms and encourage meaningful collaboration, while maintaining a sustainable business model. Our strategy is as follows:

1. Development and Testing: We begin by developing and rigorously testing a new version of our product (including source code, CAD files, documentation, etc.) to ensure it meets our quality and performance standards.

2. Preparation for Launch and Sales: Once the new version is stable, we prepare it for market with dedicated marketing and pre-sales efforts. This phase ensures visibility and early adoption by our community and customers.

3. Open-Source Copyleft Release: We release each version immediately under a copyleft license (e.g., CERN-OHL, GPL), ensuring that the community can study, modify, and share the design—while also preserving those freedoms in derivative works. At the same time, we begin commercial distribution of that version.

4. Sustainable Sales and Ongoing Development: We continue to sell the open-source version for as long as it remains commercially viable, using the generated revenue to support our business, provide user support, and fund future development.

5. Continuous Innovation Cycle: While a version is being sold, we work on the next iteration—improving the design and preparing it for the next release. Once ready, we return to step 2, keeping the cycle of development, open-source release, and sustainable sales in motion.

This strategy ensures that we stay aligned with open-source ethics while securing the resources needed to support our work. It allows us to protect user freedom through copyleft, promote transparency, and build a strong, collaborative community—all while continuing to grow as a business.

Repository Structure
====================

```
/hardware
  ├── CAD/ (STL native Blender format)
    ├── LICENSE (CERN OHL-S)
  ├── renders/ (images of the robotic hand)
    ├── LICENSE (CC-BY-SA)
  ├── BOM.md (Bill of Materials — list of components and suppliers)
/firmware
  ├── LICENSE (GNU GPL)
  ├── src/ (firmware code)
  ├── README.md (how to compile and upload the firmware)
/docs
  ├── Assembly Instructions.odt (Original source document)
  ├── Assembly Instructions.pdf (Exported final document)
├── LICENSE (licenses description for the whole repo)
├── README.md (project overview)
```


CAD Files
=========

Our CAD files are hosted on both GitHub and GrabCAD:
* GitHub (for version control and collaboration): [View CAD files here](https://github.com/opsobot/rebelia/tree/main/hardware/CAD)
* GrabCAD (for easy preview and download): [View on GrabCAD](https://grabcad.com/library/rebelia-hand-1)

Bill of Materials (BOM)
=======================
Find the full list of commercial components required to build the robotic hand in BOM.md.

Firmware
========
The custom firmware is located in the /firmware directory. Instructions for compiling and uploading are available in the Firmware README.

Licensing
=========
This project is licensed under multiple licenses:
* Hardware : released under the CERN Open Hardware License Strongly Reciprocal v2 (CERN OHL-S). [Full license text](https://ohwr.org/project/cernohl/-/wikis/uploads/819d71bea3458f71fba6cf4fb0f2de6b/cern_ohl_s_v2.txt)
* Firmware : released under the GNU GPL License. [Full license text](https://www.gnu.org/licenses/gpl-3.0.html)
* Multimedia : released under the Creative Commons Attribution License (CC-BY-SA). [Full license text](https://creativecommons.org/licenses/by-sa/4.0/deed.en)

By using or modifying this hardware, software and/or documentation, you agree to:
* Credit the original authors.
* If you release any modifications or derivatives to hardware, they must be released under the same CERN OHL-S license.
* If you release any modifications or derivatives to firmware, they must be released under the same GNU GPL license.
* If you release any modifications or derivatives to multimedia content, they must be released under the same CC-BY-SA license.
* Prevent patent claims on the design and any derivative works.

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


