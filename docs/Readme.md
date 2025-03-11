Rebelia - An Open Source Robotic Hand for Humans and Robots
===========================================================


Overview
========

Welcome to the Rebelia Hand Project — an open-source, humanlike robotic hand designed for researchers, students, and robotics enthusiasts.

Our goal is to provide an affordable, high-performance robotic hand under 2000 EUR, open to community contributions and continuous improvement.

This project hardware is released under the CERN Open Hardware License (OHL-S), ensuring the design remains open-source and preventing patent claims on its core innovations.

The firmware is released under the MIT License.

The documentation is released under the CC-BY License.


Features
* Fully 3D-printable design using affordable materials.
* Commercial off-the-shelf components for servos and electronics.
* Custom firmware for precise motor control.
* Modularity for easy upgrades and customization.

Our Approach to Open-Source Release
===================================
We use non-copyleft licenses to help us maintain a sustainable business model while still contributing to the open-source community. Our strategy is as follows:

1. Delaying Open-Source Release: We release each version of our design (source code, CAD files, documentation, etc.) under a non-copyleft, permissive license, but only after we have generated enough revenue to ensure the viability of our business. This allows us to focus on research, development, and support for our products without relying solely on open-source contributions or donations.
2. Maintaining Business Sustainability: By withholding the release of the source code or documentation for a new version until we've earned enough to support ongoing development, we can sustain our operations, pay for necessary resources, and invest in the improvement of the product. This strategy ensures that we can continue to develop new versions without compromising the financial health of our business.
3. Open-Source Releases with New Versions: Once a new version of the product is ready to be put on the market, we make the previous version open source, providing the source code, CAD files, and documentation to the community. This allows us to build on the contributions of others and encourage collaboration while ensuring we have sufficient financial backing from our latest commercial release.
4. Cycle of Continuous Development: Each version will be released commercially first, and only after we’re confident that the business can support the next phase will we release it as open-source. The new version will follow this pattern, ensuring that we’re always maintaining a healthy cycle of innovation and sustainability for our business.

This strategy enables us to balance open-source principles with the practical needs of running a business, ensuring that we can continue to develop, innovate, and support the community in the long term.


Repository Structure
====================

```
/hardware
  ├── CAD/ (STEP, IGES, and native formats like Blender or SolidWorks)
    ├── LICENSE (CERN OHL-S)
  ├── renders/ (images of the robotic hand)
    ├── LICENSE (CC-BY)
  ├── BOM.md (Bill of Materials — list of components and suppliers)
/firmware
  ├── LICENSE (MIT)
  ├── src/ (firmware code)
  ├── README.md (how to compile and upload the firmware)
├── LICENSE (general)
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
* Hardware : released under the CERN Open Hardware License Strongly Reciprocal v2 (CERN OHL-S). Full license text: [https://ohwr.org/project/cernohl/-/wikis/uploads/819d71bea3458f71fba6cf4fb0f2de6b/cern_ohl_s_v2.txt]
* Firmware : released under the MIT License (MIT). Full license text: [https://www.mit.edu/~amini/LICENSE.md]
* Multimedia : released under the Creative Commons Attribution License (CC-BY). Full license text: [https://creativecommons.org/licenses/by/4.0/]

By using or modifying this hardware, software and/or documentation, you agree to:
* Credit the original authors.
* If you release any modifications or derivatives to hardware, they must be released under the same CERN OHL-S license.
* If you release any modifications or derivatives to firmware, they must be released under the same MIT license.
* If you release any modifications or derivatives to multimedia content, they must be released under the same CC-BY license.
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


