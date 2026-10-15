Rebelia V1.0.0 - An Open Source Robotic Hand for Humans and Robots
==================================================================
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.23394100.svg)](https://doi.org/10.5281/zenodo.23394100)
*Not affiliated with igus ReBeL.*

> **This is Rebelia V1, the version certified by OSHWA.** The next version, Rebelia V2
> (branch `V2`), became **Yeah Hand** v1.0; development continues in
> [opsobot/yeah-hand](https://github.com/opsobot/yeah-hand).

Overview
========

Welcome to the Rebelia Hand Project — an open-source, humanlike robotic hand designed for researchers, students, and robotics enthusiasts.

This version (V1) has been certified by OSHWA, you can see the certificate [here](https://certification.oshwa.org/it000021.html) (UID IT000021, 4 April 2025).

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
  ├── defensive-publication/ (technical disclosure for prior-art purposes)
├── LICENSE (licenses description for the whole repo)
├── README.md (project overview)
├── VARIANTS.md (alternative embodiments and possible extensions)
├── TRADEMARK.md (use of the Yeah Hand name)
├── CITATION.cff, .zenodo.json (citation and archiving metadata)
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
* Firmware : released under the GNU GPL License, version 3 or later. [Full license text](https://www.gnu.org/licenses/gpl-3.0.html)
* Documentation and multimedia : released under the Creative Commons Attribution-ShareAlike 4.0 License (CC-BY-SA). [Full license text](https://creativecommons.org/licenses/by-sa/4.0/deed.en)

Anyone may study, build, modify and sell this design or products based on it. In short, if you distribute the design, modified versions or products made from it, you must:
* Credit the original authors and keep the license notices.
* Release modifications or derivatives of the hardware under the same CERN OHL-S license, with their sources, and indicate the Source Location (https://github.com/opsobot/rebelia) on products where practicable.
* Release modifications or derivatives of the firmware under the GNU GPL license.
* Release modifications or derivatives of documentation and multimedia content under the CC-BY-SA license.

Both CERN OHL-S v2 and GNU GPLv3 include a patent licence: every contributor grants all users the rights under their patents needed to use what they distribute, and anyone who starts patent litigation over the design loses their rights under the licence. See the LICENSE file for details.

Contributing
============
We welcome contributions! Please:
1. Fork the repo.
2. Create a branch (feature-xyz).
3. Submit a pull request.

Contributions are accepted under the same license as the part of the repository they modify. If you introduce a new technical solution, please also describe it, with any alternatives you considered, in [VARIANTS.md](VARIANTS.md): publishing ideas openly keeps them free for everyone.

Defensive Publication
=====================
This design is publicly released to establish prior art and prevent patenting by third parties.
Our goal is to ensure that the robotic hand remains open and accessible to all: nobody should be able to obtain exclusive rights that stop anyone else from making, improving or selling it.

* First public disclosure on GitHub: 3 March 2025 (mechanics), 30 March 2025 (firmware, including two-channel EMG control), 20 April 2025 (electronics).
* OSHWA certification record, an independent dated record of the prosthetic version with two EMG sensors: [IT000021](https://certification.oshwa.org/it000021.html), 4 April 2025.
* Alternative embodiments and extensions: [VARIANTS.md](VARIANTS.md)
* Technical disclosure: [docs/defensive-publication](docs/defensive-publication/technical-disclosure.md), also published on [Technical Disclosure Commons]([TDCOMMONS LINK])
* Archived releases with permanent DOI on Zenodo: v1.0.0 [10.5281/zenodo.23394100](https://doi.org/10.5281/zenodo.23394100)

Trademark
=========
The design is free. Its successor is sold as "Yeah Hand", a registered EU trademark (EUTM no. 019374244). You are welcome to build and sell your own version under your own name, and to say it is "based on the Yeah Hand". See [TRADEMARK.md](TRADEMARK.md).

How to Cite
===========
Use the "Cite this repository" button on GitHub or the metadata in [CITATION.cff](CITATION.cff).

For further information, visit our website: http://www.robotgarage.org

Contact
=======
For any questions or collaborations, reach out to us at: vittorio.lumare@robotgarage.org.


