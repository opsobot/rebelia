# Yeah Hand — Alternative embodiments, variants and extensions

This document is part of the public technical disclosure of the Yeah Hand
(formerly Rebelia), an open source tendon-driven anthropomorphic hand usable as
a robotic end effector and as an active prosthetic hand.

Its purpose is to describe, as broadly as possible, the ways in which the
concepts of this design can be implemented, modified and extended, so that all
of them are publicly available prior art and remain free for anyone to make,
use, improve and sell. Each entry is meant to be understandable and
reproducible by a person skilled in robotics or prosthetics.

First public disclosure of the design on GitHub (https://github.com/opsobot/rebelia):

| Part | Date |
|---|---|
| Mechanical design (CAD, STL, Blender source) and renders | 2025-03-03 |
| Control firmware | 2025-03-30 |
| Electronic board, prosthetic version | 2025-04-20 |
| Assembly instructions | 2025-11-12 |
| Robotic version firmware (Yeah Hand) | 2025-12-21 |
| This document | 2026-10-15 |

---

## 1. Core concepts of the reference design

1. **All actuators inside the hand.** Five rotary actuators are housed in a
   single printed internal skeleton of the hand (covered by a separate palm
   part and covers): four for finger flexion (index,
   middle, ring+little, thumb) and one for thumb rotation/opposition. No
   forearm-mounted actuators are needed, so the same hand can be mounted on a
   robot flange or on a prosthetic socket.
2. **Antagonistic tendons on a single spool.** Each finger has a flexion
   tendon and an extension tendon, both fixed to the same spool mounted on the
   actuator output flange and wound in opposite directions, so that a single
   actuator actively drives both closing and opening of the finger.
3. **Tendon routing through the printed parts.** Tendons run through a sequence
   of holes and channels in the printed phalanges and skeleton, without
   pulleys, sheaths or bearings, and pass over a tendons-barrier that keeps
   them separated and guided towards the spools.
4. **Self-locking distal anchoring.** Each tendon is anchored to the distal
   phalanx by passing it through holes and wrapping it twice around itself, so
   that tension tightens the wraps; the spool end is locked under a bolt head.
5. **Coupled ring and little fingers.** The ring and little fingers share one
   actuator. Their tendons are joined by a crimped clamp and pulled through a
   sliding tendon loop, so that the two fingers can close by different amounts
   and adapt to the shape of the object (differential, underactuated
   coupling). The relative tendon lengths set the closing sequence.
6. **Thumb opposition on the actuator flange.** The thumb base is bolted
   directly to the output flange of the thumb rotation actuator, which carries
   the thumb flexion tendon.
7. **Load-based control.** Serial bus servomotors with position and load
   feedback allow:
   - automatic tendon calibration at start-up, closing and opening each
     finger until a load threshold is reached and storing the end positions;
   - grasps that close each finger until a per-finger load limit is reached
     (force-limited, shape-adaptive grasping);
   - torque limiting of each actuator.
8. **Two-channel EMG control (prosthetic version).** Two surface EMG
   sensors (open and close). Each signal is processed by FFT and its intensity
   is compared with its own threshold: open above threshold → decrease
   closure; only close above threshold → increase closure; both below → hold.
   Open has priority. Closure changes by fixed steps within 0–100%.
   Additional features: automatic relax after a period at 0% closure; grasp
   selection by rotating the unpowered thumb by hand and reading its
   position; optional grasp switching by a double-open EMG pattern.
9. **Grasp presets controlled by one parameter.** Predefined grasps (power,
   small power, "monkey"/hook, pinch, relax, power tool) map a single closure
   value from 0 to 100% to a different trajectory for each finger. A RAW mode
   sets each degree of freedom independently. A dedicated mode moves the
   actuators to the positions needed for tendon installation.
10. **Simple text command protocol** over a wireless or wired serial link.
11. **Two configurations from the same core**: robotic gripper with mounting
    flange, and prosthetic hand with socket cover, wrist connection and a wrist
    joint with compliant (soft) bearings.

## 2. Mechanical variants

| Element | Reference implementation | Alternatives |
|---|---|---|
| Skeleton | Single FDM-printed internal skeleton housing all actuators, with separate palm part and covers | Multi-part printed assembly; SLS/MJF nylon; injection-moulded; CNC-machined aluminium or magnesium; carbon-fibre composite; cast resin |
| Finger structure | Printed phalanges with rigid and flexible parts | Fully rigid phalanges with pin joints; compliant (flexure) joints printed in TPU/TPE; monolithic finger with living hinges; overmoulded rigid core and soft skin; elastic return elements (springs, elastic cords) instead of extension tendons |
| Number of phalanges | 3 per finger, 2 for the thumb | 2 or 4 phalanges; fused distal-medial phalanges with fixed coupling |
| Number of fingers | 5 | 2, 3 or 4 fingers using the same modules; 6-finger or mirrored-thumb grippers; left and right hand versions |
| Scale | Human-size hand | Child-size, partial hand (individual fingers for partial amputations), oversize industrial gripper, miniature versions |
| Covers | Snap-fit printed covers | Silicone glove; textured high-friction pads; sensorised skin; cosmetic covers; interchangeable fingertips (soft, hard, tool tips) |
| Wrist | Joint with soft bearings | Rigid quick-disconnect; passive compliant wrist; actuated wrist rotation and/or flexion; standard prosthetic wrist interfaces (e.g. threaded or bayonet); ISO 9409 robot flange |
| Mounting | Robot flange with alignment pin; prosthetic socket cover | Quick-change tool changer; magnetic coupling; direct socket lamination; adapter plates for different robot brands |

## 3. Transmission and tendon variants

| Element | Reference implementation | Alternatives |
|---|---|---|
| Tendon material | Nylon line | UHMWPE (Dyneema/Spectra), aramid, steel cable, PTFE-coated cable, braided fishing line |
| Spool | One spool per actuator with flexion and extension tendons wound in opposite directions | Separate spools of different diameters for flexion and extension (to compensate for different tendon excursions); non-circular (cam-shaped) spools for variable transmission ratio; spool integrated in the servo horn |
| Tendon locking on spool | Bolt head clamping | Set screw, crimp sleeve, knot in a slot, wedge clamp, quick-release clip |
| Tendon locking on phalanx | Double self-wrap | Knot, crimp sleeve, glue, printed hook, threaded insert with set screw |
| Tendon guiding | Holes and channels in printed parts, tendons-barrier | PTFE tubes; Bowden sheaths; small pulleys or bearings at joints; low-friction inserts |
| Ring–little coupling | Shared tendon with crimped clamp and sliding loop | Whiffletree / rocker lever; pulley differential; elastic link; coupling of more fingers (e.g. middle+ring+little on one actuator); full single-actuator hand with differential tree |
| Tendon tension | Set at installation and calibrated by firmware | Spring-loaded tensioners; elastic segment in series (series elastic actuation); adjustable screw tensioners at the spool |
| Alternative transmissions | Tendons | Linkages; worm gear at each joint; lead screw and nut driving a linkage; hydraulic or pneumatic soft actuators |

## 4. Actuators and electronics variants

| Element | Reference implementation | Alternatives |
|---|---|---|
| Actuators | 5 serial bus servomotors (WaveShare ST3215) with position and load feedback | Other smart servos (Dynamixel, Feetech, etc.); DC gear motors with encoders and current sensing; BLDC with gearbox; linear actuators pulling tendons; non-backdrivable worm-drive motors for zero-power holding |
| Degrees of freedom | 5 actuated (index, middle, ring+little, thumb flexion, thumb rotation) | 1 to 6 actuators; independent little finger; actuated thumb abduction and adduction as separate axes; manual (passive) thumb positioning with locking |
| Controller | ESP32 (custom board for prosthesis; WaveShare servo driver for robot) | RP2040, STM32, nRF52, any microcontroller with UART; FPGA; control from the robot controller directly |
| Servo bus interface | Half-duplex UART with buffer ICs | RS-485, CAN bus, EtherCAT, PWM servos |
| Power supply | 12 V DC, switching regulator to 5 V | Prosthetic Li-ion battery pack (e.g. 2S/3S); robot tool-flange power; USB-C PD; supercapacitor buffer for peak current |
| Feedback / sensing | Servo position and load | Fingertip force sensors (FSR, capacitive, barometric), tendon tension sensors, joint angle encoders, tactile skins, IMU, proximity sensors, camera in the palm |

## 5. Control and interface variants

- Grasp selection and closure commanded by: Bluetooth serial (reference),
  Wi-Fi, USB, wired UART, robot I/O, CAN, ROS / ROS 2 nodes.
- For prosthetic use: surface EMG (one or more channels, pattern
  recognition or threshold control), force myography, ultrasound sensing,
  implanted sensors, foot switches, voice commands, smartphone app,
  co-contraction to switch grasp, IMU gestures, automatic grasp selection by a
  camera and object recognition.
- Closure strategies: position control, force limit per finger (reference),
  impedance/compliance control, slip detection with automatic re-gripping,
  closing speed proportional to the control signal.
- Calibration: load-threshold calibration at start-up (reference), calibration
  with end-stop switches, manual calibration via app, periodic re-calibration
  to compensate for tendon creep.
- Additional grasp presets: lateral (key) grip, tripod, pointing, hook,
  spherical, cylindrical, precision pinch, user-defined grasps recorded by
  demonstration.

## 6. Applications

- End effector for collaborative and industrial robot arms.
- Upper-limb prosthesis (transradial, wrist disarticulation; partial hand with
  individual finger modules).
- Research and teaching platform for robotics, grasping and machine learning.
- Humanoid robots and teleoperation (with a data glove or exoskeleton as
  master).
- Rehabilitation devices and hand exoskeletons using the same tendon and spool
  actuation on a glove.
- Remote handling in hazardous environments; animatronics; assistive
  feeding devices.

## 7. Solutions considered and discarded

*To be completed by the author: list here the approaches tried during
development and why they were changed (e.g. threading tools for tendons, other
tendon materials, other anchoring methods, other licences or architectures).
Discarded solutions are prior art too.*

- Tendon threading: several tools were tried; a loop of 0.35 mm nylon thread
  used as a needle proved the fastest and most reliable method.

---

## Change log

| Date | Change |
|---|---|
| 2026-10-15 | First version of this document |
