# Tendon-driven anthropomorphic hand with actuators housed in the hand skeleton, single-spool antagonistic tendons, load-based adaptive grasping and two-channel EMG control, for robotic and prosthetic use

- **Author:** Vittorio Lumare, RobotGarage (vittorio.lumare@robotgarage.org)
- **Project name:** Yeah Hand (formerly Rebelia)
- **First public disclosure:** 3 March 2025, https://github.com/opsobot/rebelia
- **Archived releases (Zenodo):** v1.0.0 https://doi.org/10.5281/zenodo.23394100 ; OSHWA certification snapshot https://doi.org/10.5281/zenodo.23395450
- **OSHWA certification:** IT000021, 4 April 2025, https://certification.oshwa.org/it000021.html
- **License of this document:** CC BY 4.0

---

## Abstract

An anthropomorphic hand, manufacturable by 3D printing, usable both as a robotic
end effector and as an active prosthetic hand. Five rotary actuators are housed
inside a single internal skeleton of the hand, to which the fingers, the
palm part and the covers are attached. Each finger is moved by a flexion
tendon and an extension tendon fixed to the same spool on one actuator and
wound in opposite directions, so one actuator actively both closes and opens the
finger. Tendons are routed through holes in the printed phalanges and anchored
by self-tightening wraps. Ring and little fingers share one actuator through a
crimped clamp and a sliding tendon loop, giving an adaptive differential
closure. The thumb base is mounted directly on the output flange of a thumb
rotation actuator. Servo position and load feedback are used for automatic
tendon calibration at start-up and for grasps that close each finger until a
load limit is reached. Grasp presets map a single 0–100% closure command to
different finger trajectories. In the prosthetic version, two surface EMG
sensors are processed by FFT; the signal intensity of each is compared with its
own threshold to increase, decrease or hold the closure value.

## Keywords

robotic hand; prosthetic hand; artificial hand; gripper; end effector;
tendon-driven; cable-driven; antagonistic tendons; flexor tendon; extensor
tendon; spool; capstan; underactuated; differential coupling; adaptive grasp;
force-limited grasp; servo load feedback; tendon calibration; thumb opposition;
3D-printed hand; myoelectric prosthesis; grasp preset; EMG; electromyography; FFT; threshold
control; grasp selection; back-drivable thumb

## Background

Low-cost tendon-driven hands often place actuators in the forearm, which
prevents use as a compact robot end effector, or use a single tendon per finger
with elastic return, which gives weak and imprecise opening. Calibration of
tendon length is usually manual and is lost when tendons stretch. Fixed
position commands either crush soft objects or fail to grasp irregular ones.

## Description

### 1. Skeleton and actuators (Figs. 2, 3, 4)

A single internal skeleton part, covered by a separate palm part and by front
and back covers, contains five slots, one per
actuator: index flexion, middle flexion, ring-and-little flexion, thumb
flexion and thumb rotation. In the reference implementation the actuators are
serial bus servomotors (WaveShare ST3215) connected in a daisy chain, each
fixed by four M2 screws. The upper fingers are inserted in slots at the top of
the skeleton and fixed by one M3 bolt each. A tendons-barrier part is screwed to
the rear of the skeleton to separate and guide the tendons towards the spools.

### 2. Thumb (Fig. 5)

The thumb base is bolted directly onto the output flange of the thumb rotation
actuator, so that the actuator rotates the whole thumb about an axis that
produces opposition. The thumb is bolted to the thumb base and is flexed by a
tendon driven by the thumb flexion actuator located in the skeleton.

### 3. Fingers and tendon routing (Fig. 7)

Each finger is made of printed phalanges. A tendon enters from the rear of the
skeleton, crosses to the front, and is threaded in sequence through holes in the
proximal, medial and distal phalanges on the palmar side (flexion path). At the
distal phalanx it passes to the dorsal side and back, and is wrapped twice
around its own incoming segment; tension on the tendon tightens these wraps,
anchoring it to the distal phalanx without knots or glue. The tendon then runs
back through holes on the dorsal side of the phalanges (extension path), exits
at the rear of the skeleton, passes over the tendons-barrier and reaches the
spool. Tendons are threaded using a loop of thin nylon line used as a needle.

### 4. Single-spool antagonistic actuation (Fig. 6)

A spool is bolted to the output flange of each flexion actuator. The flexion
tendon is wound about half a turn in one direction and locked under a bolt head
in a hole of the spool; the extension tendon is wound about one turn in the
opposite direction and locked under a second bolt. Rotation of the actuator in
one direction winds the flexion tendon and unwinds the extension tendon, closing
the finger; rotation in the opposite direction opens it. Both motions are
actively driven by the same actuator, without return springs. During
installation the firmware moves the actuators to a predefined tendon
installation position; each tendon is pulled until the finger is at its
maximum opening and then locked.

### 5. Ring–little differential coupling (Fig. 8)

Ring and little fingers each have their own tendon routed as in section 3. At
the rear of the skeleton the ends of the two tendons are joined in one crimped
clamp, with different free lengths between each finger exit hole and the clamp
(e.g. 5 cm and 4.5 cm), which sets the relative closing sequence. A further
tendon, connected to the spool of the shared actuator, ends in a small loop
(about 3 mm diameter) that slides along the tendon segment between the two
fingers. When one finger is blocked by the object, the loop slides and the
other finger continues closing, so the two fingers adapt to the object shape
with a single actuator.

### 6. Electronics

Robotic version: a commercial ESP32 servo driver board. Prosthetic version: a
custom board with an ESP32 module, a 12 V to 5 V switching regulator, and
single-gate tri-state buffers (sn74lvc1g125 / sn74lvc1g126) converting the
microcontroller UART into the half-duplex serial bus of the servomotors.
Supply: 12 V DC.

### 7. Control method (Figs. 9, 10)

a. **Start-up tendon calibration.** For each flexion actuator the controller
moves the finger towards opening in small increments, reading the actuator load
at each step, until the load exceeds a threshold (finger fully extended against
its tendons); it then does the same towards closing. The positions found are
stored as the actual range of the finger, compensating tendon length and
stretch without manual adjustment.

b. **Grasp presets.** A grasp type (power, small power, "monkey"/hook, pinch,
relax, power tool) defines, for each degree of freedom, a trajectory as a
function of a single closure value from 0 to 100%. The user or robot sends only
the grasp type and the closure value.

c. **Force-limited adaptive closing.** On closing, each finger advances in
increments while its load is read; each finger stops independently when its
load reaches a per-finger limit or when its target position is reached. The
hand therefore wraps around objects of different shape and softness with a
limited force. A global torque limit is also set on each actuator.

d. **RAW mode.** The closure of each degree of freedom is set independently.

e. **Two-channel EMG control (prosthetic version).** Two surface EMG sensors
are placed on two muscle groups, one assigned to opening and one to closing.
In each control cycle the controller samples each sensor (in the reference
firmware 512 samples at about 9.9 kHz), removes the DC component, applies a
Hamming window and computes the FFT magnitude; the intensity is taken as the
magnitude in a frequency bin around 200 Hz. Each channel has its own intensity
threshold. Decision rule:
- open-channel intensity above its threshold → decrease the closure value
  (open), regardless of the close channel;
- open channel below threshold and close channel above its threshold →
  increase the closure value (close);
- both below their thresholds → keep the closure value unchanged.
The closure value is changed by a fixed step (5% in the reference firmware) per
cycle and limited to 0–100%; it is applied to the selected grasp preset, so a
single proportional value drives all fingers along the preset trajectories,
with the force-limited closing of point c. Giving priority to the open channel
makes releasing an object always possible.

f. **Automatic relax.** If the closure value stays at 0% for a set time (60 s
in the reference firmware), the hand automatically switches to the relaxed
grasp posture.

g. **Grasp selection by manual thumb positioning.** While the hand is open, the
controller disables the torque of the thumb rotation actuator, so the user can
rotate the thumb by hand with the other hand. The controller reads the actuator
position and selects the grasp preset whose thumb rotation position is closest
(within a tolerance), e.g. power, small power, hook, pinch or relax. Torque is
enabled again when closing starts. The thumb thus works as a mechanical grasp
selector, without extra buttons or sensors.

h. **Grasp switching by EMG pattern.** As an option, the sequence of recent
EMG decisions (open / close / none) is stored in a short buffer while the hand
is open; a double "open" pulse pattern cycles to the next grasp preset.

i. **Command protocol.** Plain-text commands over a serial link (Bluetooth in
the reference implementation), framed as `#CMD##\r\n`: grasp names select the
grasp, a number sets the closure, and `I<n>M<n>R<n>T<n>TR<n>` sets the five
degrees of freedom in RAW mode.

### 8. Configurations (Figs. 1, 11)

The same skeleton, fingers and actuators are used in two configurations: a
robotic gripper with a flange and alignment pin for mounting on a robot wrist,
powered from the robot tool connector; and a prosthetic hand with a forearm
cover, socket cover and a wrist joint with soft bearings.

## Figures

The figures are included in the PDF version of this disclosure
(`technical-disclosure.pdf` in this folder). Sources:
`docs/Assembly Instructions.pdf`; Figs. 1, 2 and 11 are rendered from the
Rebelia V1 STL files in `hardware/CAD/STL`.

- Fig. 1 — Rebelia V1 assembled: palmar, dorsal and three-quarter views.
- Fig. 2 — Internal skeleton, front and rear views; skeleton with thumb base, spool and fingers, covers removed.
- Fig. 3 — Skeleton with the five actuators installed, front and rear views.
- Fig. 4 — Arrangement of actuators, spools and tendons-barrier.
- Fig. 5 — Thumb base mounted on the thumb rotation actuator flange; thumb installed.
- Fig. 6 — Spools on the actuator flanges; spool holes (HS: tendon holes, BS: locking bolts).
- Fig. 7 — Fingers installed on the skeleton; tendon holes of the index finger.
- Fig. 8 — Schematic of the ring–little coupling: clamp and sliding loop.
- Fig. 9 — Flow chart of start-up calibration and force-limited closing.
- Fig. 10 — Flow chart of the two-channel EMG decision rule.
- Fig. 11 — Prosthetic configuration with socket cover; main dimensions of the hand.

## Alternative embodiments

The actuators may be any rotary actuators with position and load or current
sensing (smart servos, DC or BLDC motors with encoders and current
measurement). The number of fingers, phalanges and actuators may vary from one
to six actuators; more fingers may share an actuator through a cascade of
sliding loops, levers or pulleys. Tendons may be made of nylon, UHMWPE, aramid
or steel cable and may run in PTFE tubes, sheaths or over pulleys. Flexion and
extension tendons may use spools of different or non-circular profile. The
skeleton may be printed, moulded, cast or machined. Load sensing may be
replaced or complemented by fingertip force sensors, tendon tension sensors or
tactile skins. Commands may come from a robot controller, a computer, a
smartphone, EMG or other biosignal sensors, or a camera-based grasp selector.
EMG intensity may be computed as band power over several FFT bins, RMS,
envelope or wavelet features instead of a single bin; one, two or more EMG
channels may be used, with proportional instead of fixed-step closure changes,
adaptive or user-calibrated thresholds, or pattern-recognition classifiers.
The grasp may be selected by manual positioning of any back-drivable joint, not
only the thumb.
Further variants are listed in `VARIANTS.md` of the repository, which is part
of this disclosure.

## Advantages

- Compact: all actuators inside the hand skeleton, no forearm required.
- Active opening and closing with one actuator per finger, no return springs.
- Adaptive grasp of irregular objects with fewer actuators.
- Automatic calibration compensates tendon stretch and assembly tolerances.
- Limited, controllable grasp force for fragile objects and safe interaction.
- Low cost: printed parts and off-the-shelf components.
- One design for both robotic and prosthetic applications.

## Applications

Robot end effectors, collaborative robots, humanoid robots, teleoperation,
upper-limb and partial-hand prostheses, research and education, rehabilitation
and exoskeleton devices, remote handling.
