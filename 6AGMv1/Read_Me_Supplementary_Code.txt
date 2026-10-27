Supplementary Code for

A cortex-wide self-consistent manifold for a body-environment reference frame
Fumiya Imamura, Hiroto Imamura, Reiko Hira, Yoshikazu Isomura, Riichiro Hira


Overview

This Supplementary Code (6AGMv1) contains the control software used for the custom six-degree-of-freedom motion platform described in the manuscript “A cortex-wide self-consistent manifold for a body–environment reference frame.” The platform was used to deliver controlled ground motion to head-fixed mice while neural activity was recorded across the dorsal cortex. All code was written by Fumiya Imamura.

The code controls a six-degree-of-freedom arbitrary ground motion system (6AGM) composed of six motorized linear sliders. The 6AGM was developed with inspiration from the Stewart platform. It computes the actuator positions required to generate defined platform motions, drives the stepper motors, reads rotary encoder feedback, and corrects actuator-position errors during operation.

The software consists of LabVIEW, MATLAB, and Arduino components. LabVIEW provides the main control interface and coordinates the control loop. MATLAB computes motion trajectories and inverse kinematics. Arduino sketches generate motor-driving pulse signals and read encoder positions.


Folder Structure

Posture_control_with_stewart_platform/
├── Arduino/
│   ├── Output_module/
│   ├── Output_module_ver2/
│   └── Rotary_encoder_on_MEGA_ver9_parity_bug_fix/
├── Labview/
│   ├── Stewart_platform_6_deg_freedom_ver4_easier_upgrade.vi
│   └── SubVI/
├── Labview_project/
│   └── Stewart_platform_6_deg_freedom_ver4_20230908.lvproj
└── Matlab/
    └── Posture_control_6_deg_freedom_adjustible_buffer.m


Main Code Components

1. LabVIEW control program

The main LabVIEW VI is:

Labview/Stewart_platform_6_deg_freedom_ver4_easier_upgrade.vi

This VI controls the overall operation of the six-degree-of-freedom motion platform. It initializes control parameters, calls the motion-calculation routines, sends voltage and direction commands to the motor-output hardware, samples encoder positions, and applies feedback correction.

The SubVI folder contains supporting VIs for parameter initialization, motion calculation, feedback correction, voltage output, position sampling, final output control, and individual motion types.


2. MATLAB motion-generation and inverse-kinematics code

The main MATLAB script is:

Matlab/Posture_control_6_deg_freedom_adjustible_buffer.m

This script defines the geometry of the platform and computes the inverse kinematics required to drive the six linear sliders. For each desired platform pose, the code calculates the target slider positions that satisfy the mechanical constraints of the platform.

The script also checks whether the requested motion is physically achievable within the defined actuator range. The calculated actuator positions are then converted into motor-command parameters, including movement direction and pulse interval.


3. Arduino motor-output modules

The Arduino motor-output sketches are located in:

Arduino/Output_module/
Arduino/Output_module_ver2/

These sketches run on Arduino-based output modules. Each module reads an analog command voltage, converts it into a pulse interval, and generates pulse signals for a stepper motor driver. The pulse interval determines motor speed. Direction and enable signals are provided separately by the control system.

Output_module_ver2 is an updated version of the motor-output sketch that waits for analog-to-digital conversion to complete before updating the pulse interval.


4. Arduino encoder reader

The encoder-reading sketch is located in:

Arduino/Rotary_encoder_on_MEGA_ver9_parity_bug_fix/

This code runs on an Arduino Mega and reads rotary encoder signals from the six actuators. Encoder counts are transmitted to the LabVIEW control program through serial communication. These measured positions are used for feedback correction of actuator motion.


Motion Types

The code includes routines for several types of controlled platform motion, including:

- One-way motion (transition between different motions)
- Random-direction tilt motion (8DT)
- Tilt rotation (T-Rot)
- Back-and-forth tilt motion (Roll)
- Platform torsional or yaw-like rotation (Yaw)
- Vertical translation (Vert)
- Horizontal circular translation
- YZ-plane circular motion

These motion routines were used to generate controlled body-environment perturbations in the experiments described in the manuscript.


Control Flow

The control procedure can be summarized as follows:

1. The user specifies motion parameters in the LabVIEW control interface.
2. The MATLAB script computes the desired platform trajectory.
3. The inverse-kinematics calculation converts each platform pose into six actuator positions.
4. Differences between successive actuator positions are converted into movement directions and pulse intervals.
5. LabVIEW sends voltage and direction commands to the motor-output hardware.
6. Arduino output modules generate stepper-motor pulses.
7. The Arduino Mega reads rotary encoder positions from the six actuators.
8. LabVIEW compares measured and target actuator positions and applies feedback correction.


Feedback Correction

The control system uses encoder feedback to reduce actuator-position errors. At each update step, the measured actuator position is compared with the target position. The difference is used to correct the next actuator movement command.

In simplified form:

position error = measured position - target position

corrected movement = next target movement - position error

The correction is bounded to avoid excessively large feedback commands. This procedure compensates for small deviations between the intended and actual actuator positions.


Coordinate and Motion Parameters

The MATLAB code uses a custom coordinate convention for the six-degree-of-freedom platform. The main motion-related variables include:

- theta: direction of tilt or rotation
- phi: tilt angle
- tau: rotation around the platform normal
- alpha: x-axis translation
- beta: y-axis translation
- gamma: z-axis translation

The exact interpretation of these variables depends on the platform geometry defined in the MATLAB script. Users who adapt the code to another platform should verify the coordinate convention and mechanical geometry before operation.


Platform Geometry

The platform geometry is defined in the MATLAB script. The relevant parameters include the slider origin positions, slider direction vectors, platform radius, arm length, ball-joint offsets, and the minimum and maximum actuator positions.

These parameters determine the inverse-kinematics solution and must match the actual mechanical configuration of the platform. If the code is adapted to another platform, these geometric parameters must be modified accordingly.


Notes on Use

This code was written for the custom motion platform used in the study and is provided as Supplementary Code to document the control implementation. It is not intended as a general-purpose, plug-and-play controller for the 6AGM.

Operation of the platform requires appropriate hardware, including motor drivers, rotary encoders, Arduino-based pulse-output modules, an Arduino Mega encoder reader, National Instruments data-acquisition hardware, and a LabVIEW control computer.

Before operating the system, users should verify actuator ranges, motor directions, encoder signs, DAQ channel assignments, serial communication settings, and emergency-stop procedures. All new trajectories should be tested at low speed before use in experiments.


Safety Notice

This code controls moving mechanical hardware. Users should operate the system only after confirming that the mechanical range, actuator limits, motor-driver settings, encoder readings, and emergency-stop procedures are correctly configured.

The software includes checks for actuator-position limits, but users should not rely solely on software checks for hardware safety.


Citation

If this Supplementary Code is used or referenced, please cite the associated manuscript:

Imamura F., Imamura H., Hira R., Isomura Y., and Hira R. A cortex-wide self-consistent manifold for a body-environment reference frame.
