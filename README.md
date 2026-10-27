# 6AGM supplementary control code

Control software for a custom six-degree-of-freedom arbitrary ground motion system (6AGM), built from six motorized linear sliders and inspired by a Stewart platform. The system generates controlled platform motion, calculates actuator positions, drives stepper motors, reads rotary encoders, and applies position-feedback correction.

All original code was written by **Fumiya Imamura**, as credited in the supplied documentation.

## Software components

- **LabVIEW** provides the main control interface and coordinates motion calculation, hardware commands, position sampling, and feedback.
- **MATLAB** generates trajectories and computes the inverse kinematics for the six actuators.
- **Arduino** sketches generate motor-driving pulses and read encoder positions using an Arduino Mega.

## Repository contents

```text
6AGMv1/                         Original unpacked supplementary code
  Arduino/                      Motor-output and encoder sketches
  Labview/                      Main VI and supporting VIs
  Labview_project/              LabVIEW project
  Matlab/                       Motion and inverse-kinematics script
  Read_Me_Supplementary_Code.txt
6AGMv1.zip                      Original supplementary-code archive
Read_Me_Supplementary_Code.txt   Original documentation
README.md                       This repository overview
LICENSE                         Non-commercial academic research terms
```

See the [original README](Read_Me_Supplementary_Code.txt) for the control flow, motion types, coordinate conventions, platform geometry, hardware requirements, safety notes, and citation information.

## Use and verification status

This is hardware-specific supplementary code, not a plug-and-play controller. It requires the corresponding motor drivers, rotary encoders, Arduino modules, NI data-acquisition hardware, and a LabVIEW control computer. Geometry, actuator limits, DAQ channels, serial settings, and emergency-stop procedures must match the actual system; see the original README before use.

The original source files are preserved unchanged. The supplied LabVIEW project contains relative references that may need to be relinked to the bundled files in your local environment; those references have not been modified here. No LabVIEW execution or hardware testing was performed during repository preparation.

## License

The original 6AGM code and accompanying documentation, including the code in `6AGMv1.zip`, are available free of charge **solely for non-commercial academic research** under the [6AGM Non-Commercial Academic Research License](LICENSE). You may use, copy, modify, and share them under those terms. Retain existing copyright and author-attribution notices and the license, and clearly identify modifications when sharing a modified version. Commercial use and any use outside non-commercial academic research require separate prior permission from the applicable rights holder(s). Third-party materials remain subject to their own terms. The software is provided without warranty and is used at your own risk.
