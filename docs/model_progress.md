# Model Development Progress

## Initial PMSM Drive Setup

The initial physical model of the PMSM drive has been developed in MATLAB/Simulink using Simscape Electrical.

The current model includes:

- A Permanent Magnet Synchronous Motor (PMSM).
- A three-phase converter configured with IGBT switching devices.
- A DC voltage source supplying the converter.
- Electrical and mechanical reference blocks.
- A Solver Configuration block for the Simscape physical network.
- An Ideal Torque Source for applying mechanical load to the motor shaft.
- A constant load torque input, initially set to zero.

## Current Model Structure

DC Voltage Source → Three-Phase Converter → PMSM

The mechanical load is connected to the PMSM shaft through an Ideal Torque Source.

## Next Steps

The next stage of the project is to add motor feedback measurements and develop the Field-Oriented Control system.
