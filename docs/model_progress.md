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


## Feedback and Measurement Setup

To support the future implementation of Field-Oriented Control, measurement components were added to obtain feedback from the PMSM model.

### Rotational Motion Measurement

An Ideal Rotational Motion Sensor was connected across the PMSM mechanical system.

- The sensor's **R port** is connected to the PMSM rotor shaft.
- The sensor's **C port** is connected to the mechanical reference network.
- The **W output** provides the angular velocity of the rotor.
- The **A output** provides the angular position of the rotor.

The W and A outputs are physical signals. Two PS-Simulink Converter blocks were therefore used to convert these signals into standard Simulink signals for use in the control system.

### Three-Phase Current Measurement

A Three-Phase Current Sensor was inserted between the three-phase converter and the PMSM.

The electrical connection is:

**Three-Phase Converter → Three-Phase Current Sensor → PMSM**

The sensor measures the motor's three-phase currents. Its **I output** is a physical signal and is connected to a PS-Simulink Converter to make the measured current signal available to the future Simulink-based FOC controller.

### Current Model Structure

The PMSM drive model now consists of the following main sections:

**Power path:**

DC Voltage Source → Three-Phase IGBT Converter → Three-Phase Current Sensor → PMSM

**Mechanical system:**

PMSM Rotor → Ideal Torque Source → Mechanical Rotational Reference

**Feedback signals:**

- Angular velocity
- Rotor position
- Three-phase motor currents

These measurements will provide the feedback required for the next stage of the project: implementation of the Field-Oriented Control system.
