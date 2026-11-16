# PMSM Drive Modelling and Field-Oriented Control

## 1. Project Overview

This project focuses on the modelling and simulation of a Permanent Magnet Synchronous Motor (PMSM) drive using MATLAB/Simulink and Simscape Electrical. The objective is to develop a Field-Oriented Control (FOC) system for regulating the motor's speed and electromagnetic torque.

The model is being developed progressively, beginning with the physical motor drive, followed by feedback measurement, coordinate transformations, control-loop implementation, and PWM-based inverter switching.

## 2. Initial PMSM Drive Setup

The initial physical model of the PMSM drive was developed in MATLAB/Simulink using Simscape Electrical.

The model includes:

- A Permanent Magnet Synchronous Motor (PMSM).
- A Three-Phase Converter configured with IGBT switching devices.
- A DC voltage source supplying the converter.
- Electrical and mechanical reference blocks.
- A Solver Configuration block for the Simscape physical network.
- An Ideal Torque Source for applying mechanical load to the motor shaft.
- A constant load torque input, initially set to zero.

### 2.1 Main Electrical Power Path

The electrical power path is arranged as follows:

**DC Voltage Source → Three-Phase Converter → Three-Phase Current Sensor → PMSM**

The DC voltage source supplies the inverter, while the three-phase converter provides the switching interface required to drive the PMSM.

The Three-Phase Current Sensor is connected between the converter and the motor to measure the motor's phase currents.

### 2.2 Mechanical System

An Ideal Torque Source is connected to the PMSM shaft to provide an external mechanical load.

A constant input is used to define the applied load torque. The initial load torque is set to zero so that the motor can later be evaluated under different loading conditions.

Mechanical reference and rotational sensing components provide the necessary mechanical connections and measurements.

## 3. Feedback and Measurement Setup

Measurement components were added to obtain the feedback signals required for the future FOC controller.

### 3.1 Rotor Speed and Position Measurement

An Ideal Rotational Motion Sensor was connected across the PMSM mechanical system.

Its connections are:

- **R port:** Connected to the PMSM rotor shaft.
- **C port:** Connected to the mechanical rotational reference.

The sensor provides two physical signals:

- **W:** Rotor angular velocity.
- **A:** Rotor angular position.

Since these are physical signals, PS-Simulink Converter blocks are used to convert them into standard Simulink signals.

The angular velocity signal will be used for motor speed feedback, while the angular position signal will be used to calculate the rotor electrical angle required for coordinate transformations.

### 3.2 Three-Phase Current Measurement

A Three-Phase Current Sensor was inserted between the converter and the PMSM.

Its electrical arrangement is:

**Three-Phase Converter → Three-Phase Current Sensor → PMSM**

The sensor measures the motor's three-phase currents. Its physical signal output is connected to a PS-Simulink Converter to make the measurements available to the Simulink control system.

The measured phase currents are required for the Clarke and Park transformations.

## 4. Current Feedback Transformation

To implement Field-Oriented Control, the measured three-phase currents must be transformed into a rotating reference frame.

The current feedback transformation consists of two stages:

1. Clarke Transformation.
2. Park Transformation.

### 4.1 Separation of Three-Phase Current Signals

After conversion from physical signals to Simulink signals, a Demux block is used to separate the three-phase current signal into individual phase-current signals.

The Demux block is configured with three outputs:

- Output 1: Phase current \(i_a\).
- Output 2: Phase current \(i_b\).
- Output 3: Phase current \(i_c\).

The first two outputs are connected to the Clarke Transform inputs labelled **a** and **b**, respectively. The third output is currently not connected.

The use of two phase currents relies on the balanced three-phase assumption, where:

\[
i_a+i_b+i_c=0
\]

Therefore:

\[
i_c=-(i_a+i_b)
\]

This assumption and the Clarke Transform block configuration will be checked during model validation.

### 4.2 Clarke Transformation

The Clarke Transform converts the three-phase current representation into a stationary two-axis reference frame.

It produces two components:

- \(i_\alpha\): Current component along the stationary alpha axis.
- \(i_\beta\): Current component along the stationary beta axis.

The transformation is represented by:

\[
\begin{bmatrix}
i_\alpha\\
i_\beta
\end{bmatrix}
=
\frac{2}{3}
\begin{bmatrix}
1&-\frac{1}{2}&-\frac{1}{2}\\
0&\frac{\sqrt{3}}{2}&-\frac{\sqrt{3}}{2}
\end{bmatrix}
\begin{bmatrix}
i_a\\
i_b\\
i_c
\end{bmatrix}
\]

For a balanced three-phase system, the zero-sequence component is zero.

The Clarke Transform outputs are connected to the corresponding alpha and beta inputs of the Park Transform.

### 4.3 Park Transformation

The Park Transform converts the stationary alpha-beta current components into a rotating direct-quadrature reference frame.

Its outputs are:

- \(i_d\): Direct-axis current.
- \(i_q\): Quadrature-axis current.

The transformation is:

\[
\begin{bmatrix}
i_d\\
i_q
\end{bmatrix}
=
\begin{bmatrix}
\cos\theta_e&\sin\theta_e\\
-\sin\theta_e&\cos\theta_e
\end{bmatrix}
\begin{bmatrix}
i_\alpha\\
i_\beta
\end{bmatrix}
\]

The Park Transform requires the rotor electrical angle, supplied through separate sine and cosine inputs.

These d-q current components will later be used by the FOC current-control loops.

## 5. Rotor Electrical Angle Calculation

The Park Transformation requires the electrical position of the rotor rather than its mechanical position.

The Ideal Rotational Motion Sensor provides the mechanical angular position, \(\theta_m\).

The relationship between mechanical and electrical rotor position is:

\[
\theta_e=p\theta_m
\]

where:

- \(\theta_e\) = Electrical rotor angle.
- \(\theta_m\) = Mechanical rotor angle.
- \(p\) = Number of pole pairs.

The PMSM model is configured with **6 pole pairs**.

Therefore:

\[
\boxed{\theta_e=6\theta_m}
\]

### 5.1 Signal Processing Arrangement

The rotor position signal passes through the following blocks:

1. **PS-Simulink Converter:** Converts the physical angular position signal into a Simulink signal. Its output unit is configured in radians.
2. **Gain Block:** Multiplies the mechanical angular position by 6 to obtain the electrical angular position.
3. **Trigonometric Function Blocks:** Two branches receive the electrical angle. One calculates its sine, while the other calculates its cosine.

The resulting signals are connected as follows:

- Sine output → Park Transform `sin θe` input.
- Cosine output → Park Transform `cos θe` input.

This provides the angular information required to convert the measured stator currents into the rotating d-q reference frame.

## 6. Three-Phase Converter and Gate Control Interface

The Three-Phase Converter is configured as a detailed switching-device model using Insulated-Gate Bipolar Transistors (IGBTs).

The configured switching-device parameters include:

| Parameter | Configured value |
|---|---:|
| Switching device | IGBT |
| Threshold voltage | 6 V |
| Forward voltage | 0.8 V |
| On-state resistance | 0.001 Ω |
| Off-state conductance | \(1\times10^{-6}\) 1/Ω |

### 6.1 Six-Pulse Gate Multiplexer

A Six-Pulse Gate Multiplexer was added to provide the interface for the converter's six switching signals.

The multiplexer has six gate-signal inputs:

- Ga(H): Phase A upper switch.
- Ga(L): Phase A lower switch.
- Gb(H): Phase B upper switch.
- Gb(L): Phase B lower switch.
- Gc(H): Phase C upper switch.
- Gc(L): Phase C lower switch.

The multiplexer combines these six signals into a gate-control output.

Its output is connected to the `G` input of the Three-Phase Converter.

**Current status:** The multiplexer has been connected to the converter, but its six individual gate inputs have not yet been supplied with PWM switching signals.

The PWM generation and appropriate complementary switching/dead-time arrangement remain to be implemented.

## 7. Current Model Architecture

The model currently contains the following main sections.

### 7.1 Electrical Power Path

DC Voltage Source → Three-Phase IGBT Converter → Three-Phase Current Sensor → PMSM

### 7.2 Mechanical System

PMSM Rotor → Ideal Torque Source → Mechanical Reference Network

### 7.3 Feedback Measurement and Transformation

**Speed feedback:**

Ideal Rotational Motion Sensor → PS-Simulink Converter → Rotor Speed Signal

**Rotor position feedback:**

Ideal Rotational Motion Sensor → PS-Simulink Converter → Gain (6) → Sine and Cosine Signals

**Current feedback:**

Three-Phase Current Sensor → PS-Simulink Converter → Demux → Clarke Transform → Park Transform

The Park Transform receives the sine and cosine signals generated from the rotor electrical angle.

## 8. Current Development Status

The physical PMSM drive model and the initial feedback measurement and transformation structures have been assembled.

The following components have been added:

- PMSM and three-phase IGBT converter.
- DC voltage source and mechanical load arrangement.
- Three-phase current measurement.
- Rotor speed and position measurement.
- PS-Simulink signal conversion.
- Demux for phase-current separation.
- Clarke Transform.
- Park Transform.
- Rotor electrical angle calculation using six pole pairs.
- Sine and cosine generation.
- Six-Pulse Gate Multiplexer connected to the converter gate input.

### 8.1 Simulation and Validation Status

An initial simulation attempt produced a Simscape transient initialization error, with the Three-Phase Converter among the components reported in the Diagnostic Viewer.

The converter gate-control interface has since been connected using the Six-Pulse Gate Multiplexer. However, the six gate inputs are still awaiting PWM signals.

Consequently, the current feedback transformation structure and the complete drive model have not yet been verified through a successful simulation.

The model should therefore be considered a work in progress.

## 9. Next Development Stages

The remaining development work will follow these stages:

1. Implement a three-phase PWM generation system.
2. Generate six appropriate gate signals for the IGBT converter.
3. Implement complementary switching and dead-time protection.
4. Verify the converter and PMSM power circuit through simulation.
5. Validate the Clarke and Park transformation outputs.
6. Implement the outer speed PI controller.
7. Implement the inner d-axis and q-axis current PI controllers.
8. Develop the inverse Park and inverse Clarke transformations.
9. Integrate the complete FOC control system.
10. Evaluate motor response under different speed references and mechanical load conditions.
11. Analyse rotor speed, electromagnetic torque, and d-q current responses.
12. Document simulation results, observations, and conclusions.

## 10. Project Objective

The completed model will demonstrate the implementation of Field-Oriented Control for a PMSM drive using MATLAB/Simulink.

The performance evaluation will focus on speed and torque regulation, current response, and the motor's behaviour under changing reference speeds and mechanical loads.

The final project documentation will include the model architecture, mathematical equations, control-system design, simulation results, and performance analysis.
