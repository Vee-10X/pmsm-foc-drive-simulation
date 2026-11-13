# Field-Oriented Control Fundamentals

## Overview

Field-Oriented Control (FOC) is a control strategy used to regulate the operation of a Permanent Magnet Synchronous Motor (PMSM). The main objective is to control the motor by transforming the measured three-phase currents into components that are easier to regulate.

The PMSM model developed in this project provides three important feedback signals:

- Motor speed
- Rotor position
- Three-phase motor currents

These signals will be used in the FOC controller.

## From Three-Phase Currents to the d-q Reference Frame

The three-phase current sensor measures the motor phase currents:

\[
i_a,\quad i_b,\quad i_c
\]

In FOC, these currents are transformed into the rotating d-q reference frame through two main transformations:

\[
(i_a,i_b,i_c)
\rightarrow
(i_\alpha,i_\beta)
\rightarrow
(i_d,i_q)
\]

### Clarke Transformation

The Clarke transformation converts the three-phase currents into two stationary reference-frame components:

\[
(i_a,i_b,i_c)
\rightarrow
(i_\alpha,i_\beta)
\]

### Park Transformation

The Park transformation converts the stationary components into the rotating d-q reference frame:

\[
(i_\alpha,i_\beta)
\rightarrow
(i_d,i_q)
\]

The transformation uses the rotor position measured from the PMSM.

## The d- and q-Axis Currents

The d-axis is aligned with the rotor magnetic field, while the q-axis is perpendicular to it.

For the initial control strategy in this project, the d-axis current reference is set to:

\[
i_d^*=0
\]

The q-axis current is primarily associated with torque production. For a surface-mounted PMSM operating with \(i_d=0\), the electromagnetic torque can be expressed as:

\[
T_e=\frac{3}{2}p\lambda_m i_q
\]

where:

- \(T_e\) is the electromagnetic torque.
- \(p\) is the number of pole pairs.
- \(\lambda_m\) is the permanent-magnet flux linkage.
- \(i_q\) is the q-axis current.

## Speed Control Loop

The desired motor speed is represented by:

\[
\omega^*
\]

The controller compares the desired speed with the measured motor speed:

\[
e_\omega=\omega^*-\omega
\]

The speed error is processed by a PI controller to generate the q-axis current reference:

\[
i_q^*=K_p e_\omega+K_i\int e_\omega dt
\]

If the motor speed is below the reference value, the controller increases the torque-producing current demand to accelerate the motor.

## Current Control Loops

The FOC system uses two current control loops.

The d-axis current error is:

\[
e_d=i_d^*-i_d
\]

The q-axis current error is:

\[
e_q=i_q^*-i_q
\]

These errors are processed by PI controllers to generate voltage commands:

\[
v_d^*,\quad v_q^*
\]

The voltage commands are then transformed back into three-phase quantities and used to generate switching signals for the three-phase IGBT converter.

## FOC Control Structure

The overall control process can be summarised as:

1. A speed reference is provided.
2. The measured motor speed is compared with the reference.
3. The speed PI controller generates the torque-producing current reference, \(i_q^*\).
4. The measured three-phase currents are transformed into \(i_d\) and \(i_q\).
5. Current PI controllers regulate \(i_d\) and \(i_q\) to generate voltage commands.
6. The voltage commands are transformed back into three-phase quantities.
7. Switching signals are generated for the IGBT converter.
8. The converter supplies controlled three-phase power to the PMSM.
9. Motor speed, position, and currents are measured and fed back to the controller.

## Next Stage

The next stage of the project is to implement the FOC control structure in MATLAB/Simulink, beginning with the current transformations and feedback signals.
