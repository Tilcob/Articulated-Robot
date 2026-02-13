# 3DOF Robotic Arm Controller (Arduino Nano)

## Overview

This project implements a 3DOF robotic arm controller using an Arduino Nano.
The system includes:

* Analytical forward and inverse kinematics
* Potentiometer-based Cartesian target input (X, Y, Z)
* Button-based target commit (short press)
* Long-press trajectory execution (predefined waypoints)
* Smooth joint motion control
* Configurable servo calibration and joint limits

The kinematic model is consistent with the following convention:

```
x =  cos(q1) * (L1 cos(q2) + L2 cos(q2+q3))
y = -sin(q1) * (L1 cos(q2) + L2 cos(q2+q3))
z =  h + L1 sin(q2) + L2 sin(q2+q3)
```

The negative sign in the Y coordinate is fixed and part of the model.

---

## Hardware Requirements

* Arduino Nano (ATmega328)
* 3 RC servos (Base, Shoulder, Elbow)
* Optional gripper servo
* 3 potentiometers (X, Y, Z)
* Optional gripper potentiometer
* 1 momentary push button
* External 5–6V servo power supply (recommended)

---

## Wiring

### Potentiometers

Each potentiometer:

* One side → 5V
* Other side → GND
* Middle pin → Analog input (A0–A3)

### Button

Configured with `INPUT_PULLUP`.

Wiring:

```
Pin 2  ---- Button ---- GND
```

* Not pressed → HIGH
* Pressed → LOW

### Servos

* Signal pins defined in `Config.h`
* Power servos from external supply
* Connect grounds together (Arduino GND ↔ Servo GND)

---

## Project Structure

* `Config.h` — geometry, limits, pins, calibration
* `Types.h` — basic data structures
* `Kinematics.*` — forward and inverse kinematics
* `Input.*` — potentiometer + button handling
* `Mapping.*` — joint angles → servo degrees
* `Motion.*` — smooth motion controller
* `Trajectory.*` — waypoint-based path execution
* `main.cpp` — system integration

---

## Operation Modes

### 1. Manual Target Mode (Short Button Press)

1. Adjust X, Y, Z using potentiometers.
2. Press the button briefly.
3. The target is latched and the arm moves to that position.

### 2. Trajectory Mode (Long Press > 700 ms)

1. Hold the button.
2. After release, trajectory execution begins.
3. The arm moves through predefined waypoints in `Trajectory.cpp`.

---

## Defining Custom Trajectories

Edit `Trajectory.cpp`:

```cpp
const Waypoint TRAJ_POINTS[] = {
    { {0.20f, 0.00f, 0.10f}, 0.5f },
    { {0.23f, 0.03f, 0.14f}, 0.5f },
    { {0.25f, -0.02f, 0.12f}, 0.2f },
};
```

Each waypoint contains:

* Cartesian position (meters)
* Gripper value (0.0–1.0)

---

## Motion Control

The motion controller:

* Limits joint speed (`MAX_SPEED_DEG_PER_S`)
* Moves joints smoothly toward targets
* Detects arrival within tolerance
* Advances trajectory automatically

---

## Calibration

Adjust in `Config.h`:

* `BASE_ZERO_DEG`
* `SHOULDER_ZERO_DEG`
* `ELBOW_ZERO_DEG`
* `*_SIGN`

Procedure:

1. Move arm mechanically to mathematical zero pose.
2. Set servo zero angles accordingly.
3. Adjust sign if joint moves in wrong direction.

---

## Inverse Kinematics

The IK uses a stable atan2-based solution:

```
q1 = atan2(-y, x)
q3 = atan2(±sqrt(1 - c3²), c3)
q2 = alpha - beta
```

Advantages:

* Stable near singularities
* Supports elbow-up and elbow-down configurations
* No acos flipping issues

---

## Safety Notes

* Always power servos from external supply.
* Avoid commanding unreachable targets.
* Ensure mechanical limits match software limits.

---

## Future Improvements

Possible extensions:

* Cartesian linear interpolation between waypoints
* Trapezoidal acceleration profiles
* Closed-loop control with encoders
* Serial command interface (G-code style)

---

## License

Personal educational project.

---

Developed for Arduino Nano 3DOF robotic arm experimentation.
