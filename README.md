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
    { {0.11f,  0.00f, 0.09f}, 0.5f },
    { {0.10f,  0.03f, 0.11f}, 0.5f },
    { {0.08f, -0.03f, 0.06f}, 0.2f }
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

Serial Monitor: Permitted coordinates (depending on `h`, `L1`, `L2`)

Entries in the Serial Monitor are **absolute target points in the base coordinate system** in **metres**:

```
x y z [g]
```

* `x, y, z` in metres
* optional `g` as gripper value from `0.0 .. 1.0`

Relative TCP commands are also possible:

```
t dx dy dz [g]
```

Here, `(dx,dy,dz)` is specified in the current TCP coordinate system and is converted internally to the base system.

### Geometric conditions for valid targets

With

* `r = sqrt(x² + y²)`
* `d = sqrt(r² + (z - h)²)`

the following must apply:

* `R_MIN <= r <= R_MAX`
* `Z_MIN <= z <= Z_MAX`
* `|L1 - L2| <= d <= (L1 + L2)`

This is the combination of the configured cylindrical boundaries and the actual 2-link reachability.

### Current figures from `Config.h`

* `L1 = 0.080 m`
* `L2 = 0.068 m`
* `h  = 0.027 m`
* `R_MIN = 0.050 m`
* `R_MAX = L1 + L2 = 0.148 m`
* `Z_MIN = h - 0.02 = 0.007 m`
* `Z_MAX = h + 0.80*(L1+L2) = 0.1454 m`
* Additional IK ring: `0.012 m <= d <= 0.148 m`

### Examples of permitted entries

* `0.10 0.00 0.08`
* `0.09 0.03 0.11 0.6`
* `t 0.01 0.00 -0.01`

If a target is outside the workspace, the firmware reports `WS ERR` or `IK ERR`.

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
