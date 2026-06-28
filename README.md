cat << 'EOF' > README.md
# Autonomous UAV Interceptor: 3D Proportional Navigation

## Overview

This repository contains the flight control, perception, and guidance software for an autonomous UAV interceptor. Developed using **ROS 2** and **MAVROS**, the system is designed to track and kinetically intercept highly maneuverable fixed-wing targets (such as Quadplanes) in full 3D space.

The core achievement of this project is the successful implementation of **Earth-Frame Proportional Navigation (PN)**, allowing a slower interceptor to mathematically "cut the corner" and intercept a faster circling target, solving the classic "Hound and Hare" aerospace geometry problem.

### 🎥 Demonstration

[Link to your LinkedIn Video or YouTube Demo goes here]

## System Architecture

The project utilizes a decoupled, high-reliability software architecture:

* **ArduPilot SITL:** Simulates the physical flight dynamics, aerodynamics, and momentum of the interceptor and target UAVs.

* **ROS 2 / MAVROS:** Handles high-frequency telemetry streaming (Pose/Velocity) and asynchronous command publishing via `geometry_msgs/Twist`.

* **Core Kinematics (`interceptor_core`):** Pure-Python mathematical libraries for calculating Line-of-Sight (LOS) vectors, closing velocities, and proportional lateral accelerations. Fully unit-tested using `pytest`.

* **Vision & Perception:** Translates 2D pixel coordinates from a camera feed into physical 3D Line-of-Sight angles.

* **3D Visualization (`visualization_node`):** A dedicated ROS 2 node that translates live telemetry into `nav_msgs/Path` and 3D `visualization_msgs/Marker` meshes for real-time engagement geometry rendering in **RViz2**.

## Guidance Methodology

### The Problem: Pure Pursuit vs. Circling Targets

Initial testing utilized a **Pure Pursuit** guidance law. Against a target in a loitering circle (QLOITER), this resulted in a continuous tail-chase. Because the simulated quadcopter's physical top speed (limited by air resistance and tilt angle) was only marginally faster than the fixed-wing target's cruise speed, the intercept required excessive time and resulted in a massive spiral trajectory.

### The Solution: Proportional Navigation (PN)

To achieve a rapid kinetic intercept, the guidance law was upgraded to **Proportional Navigation**.

1. **Line-of-Sight (LOS) Rate:** The system continuously calculates the derivative of the 3D vector between the interceptor and the target ($\dot{\lambda}$).

2. **Lateral Acceleration:** A lateral acceleration vector is generated proportional to the closing velocity and the LOS rate ($a = N \cdot V_c \cdot \dot{\lambda}$).

3. **Earth-Frame Translation:** This acceleration is blended with the pursuit vector to command the drone's 3D Earth-Frame velocities ($V_x, V_y, V_z$).

By relying on ArduCopter's native `WP_YAW_BEHAVIOR`, manual yaw override is bypassed. The flight controller smoothly aligns the airframe with the commanded PN velocity vector, successfully cutting across the target's turning circle and reducing intercept time from minutes to seconds.

## Repository Structure
