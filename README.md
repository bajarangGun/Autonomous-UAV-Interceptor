# Autonomous UAV Interceptor: 3D Proportional Navigation

## Overview

This repository contains the flight control, perception, and guidance software for an **autonomous UAV interceptor**. Developed using **ROS 2** and **MAVROS**, the system is designed to track and kinetically intercept highly maneuverable fixed-wing targets (such as Quadplanes) in full **3D space**.

The primary achievement of this project is the successful implementation of **Earth-Frame Proportional Navigation (PN)**, enabling a slower interceptor to mathematically *cut the corner* and intercept a faster circling target. This solves the classic **"Hound and Hare"** aerospace guidance problem.



# System Architecture

The project follows a modular and high-reliability software architecture.

```
                    +-----------------------+
                    |   ArduPilot SITL      |
                    |  (Flight Dynamics)    |
                    +----------+------------+
                               |
                         MAVLink Telemetry
                               |
                      +--------v---------+
                      | ROS 2 + MAVROS   |
                      +--------+---------+
                               |
        +----------------------+----------------------+
        |                      |                      |
        |                      |                      |
+-------v------+      +--------v-------+      +------v------+
| Flight Node  |      | Perception     |      | Visualization|
|              |      | & Guidance     |      |    Node      |
+--------------+      +----------------+      +-------------+
```

---

## Components

### ArduPilot SITL

Simulates the physical flight dynamics, aerodynamics, and momentum of both the interceptor and the target UAV.

---

### ROS 2 + MAVROS

Provides

- High-frequency telemetry streaming
- Pose and velocity estimation
- MAVLink communication
- Velocity command publishing through `geometry_msgs/Twist`

---

### Core Kinematics (`interceptor_core`)

Pure Python mathematical libraries responsible for

- Line-of-Sight (LOS) computation
- Relative position estimation
- Closing velocity calculation
- Proportional Navigation guidance
- Unit-tested using **pytest**

---

### Vision & Perception

Transforms camera pixel coordinates into physical **3D Line-of-Sight** angles for target localization.

---

### 3D Visualization (`visualization_node`)

Publishes

- `nav_msgs/Path`
- `visualization_msgs/Marker`

to RViz2 for real-time visualization of

- UAV trajectories
- Engagement geometry
- Target mesh
- Interceptor path

---

# Guidance Methodology

## The Problem: Pure Pursuit

The initial implementation used a **Pure Pursuit** guidance law.

When the target aircraft performed a continuous loiter (QLOITER), the interceptor simply chased the target from behind.

Since the interceptor's maximum physical speed was only slightly greater than the target's cruise speed, this resulted in

- Continuous tail chase
- Large spiral trajectories
- Very long interception times

---

## The Solution: Proportional Navigation (PN)

To achieve rapid interception, the guidance law was upgraded to **Earth-Frame Proportional Navigation**.

### 1. Line-of-Sight Rate

The system continuously computes the derivative of the Line-of-Sight vector

\[
\dot{\lambda}
\]

---

### 2. Lateral Acceleration

The commanded acceleration is

\[
a = N V_c \dot{\lambda}
\]

where

- \(N\) = Navigation Constant
- \(V_c\) = Closing Velocity
- \(\dot{\lambda}\) = LOS Rate

---

### 3. Earth-Frame Velocity Commands

The computed lateral acceleration is blended with the pursuit vector to produce

- \(V_x\)
- \(V_y\)
- \(V_z\)

commands in the Earth frame.

Instead of manually commanding yaw, the system relies on ArduCopter's native

```
WP_YAW_BEHAVIOR
```

parameter.

The flight controller automatically aligns the UAV with the desired velocity vector, allowing the interceptor to

- Cut across the target's turning circle
- Reduce intercept time dramatically
- Produce realistic missile-like trajectories

---

# Repository Structure

```text
interceptor_project/
│
├── meshes/
│   ├── target.dae
│   ├── interceptor.stl
│   └── ...
│
├── src/
│   ├── interceptor_core/
│   │   ├── guidance.py
│   │   ├── kinematics.py
│   │   ├── perception.py
│   │   └── ...
│   │
│   └── interceptor_ros/
│       ├── interceptor_node.py
│       ├── visualization_node.py
│       └── ...
│
├── tests/
│   ├── test_guidance.py
│   ├── test_camera.py
│   └── ...
│
├── package.xml
├── setup.py
├── pyproject.toml
└── README.md
```

---

# Running the Simulation

## 1. Launch ArduPilot SITL

Launch two SITL instances

- **Drone 0** → Interceptor
- **Drone 1** → Target

---

## 2. Configure Flight Limits

Increase the interceptor agility.

```bash
param set WPNAV_SPEED 2500
param set WPNAV_ACCEL 1000
```

---

## 3. Build ROS 2 Workspace

```bash
colcon build --symlink-install
```

```bash
source install/setup.bash
```

---

## 4. Launch Nodes

Interceptor

```bash
ros2 run interceptor_project interceptor_node
```

Visualization

```bash
ros2 run interceptor_project visualizer_node
```

---

# RViz2 Visualization

Open RViz2

Set

```
Fixed Frame = map
```

Subscribe to

```
/visuals/interceptor_path
```

and

```
/visuals/target_shape
```

to observe

- Live UAV trajectory
- Target mesh
- Path history
- Engagement geometry

---

# Features

- Autonomous UAV interception
- Full 3D Earth-Frame Proportional Navigation
- ROS 2 + MAVROS integration
- ArduPilot SITL support
- Modular architecture
- Real-time RViz2 visualization
- Camera-based perception pipeline
- Unit-tested mathematical guidance libraries
- Easily extendable for future vision algorithms

---

# Technologies Used

- ROS 2
- MAVROS
- MAVLink
- ArduPilot SITL
- Python
- OpenCV
- NumPy
- RViz2
- Gazebo Harmonic
- PyTest

---

# Future Improvements

- Object detection using YOLO
- Multi-target tracking
- Kalman Filter-based target prediction
- Optical flow integration
- GPS-denied interception
- Swarm interception
- Reinforcement Learning guidance
- Real hardware deployment

---

# Open Source & Contributions

This project is open source and intended for

- Academic research
- Robotics development
- UAV guidance research
- Engineering portfolio demonstrations

Contributions are welcome!

Feel free to

- Fork the repository
- Open an issue
- Submit a pull request
- Suggest improvements

---

# License

This project is licensed under the **MIT License**.

See the `LICENSE` file for more details.
