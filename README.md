Autonomous UAV Interceptor: 3D Proportional Navigation

Overview

This repository contains the flight control and guidance software for an autonomous UAV interceptor. Developed using ROS 2 and MAVROS, the system is designed to track and kinetically intercept highly maneuverable fixed-wing targets (Quadplanes) in full 3D space.

The core achievement of this project is the successful implementation of Earth-Frame Proportional Navigation (PN), allowing a slower interceptor to mathematically "cut the corner" and intercept a faster circling target, solving the classic "Hound and Hare" aerospace geometry problem.

🎥 Demonstration

[Link to your LinkedIn Video or YouTube Demo goes here]

System Architecture

The project utilizes a decoupled, high-reliability software architecture:

ArduPilot SITL: Simulates the physical flight dynamics, aerodynamics, and momentum of an Iris quadcopter (Interceptor) and a Quadplane (Target).

ROS 2 / MAVROS: Handles high-frequency telemetry streaming (Pose/Velocity) and asynchronous command publishing.

Core Kinematics (interceptor_core): Pure-Python mathematical libraries for calculating Line-of-Sight (LOS) vectors, closing velocities, and proportional lateral accelerations.

3D Visualization (visualization_node): A dedicated node that translates live telemetry into nav_msgs/Path and visualization_msgs/Marker data for real-time engagement geometry rendering in RViz2.

Guidance Methodology

The Problem: Pure Pursuit vs. Circling Targets

Initial testing utilized a Pure Pursuit guidance law, where the interceptor's velocity vector was pointed directly at the target's current position. Against a target in a loitering circle (QLOITER), this resulted in a continuous tail-chase. Because the simulated Iris quadcopter's physical top speed (limited by air resistance and tilt angle) was only marginally faster than the fixed-wing target's cruise speed, the intercept required excessive time and resulted in a massive spiral trajectory.

The Solution: Proportional Navigation (PN)

To achieve a rapid kinetic intercept, the guidance law was upgraded to Proportional Navigation.

Line-of-Sight (LOS) Rate: The system continuously calculates the derivative of the 3D vector between the interceptor and the target ($\dot{\lambda}$).

Lateral Acceleration: A lateral acceleration vector is generated proportional to the closing velocity and the LOS rate ($a = N \cdot V_c \cdot \dot{\lambda}$).

Earth-Frame Translation: This acceleration is blended with the pursuit vector to command the drone's 3D Earth-Frame velocities ($V_x, V_y, V_z$).

By utilizing ArduCopter's native WP_YAW_BEHAVIOR, the manual yaw loop was bypassed. The flight controller smoothly aligns the airframe with the commanded PN velocity vector, successfully cutting across the target's turning circle and reducing intercept time from minutes to seconds.

Repository Structure

interceptor_project/
├── archive/                # Deprecated vision-based processing nodes
├── meshes/                 # 3D STL files for RViz2 target visualization
├── src/
│   ├── interceptor_core/   # Pure-math kinematics and guidance libraries
│   └── interceptor_ros/    # ROS 2 Nodes (Truth Node, Visualizer Node)
├── tests/                  # Pytest unit tests for guidance algorithms
├── setup.py                # ROS 2 package configuration
└── pyproject.toml          # Code formatting (Black) and static typing (Mypy)


Running the Simulation

Launch ArduPilot SITL:
Start two instances of ArduPilot SITL (Drone 0: Interceptor, Drone 1: Target).

Configure Flight Limits (Interceptor Console):
Unlock the drone's physical agility to allow high-speed maneuvering:

param set WPNAV_SPEED 2500
param set WPNAV_ACCEL 1000


Launch ROS 2 Nodes:

ros2 run interceptor_project truth_node
ros2 run interceptor_project visualizer_node


Visualize in RViz2:
Open RViz2, set the fixed frame to map, and subscribe to /visuals/interceptor_path and /visuals/target_shape.

Open Source & Contributions

This project is open-source and intended for academic, research, and engineering portfolio purposes. If you are researching UAV swarm dynamics, missile guidance laws, or ROS 2 integration, feel free to fork this repository, open an issue, or submit a pull request!

License

This project is licensed under the MIT License.
