cat << 'EOF' > README.md
# Autonomous UAV Interceptor: 3D Proportional Navigation

## Overview

This repository contains the flight control, perception, and guidance software for an autonomous UAV interceptor. Developed using **ROS 2** and **MAVROS**, the system is designed to track and kinetically intercept highly maneuverable fixed-wing targets (such as Quadplanes) in full 3D space.

The core achievement of this project is the successful implementation of **Earth-Frame Proportional Navigation (PN)**, allowing a slower interceptor to mathematically "cut the corner" and intercept a faster circling target, solving the classic "Hound and Hare" aerospace geometry problem.



## Guidance Methodology


### : Proportional Navigation (PN)

To achieve a rapid kinetic intercept, the guidance law was upgraded to **Proportional Navigation**.

1. **Line-of-Sight (LOS) Rate:** The system continuously calculates the derivative of the 3D vector between the interceptor and the target ($\dot{\lambda}$).

2. **Lateral Acceleration:** A lateral acceleration vector is generated proportional to the closing velocity and the LOS rate ($a = N \cdot V_c \cdot \dot{\lambda}$).

3. **Earth-Frame Translation:** This acceleration is blended with the pursuit vector to command the drone's 3D Earth-Frame velocities ($V_x, V_y, V_z$).

By relying on ArduCopter's native `WP_YAW_BEHAVIOR`, manual yaw override is bypassed. The flight controller smoothly aligns the airframe with the commanded PN velocity vector, successfully cutting across the target's turning circle and reducing intercept time from minutes to seconds.

## Running the Simulation

1. **Launch ArduPilot SITL:**
   Start two instances of ArduPilot SITL (Drone 0: Interceptor, Drone 1: Target).

2. **Configure Flight Limits (Interceptor Console):**
   Unlock the drone's physical agility to allow high-speed maneuvering:
3.  **Build and Launch ROS 2 Nodes:**
4. **Visualize in RViz2:**
Open RViz2, set the fixed frame to `map`, and subscribe to `/visuals/interceptor_path` and `/visuals/target_shape`.

## Open Source & Contributions

This project is open-source and intended for academic, research, and engineering portfolio purposes. If you are researching UAV swarm dynamics, missile guidance laws, or ROS 2 computer vision integration, feel free to fork this repository, open an issue, or submit a pull request!

## License

This project is licensed under the MIT License.
EOF
