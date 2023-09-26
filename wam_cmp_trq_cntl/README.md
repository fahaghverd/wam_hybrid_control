# Computed Torque Controller for BarrettWAMArm
This repository contains a Computed Torque Controller implementation for a 4 Degree of Freedom (DOF) WAM Arm using the libbarrett library from Barrett Technology. The controller is designed to operate in both joint space and Cartesian space, providing precise control and manipulation capabilities for the WAM Arm. 

## Introduction
The Computed Torque Controller is a feedback-based control approach that aims to achieve accurate tracking of desired trajectories for robotic manipulators. It utilizes a model of the robot dynamics and calculates the desired torques or forces needed to follow the desired trajectory. Here, a it is developed for a 4DOF WAM arm based on libbarret. Libbarrett is a real-time controls library written in C++ that runs Barrett Technology products (WAM and BarrettHand).

## Features
- Support for both joint space and cartesian space control.
- Three types of trajectories generated:
    - Regulation
    - Constant vel. profile
    - Trapezoidal vel. profile
 
## Requirements
- Barrett WAM Arm with 4 DOF
- libbaret (follow install steps [here](https://git.barrett.com/software/libbarrett))

## Contact
If you have any questions or need further assistance, feel free to contact the project maintainers:

Faezeh Haghverd (haghverd@ualberta.ca)
