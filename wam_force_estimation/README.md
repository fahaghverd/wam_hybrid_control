# WAM_Force_Estimation
This repository contains a framework for estimating the contact force applied to the WAM Barrett arm using the libbarrett library from Barrett Technology. The framework is based on the arm's dynamic model, and two different methods for obtaining joint accelerations are implemented: differentiating and desgining observer. 

## Introduction
The usual robot dynamic equation, also known as the manipulator equation or the robot dynamic model, describes the relationship between joint torques (or forces) and joint accelerations for a robot manipulator. It is based on the principles of Newton-Euler or Lagrange-Euler formalisms and represents the dynamics of a robot's motion in a compact mathematical form.

The manipulator equation for a robot with n degrees of freedom can be written as:

**$${ \tau = M(q) \ddot{q} + C(q, \dot{q}) \dot{q} + g(q) + J(q)^T F }$$**

- **$\tau$** is the vector of joint torques applied to the robot's joints.
- **$q$** is the vector of joint positions.
- **$\dot{q}$** is the vector of joint velocities.
- **$\ddot{q}$** is the vector of joint accelerations.
- **$M(q)$** is the inertia matrix, an n x n symmetric positive-definite matrix that represents the robot's mass distribution and rotational inertia.
- **$C(q, \dot{q})$** is the matrix of Coriolis and centripetal forces, which are caused by the robot's motion.
- **$g(q)$** is the vector of gravitational forces, representing the effects of gravity on the robot's joints.
- **$J(q)$** is the Jacobian matrix that relates the end-effector velocity to the joint velocities.
- **$F$** is a vector representing external forces or disturbances acting on the robot.

The Jacobian transpose term J(q)^T F allows you to represent the effect of external forces acting at the end-effector and transforming them into joint torques (or forces) using the transpose of the Jacobian matrix.

With the robot's joint positions, velocities, and torques available, and by employing either an observer or differentiator to compute joint accelerations, we can utilize the aforementioned equation to estimate the contact force.

## Installation
To use this package, you need to have the following prerequisites installed on your system:
- C++ Compiler with C++17 support
- CMake (version 3.12 or higher)
- Eigen (linear algebra library)
- libbaret (follow install steps [here](https://git.barrett.com/software/libbarrett))

Follow these steps to use the package:

1. Clone the GitHub repository:
'git clone https://github.com/fahaghverd/wam-barrett-force-estimation.git
cd wam-barrett-force-estimation'

2. Build the project using CMake:
   '''mkdir build
cd build
cmake ..
make'''

## Contact
If you have any questions or need further assistance, feel free to contact the project maintainers:

Faezeh Haghverd (haghverd@ualberta.ca)
