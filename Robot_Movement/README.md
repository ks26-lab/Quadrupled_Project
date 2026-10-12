# Robot Movement Subsystem Architecture

This directory contains physical robot dynamics, joint actuation, contact mechanics, and terrain interaction modules:

- `Robot_Parameters/`: Physical geometry, inertia parameters, mass distribution (`get_quadruped_geometry.m`).
- `Robot_Kinematics/`: Forward kinematics, leg Jacobian formulations.
- `Robot_Dynamics/`: Full rigid-body floating-base dynamics.
- `Joint_Models/`: Actuator torque/position transformation models.
- `Contact_Model/`: Ground reaction force (GRF) contact models.
- `Terrain_Model/`: Uneven, compliant, and slope terrain representations.
- `Results/`: Kinematic and dynamic verification outputs.
