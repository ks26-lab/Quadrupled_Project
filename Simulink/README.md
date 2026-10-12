# Simulink Architecture Specification

This directory holds the multi-subsystem Simulink model and component libraries:

- `Quadruped_Main.slx`: Main top-level integrated Simulink model harness.
- `Gait_Subsystem/`: CPG, gait state machines, and leg trajectory generation blocks.
- `Robot_Subsystem/`: Multibody dynamic model and actuator transfer functions.
- `Balance_Subsystem/`: Discrete-time implementation of the frozen PI pitch controller.
- `Terrain_Subsystem/`: Surface profile, friction models, and obstacle generators.
- `Sensor_Subsystem/`: IMU pitch sensing, rate measurement, noise & quantization blocks.
