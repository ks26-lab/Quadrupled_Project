# Simulink Models Directory

This directory is designated for Simulink models (`.slx`, `.mdl`) for the Quadruped Body-Pitch Control Project.

## Path Integration
All models placed in this directory are automatically added to the MATLAB search path by `01_SETUP/project_paths.m`.

## Model Pre-load Requirements
Before simulating pitch control Simulink models, ensure required model parameters are loaded by calling:
```matlab
simulink_init
```
or initializing the project via `project_setup`.
