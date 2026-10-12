# Processed Data Directory

Stores computed intermediate datasets and frozen candidate configurations for decoupled execution across phases:
- `frozen_pi_controller.mat` : Saved output of Phase 4.4 containing `Kp_final`, `Ki_final`, `C_final`, `T_final`, and `info_final` so Phase 4.5 and downstream phases can run completely independently without recalculating the gain sweep.
