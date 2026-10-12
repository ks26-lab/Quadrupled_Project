# Phase 8: Conventional Feedback vs. Machine-Learning Control Benchmark

## 1. Overview
Phase 8 implements an isolated, reproducible comparative evaluation of:
1. **Conventional PI Controller (Frozen Baseline)**: $K_p = 0.50$, $K_i = 0.20$, with anti-windup clamping and actuator torque saturation at $\pm 0.40\,\text{N}\cdot\text{m}$.
2. **LSTM Supervised Recurrent Controller**: Multi-step sequential recurrent neural network with input vector $[e(t), \dot{e}(t), \theta(t)]^T$, 16 hidden LSTM units, gating mechanisms ($i_t, f_t, g_t, o_t$), and state cell memory ($c_t, h_t$).
3. **TD3 / Continuous Policy Controller**: Actor-critic continuous policy network mapping $[e(t), \dot{e}(t), \theta(t), \int e\,dt]^T$ through two 24-neuron hidden layers with ReLU activations and a bounded $\tanh$ output layer scaled to actuator saturation $\pm 0.40\,\text{N}\cdot\text{m}$.

---

## 2. Directory Architecture
```
Quadruped_Body_Pitch_Control/03_PHASES/Phase_8_Learning_Control_Benchmark/
├── run_phase_8.m                    # Master standalone launcher
├── README.md                        # Phase documentation & mathematical formulation
├── controllers/
│   ├── pi_baseline_controller.m     # Conventional PI constructor with anti-windup & saturation
│   ├── pi_step.m                    # Discrete-time execution step for PI
│   ├── lstm_pitch_controller.m      # Recurrent LSTM controller constructor
│   ├── lstm_step.m                  # Recurrent forward inference step for LSTM
│   ├── td3_pitch_controller.m       # Continuous Actor controller constructor
│   └── td3_step.m                   # Deterministic Actor forward evaluation step
├── training/
│   ├── generate_training_data.m     # Independent multi-scenario trajectory generator
│   ├── train_lstm_controller.m      # Sequence gradient trainer (Adam + BPTT)
│   └── train_td3_controller.m       # Policy gradient optimization engine
├── simulation/
│   ├── simulate_closed_loop.m       # Runge-Kutta 4th-order SISO plant simulation engine
│   ├── generate_test_scenarios.m    # 4 rigorous test scenarios (strictly separated from training)
│   ├── compute_benchmark_metrics.m  # Standardized metric extractor (RMSE, IAE, Overshoot, etc.)
│   └── run_benchmark_suite.m        # Multi-controller test harness executor
└── visualization/
    ├── plot_comparative_responses.m # Publication-grade response plotter
    └── export_benchmark_tables.m    # CSV tables and summary exporter
```

---

## 3. Disjoint Training and Testing Scenarios

### Training Distribution
- **Dataset:** 20 randomized multi-step reference profiles with stochastic step transitions between $[-0.10, +0.10]\,\text{rad}$ ($[-5.7^\circ, +5.7^\circ]$).
- **Disturbances:** Randomized transient pulses ($|T_d| \le 0.10\,\text{N}\cdot\text{m}$) across random intervals.
- **Plant Variations:** Small uniform variations in inertia $J \in [0.045, 0.055]\,\text{kg}\cdot\text{m}^2$ and damping $B \in [0.09, 0.11]\,\text{N}\cdot\text{m}\cdot\text{s/rad}$.

### Benchmark Test Scenarios (Never Seen During Training)
1. **Test 1: Dynamic Reference Trajectory Tracking**
   - Reference: $0\,\text{rad} \to +5^\circ (+0.0873\,\text{rad})$ at $t=1\,\text{s}$, to $-3^\circ (-0.0524\,\text{rad})$ at $t=3\,\text{s}$, followed by a $0.5\,\text{Hz}$ continuous sinusoid.
   - External disturbance: None ($T_d = 0$).
2. **Test 2: Sustained Torque Disturbance Rejection**
   - Reference: Pitch stabilization at $\theta_d = 0\,\text{rad}$.
   - Disturbance: Sustained step torque $T_d = +0.15\,\text{N}\cdot\text{m}$ applied from $t=2.0\,\text{s}$ to $4.5\,\text{s}$.
3. **Test 3: Contact Loss / Foot Slip Fault**
   - Reference: $\theta_d = 0\,\text{rad}$.
   - Fault: Sudden severe torque impact $T_d = -0.25\,\text{N}\cdot\text{m}$ from $t=2.0\,\text{s}$ to $3.5\,\text{s}$ with simultaneous 30% reduction in ground damping ($B = 0.070$).
4. **Test 4: Plant Uncertainty & Parameter Mismatch**
   - Reference: $\pm 4^\circ$ alternating steps.
   - Model mismatch: Body pitch inertia $J$ increased by $+35\%$ ($J = 0.0675\,\text{kg}\cdot\text{m}^2$), joint damping $B$ reduced by $-30\%$ ($B = 0.070\,\text{N}\cdot\text{m}\cdot\text{s/rad}$).

---

## 4. How to Execute
From the MATLAB command window:
```matlab
% Run Phase 8 standalone
run_phase_8

% Or via the central project launcher
run_project('8')

% Or via the root forwarder
run_phase_8
```
