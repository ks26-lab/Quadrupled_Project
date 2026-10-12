%% =========================================================================
% QUADRUPED ROBOT BODY-PITCH ATTITUDE CONTROL
% PHASE 5 - HARDWARE-REALISTIC SISO SIMULATION
%
% Architecture : Single-Input Single-Output (SISO) Body-Pitch Control Loop
% Controlled Var : Actual body pitch angle theta(t) [rad]
% Reference      : Desired body pitch angle theta_d(t) [rad]
% Feedback       : IMU body-pitch measurement theta_m(t) [rad]
% Control Action : Net body-pitch control torque Tc(t) [N.m]
% Actuator Plant : J*theta_ddot + B*theta_dot + K*theta = Tc(t) + Td(t)
%
% Objective:
%   Progressively introduce realistic implementation non-idealities to the
%   frozen Phase 4 PI controller:
%     Case 0 : Ideal continuous baseline
%     Case 1 : IMU measurement noise
%     Case 2 : IMU discrete sampling (Sample-and-Hold)
%     Case 3 : Low-pass measurement filtering
%     Case 4 : Actuator torque saturation
%     Case 5 : Actuator rate / slew limitation
%     Case 6 : Combined sensor + actuator realistic scenario
%     Case 7 : Combined realistic scenario with external disturbance
%     Case 8 : Anti-windup diagnostic comparison under severe saturation
%
% CRITICAL PRINCIPLES:
%   - The Phase 4 controller gains (Kp_final, Ki_final) remain strictly FROZEN.
%   - Plant parameters (J, B, K) come from get_robot_parameters().
%   - All non-ideality parameters are clearly labeled as simulation scenario assumptions.
% =========================================================================

ensure_clean_env(false);

print_banner('Phase 5 - Hardware-Realistic Simulation', ...
    'SISO Quadruped Body-Pitch Attitude Control');

%% =========================================================================
% SECTION 1: LOAD FROZEN CONTROLLER & AUTHORITATIVE PLANT
% =========================================================================

paths = project_paths();
frozen_file = fullfile(paths.data, 'processed', 'frozen_pi_controller.mat');

if ~exist(frozen_file, 'file')
    fprintf('1. Frozen controller file not found at: %s\n', frozen_file);
    fprintf('   Executing Phase 4 to design, select, and freeze controller...\n');
    run_phase_4();
end

if ~exist(frozen_file, 'file')
    error(['CRITICAL ERROR: Unable to load frozen controller file: %s. ', ...
           'Phase 5 execution halted.'], frozen_file);
end

% Load frozen controller data
frozen_data = load(frozen_file);
Kp_final = frozen_data.Kp_final;
Ki_final = frozen_data.Ki_final;
C_final  = frozen_data.C_final;
info_final = frozen_data.info_final;

% Retrieve authoritative physical parameters and plant transfer function
params = get_robot_parameters();
J = params.J;
B = params.B;
K = params.K;
[G, sys_ss, s] = get_pitch_plant(params);

fprintf('1. Loaded Frozen Controller & Authoritative Plant:\n');
fprintf('   - Kp_final                   = %.4f\n', Kp_final);
fprintf('   - Ki_final                   = %.4f\n', Ki_final);
fprintf('   - Controller C_final(s)      = (%.4f*s + %.4f)/s\n', Kp_final, Ki_final);
fprintf('   - Plant Inertia (J)          = %.4f kg.m^2\n', J);
fprintf('   - Joint Damping (B)          = %.4f N.m.s/rad\n', B);
fprintf('   - Restoring Stiffness (K)    = %.4f N.m/rad\n', K);
fprintf('   - Plant G(s)                 = 1 / (%.4f*s^2 + %.4f*s + %.4f)\n\n', J, B, K);

%% =========================================================================
% SECTION 2: CENTRALIZED PHASE 5 CONFIGURATION STRUCTURE
% =========================================================================
% All hardware-realism parameters are centralized here as clearly identified
% simulation scenario assumptions (not fabricated hardware specifications).

p5_config = struct();

% Time configuration
p5_config.t_final   = 10.0;    % Simulation duration [s]
p5_config.dt_sim    = 0.001;   % Fine simulation step size [s] (1000 Hz integration)
p5_config.time      = (0:p5_config.dt_sim:p5_config.t_final)';
N_steps             = length(p5_config.time);

% Reference command (Unit step pitch command: 1.0 rad)
p5_config.ref_val   = 1.0;
p5_config.ref_step  = p5_config.ref_val * ones(N_steps, 1);

% Sensor Noise Scenario Assumption
% (Realistic representation of MEMS IMU pitch angle estimation noise)
p5_config.noise_sigma   = 0.005;   % [rad] (approx 0.286 deg standard deviation)
p5_config.noise_seed    = 42;      % Fixed seed for reproducible stochastic simulations

% Sensor Sampling Scenario Assumption
% (Typical robotic IMU / flight controller SPI bus update rate)
p5_config.f_imu         = 100.0;   % IMU sampling frequency [Hz]
p5_config.Ts_imu        = 1.0 / p5_config.f_imu; % Sampling period [s] (0.01 s = 10 ms)

% Measurement Filter Scenario Assumption
% (First-order Butterworth/RC low-pass anti-aliasing / filtering)
p5_config.filter_fc_hz  = 15.0;    % Cutoff frequency [Hz]
p5_config.filter_wc     = 2 * pi * p5_config.filter_fc_hz; % [rad/s]

% Actuator Torque Saturation Scenario Assumption
% (Symmetric peak motor torque limit; nominal unconstrained peak is ~0.50 N.m)
p5_config.u_sat_max     = 0.40;    % Peak torque limit [N.m]

% Actuator Torque Slew Rate Limit Scenario Assumption
% (Motor torque build-up / current loop slew limit)
p5_config.R_max         = 5.0;     % Maximum torque rate [N.m/s]

% External Disturbance Scenario Assumption
% (Stance-phase ground contact disturbance torque pulse, following Phase 4.5)
p5_config.dist_mag      = 0.30;    % Disturbance torque magnitude [N.m]
p5_config.dist_t_start  = 5.0;     % Start time [s]
p5_config.dist_duration = 0.5;     % Duration [s] (applied from 5.0 s to 5.5 s)

fprintf('2. Centralized Hardware-Realism Configuration:\n');
fprintf('   - Integration step (dt_sim)  : %.4f s (f_sim = %d Hz)\n', p5_config.dt_sim, round(1/p5_config.dt_sim));
fprintf('   - IMU Noise std (sigma)      : %.4f rad (%.3f deg) [Assumption]\n', p5_config.noise_sigma, rad2deg(p5_config.noise_sigma));
fprintf('   - IMU Sampling rate (f_imu)  : %.1f Hz (Ts = %.3f s) [Assumption]\n', p5_config.f_imu, p5_config.Ts_imu);
fprintf('   - Low-pass filter cutoff     : %.1f Hz (wc = %.2f rad/s) [Assumption]\n', p5_config.filter_fc_hz, p5_config.filter_wc);
fprintf('   - Actuator Saturation (u_max): +/- %.2f N.m [Assumption]\n', p5_config.u_sat_max);
fprintf('   - Actuator Rate Limit (R_max): %.2f N.m/s [Assumption]\n', p5_config.R_max);
fprintf('   - Disturbance pulse (Td)     : %.2f N.m from %.1f s to %.1f s [Phase 4.5]\n\n', ...
    p5_config.dist_mag, p5_config.dist_t_start, p5_config.dist_t_start + p5_config.dist_duration);

%% =========================================================================
% SECTION 3: REPRODUCIBLE STOCHASTIC SENSOR NOISE GENERATION
% =========================================================================

rng(p5_config.noise_seed);
noise_vector = p5_config.noise_sigma * randn(N_steps, 1);
noise_rms    = sqrt(mean(noise_vector.^2));
noise_peak   = max(abs(noise_vector));

fprintf('3. Precomputed Stochastic Noise Vector:\n');
fprintf('   - Random Seed                : %d\n', p5_config.noise_seed);
fprintf('   - Evaluated Noise RMS        : %.6f rad (%.4f deg)\n', noise_rms, rad2deg(noise_rms));
fprintf('   - Evaluated Peak Noise       : %.6f rad (%.4f deg)\n\n', noise_peak, rad2deg(noise_peak));

%% =========================================================================
% SECTION 4: SIMULATION CASES EVALUATION
% =========================================================================
% Each case is simulated using the exact non-linear time-domain simulation
% engine implemented in local helper simulate_siso_loop().

case_names = { ...
    'Case 0: Ideal Baseline', ...
    'Case 1: Sensor Noise', ...
    'Case 2: IMU Sampling', ...
    'Case 3: Measurement Filtering', ...
    'Case 4: Actuator Saturation', ...
    'Case 5: Actuator Rate Limit', ...
    'Case 6: Combined Realistic', ...
    'Case 7: Combined Realistic + Disturbance', ...
    'Case 8: Anti-Windup Clamping (Severe Saturation)' ...
};

results_cell = cell(length(case_names), 1);

fprintf('4. Executing Controlled Non-Ideality Experiment Progression...\n');

% --- CASE 0: IDEAL CONTINUOUS BASELINE ---
cfg0 = get_default_sim_flags();
results_cell{1} = simulate_siso_loop(p5_config, cfg0, Kp_final, Ki_final, params, noise_vector);
fprintf('   [x] Case 0 completed: Ideal continuous baseline.\n');

% --- CASE 1: SENSOR NOISE ONLY ---
cfg1 = get_default_sim_flags();
cfg1.enable_noise = true;
results_cell{2} = simulate_siso_loop(p5_config, cfg1, Kp_final, Ki_final, params, noise_vector);
fprintf('   [x] Case 1 completed: IMU measurement noise.\n');

% --- CASE 2: IMU SAMPLING ONLY ---
cfg2 = get_default_sim_flags();
cfg2.enable_sampling = true;
results_cell{3} = simulate_siso_loop(p5_config, cfg2, Kp_final, Ki_final, params, noise_vector);
fprintf('   [x] Case 2 completed: IMU sampling (100 Hz ZOH).\n');

% --- CASE 3: MEASUREMENT FILTERING ONLY ---
cfg3 = get_default_sim_flags();
cfg3.enable_filter = true;
results_cell{4} = simulate_siso_loop(p5_config, cfg3, Kp_final, Ki_final, params, noise_vector);
fprintf('   [x] Case 3 completed: Low-pass measurement filtering (15 Hz).\n');

% --- CASE 4: ACTUATOR SATURATION ONLY ---
cfg4 = get_default_sim_flags();
cfg4.enable_saturation = true;
results_cell{5} = simulate_siso_loop(p5_config, cfg4, Kp_final, Ki_final, params, noise_vector);
fprintf('   [x] Case 4 completed: Actuator torque saturation (+/- 0.40 N.m).\n');

% --- CASE 5: ACTUATOR RATE LIMITATION ONLY ---
cfg5 = get_default_sim_flags();
cfg5.enable_rate_limit = true;
results_cell{6} = simulate_siso_loop(p5_config, cfg5, Kp_final, Ki_final, params, noise_vector);
fprintf('   [x] Case 5 completed: Actuator rate limit (5.0 N.m/s).\n');

% --- CASE 6: COMBINED REALISTIC SCENARIO ---
cfg6 = get_default_sim_flags();
cfg6.enable_noise      = true;
cfg6.enable_sampling   = true;
cfg6.enable_filter     = true;
cfg6.enable_saturation = true;
cfg6.enable_rate_limit = true;
results_cell{7} = simulate_siso_loop(p5_config, cfg6, Kp_final, Ki_final, params, noise_vector);
fprintf('   [x] Case 6 completed: Combined sensor + actuator non-idealities.\n');

% --- CASE 7: COMBINED REALISTIC + DISTURBANCE ---
cfg7 = cfg6;
cfg7.enable_disturbance = true;
results_cell{8} = simulate_siso_loop(p5_config, cfg7, Kp_final, Ki_final, params, noise_vector);
fprintf('   [x] Case 7 completed: Combined realistic scenario with external disturbance.\n');

% --- CASE 8: ANTI-WINDUP DIAGNOSTIC (Severe Saturation u_max = 0.30 N.m) ---
cfg8 = get_default_sim_flags();
cfg8.enable_saturation = true;
cfg8.enable_anti_windup = true;
p5_config_severe = p5_config;
p5_config_severe.u_sat_max = 0.30; % Severe torque limit to emphasize windup
results_cell{9} = simulate_siso_loop(p5_config_severe, cfg8, Kp_final, Ki_final, params, noise_vector);
% Baseline severe saturation without anti-windup for comparison:
cfg8_nowindup = cfg8;
cfg8_nowindup.enable_anti_windup = false;
res_severe_nowindup = simulate_siso_loop(p5_config_severe, cfg8_nowindup, Kp_final, Ki_final, params, noise_vector);
fprintf('   [x] Case 8 completed: Anti-windup diagnostic evaluation.\n\n');

%% =========================================================================
% SECTION 5: METRICS EXTRACTION & COMPARISON TABLE
% =========================================================================

num_cases = length(case_names);
table_rows = cell(num_cases, 12);

for i = 1:num_cases
    res = results_cell{i};
    m = res.metrics;
    table_rows(i, :) = { ...
        case_names{i}, ...
        m.RiseTime, ...
        m.SettlingTime, ...
        m.Overshoot, ...
        m.SteadyStateError, ...
        m.PeakError, ...
        m.RMS_Error, ...
        m.MaxControlEffort, ...
        m.RMS_ControlEffort, ...
        m.PctTimeSaturated, ...
        m.RateLimitEvents, ...
        m.DisturbanceRecoveryTime ...
    };
end

col_headers = { ...
    'Case', 'RiseTime_s', 'SettlingTime_s', 'Overshoot_pct', ...
    'SteadyStateError_rad', 'PeakError_rad', 'RMS_Error_rad', ...
    'MaxTorque_Nm', 'RMSTorque_Nm', 'PctSaturated', ...
    'RateLimitEvents', 'RecoveryTime_s' ...
};
comparison_table = cell2table(table_rows, 'VariableNames', col_headers);

fprintf('5. Quantitative Summary of Hardware-Realistic Simulations:\n');
disp(comparison_table(:, 1:9));
disp(comparison_table(:, [1, 10:12]));

%% =========================================================================
% SECTION 6: SAVE ARTIFACTS (TABLES & RESULTS STRUCT)
% =========================================================================

tables_dir = fullfile(paths.results, 'tables');
if ~exist(tables_dir, 'dir'), mkdir(tables_dir); end

exported_dir = fullfile(paths.results, 'exported');
if ~exist(exported_dir, 'dir'), mkdir(exported_dir); end

% Export CSV comparison table
writetable(comparison_table, fullfile(tables_dir, 'phase5_performance_comparison.csv'));

% Build complete persistent result structure
phase5_results = struct();
phase5_results.config              = p5_config;
phase5_results.Kp_frozen           = Kp_final;
phase5_results.Ki_frozen           = Ki_final;
phase5_results.comparison_table    = comparison_table;
phase5_results.case0_ideal         = results_cell{1};
phase5_results.case1_noise         = results_cell{2};
phase5_results.case2_sampling      = results_cell{3};
phase5_results.case3_filter        = results_cell{4};
phase5_results.case4_saturation    = results_cell{5};
phase5_results.case5_rate_limit    = results_cell{6};
phase5_results.case6_combined      = results_cell{7};
phase5_results.case7_disturbed     = results_cell{8};
phase5_results.case8_anti_windup   = results_cell{9};
phase5_results.severe_no_antiwindup= res_severe_nowindup;
phase5_results.date_executed       = datestr(now);

save(fullfile(exported_dir, 'phase5_simulation_results.mat'), 'phase5_results');

fprintf('6. Results Saved Successfully:\n');
fprintf('   - CSV Table : %s\n', fullfile(tables_dir, 'phase5_performance_comparison.csv'));
fprintf('   - Data MAT  : %s\n\n', fullfile(exported_dir, 'phase5_simulation_results.mat'));

%% =========================================================================
% SECTION 7: GENERATE 16 HARDWARE-REALISTIC FIGURES
% =========================================================================

fig_dir = fullfile(paths.results, 'figures');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

t = p5_config.time;
res0 = results_cell{1};
res1 = results_cell{2};
res2 = results_cell{3};
res3 = results_cell{4};
res4 = results_cell{5};
res5 = results_cell{6};
res6 = results_cell{7};
res7 = results_cell{8};

% --- Figure 1: Ideal vs Noisy Measurement ---
f1 = figure('Name', 'Phase 5 - Ideal vs Noisy Measurement', 'Position', [50, 50, 750, 480]);
plot(t, res0.theta, 'k--', 'LineWidth', 2);
hold on;
plot(t, res1.theta_m, 'r-', 'LineWidth', 1);
grid on;
title('Figure 1: Closed-Loop Pitch Angle — Ideal vs IMU Measurement with Noise');
xlabel('Time [s]'); ylabel('\theta(t) [rad]');
legend('Ideal Pitch \theta(t)', 'Measured Pitch \theta_m(t) (\sigma = 0.005 rad)', 'Location', 'southeast');
saveas(f1, fullfile(fig_dir, 'phase5_fig01_ideal_vs_noisy_measurement.png'));

% --- Figure 2: True Pitch vs Measured Pitch (Case 1) ---
f2 = figure('Name', 'Phase 5 - True vs Measured Pitch', 'Position', [70, 70, 750, 480]);
plot(t, res1.theta, 'b-', 'LineWidth', 2);
hold on;
plot(t, res1.theta_m, 'r:', 'LineWidth', 1.2);
grid on;
title('Figure 2: True Plant State \theta(t) vs Measured Signal \theta_m(t) (Case 1)');
xlabel('Time [s]'); ylabel('Body Pitch [rad]');
legend('True Plant State \theta(t)', 'Noisy Measurement \theta_m(t)', 'Location', 'southeast');
saveas(f2, fullfile(fig_dir, 'phase5_fig02_true_vs_measured_pitch.png'));

% --- Figure 3: Sensor Noise Signal ---
f3 = figure('Name', 'Phase 5 - Sensor Noise Signal', 'Position', [90, 90, 750, 450]);
plot(t, noise_vector, 'Color', [0.8 0.2 0.2], 'LineWidth', 1);
grid on;
title(sprintf('Figure 3: Stochastic IMU Pitch Noise n_\\theta(t) (RMS = %.5f rad, Peak = %.5f rad)', noise_rms, noise_peak));
xlabel('Time [s]'); ylabel('Noise n_\theta(t) [rad]');
saveas(f3, fullfile(fig_dir, 'phase5_fig03_sensor_noise_signal.png'));

% --- Figure 4: Ideal vs Sampled Response (Case 2) ---
f4 = figure('Name', 'Phase 5 - Ideal vs Sampled Response', 'Position', [110, 110, 750, 480]);
plot(t, res0.theta, 'k--', 'LineWidth', 2);
hold on;
plot(t, res2.theta, 'b-', 'LineWidth', 1.8);
stairs(t(1:20:1000), res2.theta_m(1:20:1000), 'm-', 'LineWidth', 1.2);
grid on;
title('Figure 4: Ideal vs Sampled IMU Measurement (f_{imu} = 100 Hz, T_s = 10 ms)');
xlabel('Time [s]'); ylabel('\theta(t) [rad]');
legend('Ideal Continuous Response', 'Plant State with Sampled Sensor', 'Discrete ZOH Measurement \theta_m[k]', 'Location', 'southeast');
saveas(f4, fullfile(fig_dir, 'phase5_fig04_ideal_vs_sampled_response.png'));

% --- Figure 5: Filtered vs Unfiltered Measurement (Case 3) ---
f5 = figure('Name', 'Phase 5 - Filtered vs Unfiltered Measurement', 'Position', [130, 130, 750, 480]);
plot(t, res1.theta_m, 'Color', [0.85 0.5 0.5], 'LineWidth', 1);
hold on;
plot(t, res3.theta_m, 'b-', 'LineWidth', 2);
plot(t, res0.theta, 'k--', 'LineWidth', 1.5);
grid on;
title('Figure 5: Low-Pass Filtered IMU Signal vs Raw Noisy Measurement (f_c = 15 Hz)');
xlabel('Time [s]'); ylabel('Pitch Measurement [rad]');
legend('Raw Noisy Measurement', 'Filtered Measurement \theta_f(t)', 'Ideal Plant Pitch \theta(t)', 'Location', 'southeast');
saveas(f5, fullfile(fig_dir, 'phase5_fig05_filtered_vs_unfiltered_measurement.png'));

% --- Figure 6: Ideal vs Saturation Response (Case 4) ---
f6 = figure('Name', 'Phase 5 - Ideal vs Saturation Response', 'Position', [150, 150, 750, 480]);
plot(t, res0.theta, 'k--', 'LineWidth', 2);
hold on;
plot(t, res4.theta, 'r-', 'LineWidth', 2);
grid on;
title('Figure 6: Transient Pitch Response Under Actuator Saturation (u_{max} = 0.40 N.m)');
xlabel('Time [s]'); ylabel('\theta(t) [rad]');
legend('Ideal Linear Response', sprintf('Torque Saturated Response (Sat = %.1f%%)', res4.metrics.PctTimeSaturated), 'Location', 'southeast');
saveas(f6, fullfile(fig_dir, 'phase5_fig06_ideal_vs_saturation_response.png'));

% --- Figure 7: Commanded vs Actual Actuator Torque (Case 4) ---
f7 = figure('Name', 'Phase 5 - Commanded vs Actual Torque', 'Position', [170, 170, 750, 480]);
plot(t, res4.u_cmd, 'r--', 'LineWidth', 1.5);
hold on;
plot(t, res4.u_actual, 'b-', 'LineWidth', 2);
yline(p5_config.u_sat_max, 'k:', 'Upper Saturation Bound (+0.40 N.m)');
yline(-p5_config.u_sat_max, 'k:', 'Lower Saturation Bound (-0.40 N.m)');
grid on;
title('Figure 7: Commanded vs Actual Actuator Torque Under Saturation Limit');
xlabel('Time [s]'); ylabel('Control Torque T_c(t) [N.m]');
legend('Commanded Torque u_{cmd}(t)', 'Applied Torque u_{actual}(t)', 'Location', 'northeast');
saveas(f7, fullfile(fig_dir, 'phase5_fig07_commanded_vs_actual_torque.png'));

% --- Figure 8: Actuator Saturation Indication (Case 4) ---
f8 = figure('Name', 'Phase 5 - Actuator Saturation Indication', 'Position', [190, 190, 750, 420]);
area(t, double(res4.is_saturated), 'FaceColor', [0.9 0.3 0.3], 'EdgeColor', 'none', 'FaceAlpha', 0.6);
ylim([-0.1, 1.2]);
grid on;
title(sprintf('Figure 8: Actuator Saturation Window Indication (Total Duration = %.3f s, %.1f%% of Horizon)', ...
      res4.sat_duration, res4.metrics.PctTimeSaturated));
xlabel('Time [s]'); ylabel('Saturation Active [0/1]');
saveas(f8, fullfile(fig_dir, 'phase5_fig08_actuator_saturation_indication.png'));

% --- Figure 9: Actuator Rate-Limit Indication (Case 5) ---
f9 = figure('Name', 'Phase 5 - Actuator Rate-Limit Indication', 'Position', [210, 210, 750, 450]);
subplot(2, 1, 1);
plot(t, res5.u_actual, 'b-', 'LineWidth', 2);
grid on; title('Figure 9: Actuator Slew Rate Limitation (R_{max} = 5.0 N.m/s)');
ylabel('Torque T_c(t) [N.m]');
subplot(2, 1, 2);
plot(t(1:end-1), abs(diff(res5.u_actual)/p5_config.dt_sim), 'm-', 'LineWidth', 1.5);
yline(p5_config.R_max, 'r--', 'Rate Limit (5.0 N.m/s)');
grid on; xlabel('Time [s]'); ylabel('|du/dt| [N.m/s]');
saveas(f9, fullfile(fig_dir, 'phase5_fig09_actuator_rate_limit_indication.png'));

% --- Figure 10: Ideal vs Realistic Closed-Loop Pitch Response (Case 6) ---
f10 = figure('Name', 'Phase 5 - Ideal vs Realistic Pitch Response', 'Position', [230, 230, 750, 480]);
plot(t, p5_config.ref_step, 'k--', 'LineWidth', 1.2);
hold on;
plot(t, res0.theta, 'k-', 'LineWidth', 2);
plot(t, res6.theta, 'b-', 'LineWidth', 2);
plot(t, res6.theta_m, 'r:', 'LineWidth', 1);
grid on;
title('Figure 10: Closed-Loop Pitch Response — Ideal vs Combined Realistic (Case 6)');
xlabel('Time [s]'); ylabel('\theta(t) [rad]');
legend('Reference \theta_d', 'Ideal Linear Pitch', 'Realistic True Pitch \theta(t)', 'Realistic Sensor Measurement \theta_m(t)', 'Location', 'southeast');
saveas(f10, fullfile(fig_dir, 'phase5_fig10_ideal_vs_realistic_pitch_response.png'));

% --- Figure 11: Realistic Tracking Error (Case 6) ---
f11 = figure('Name', 'Phase 5 - Realistic Tracking Error', 'Position', [250, 250, 750, 450]);
plot(t, res0.error, 'k--', 'LineWidth', 1.5);
hold on;
plot(t, res6.error, 'r-', 'LineWidth', 2);
grid on;
title(sprintf('Figure 11: Tracking Error e(t) = \\theta_d - \\theta (RMS: Ideal=%.4f rad, Realistic=%.4f rad)', ...
      res0.metrics.RMS_Error, res6.metrics.RMS_Error));
xlabel('Time [s]'); ylabel('Tracking Error [rad]');
legend('Ideal Error', 'Realistic Error', 'Location', 'northeast');
saveas(f11, fullfile(fig_dir, 'phase5_fig11_realistic_tracking_error.png'));

% --- Figure 12: Realistic Control Effort (Case 6) ---
f12 = figure('Name', 'Phase 5 - Realistic Control Effort', 'Position', [270, 270, 750, 450]);
plot(t, res0.u_actual, 'k--', 'LineWidth', 1.5);
hold on;
plot(t, res6.u_actual, 'm-', 'LineWidth', 2);
yline(p5_config.u_sat_max, 'r:', 'Saturation Bound (+0.40 N.m)');
yline(-p5_config.u_sat_max, 'r:', 'Saturation Bound (-0.40 N.m)');
grid on;
title(sprintf('Figure 12: Control Torque u(t) (RMS: Ideal=%.3f N.m, Realistic=%.3f N.m)', ...
      res0.metrics.RMS_ControlEffort, res6.metrics.RMS_ControlEffort));
xlabel('Time [s]'); ylabel('Torque T_c(t) [N.m]');
legend('Ideal Control Torque', 'Realistic Actuator Torque', 'Location', 'northeast');
saveas(f12, fullfile(fig_dir, 'phase5_fig12_realistic_control_effort.png'));

% --- Figure 13: Combined Realistic Scenario Multi-Panel (Case 6) ---
f13 = figure('Name', 'Phase 5 - Combined Realistic Multi-Panel', 'Position', [290, 80, 800, 680]);
subplot(3, 1, 1);
plot(t, p5_config.ref_step, 'k--', 'LineWidth', 1.2); hold on;
plot(t, res6.theta, 'b-', 'LineWidth', 2);
plot(t, res6.theta_m, 'r:', 'LineWidth', 1);
grid on; title('Figure 13: Combined Hardware-Realistic Simulation (Case 6)');
ylabel('\theta(t) [rad]'); legend('Reference', 'True Pitch', 'Sensor Measurement', 'Location', 'southeast');
subplot(3, 1, 2);
plot(t, res6.error, 'r-', 'LineWidth', 1.8);
grid on; ylabel('Error e(t) [rad]'); legend('Tracking Error', 'Location', 'northeast');
subplot(3, 1, 3);
plot(t, res6.u_cmd, 'k:', 'LineWidth', 1.2); hold on;
plot(t, res6.u_actual, 'm-', 'LineWidth', 1.8);
grid on; xlabel('Time [s]'); ylabel('Torque [N.m]');
legend('Commanded u_{cmd}', 'Applied u_{actual}', 'Location', 'northeast');
saveas(f13, fullfile(fig_dir, 'phase5_fig13_combined_realistic_scenario.png'));

% --- Figure 14: Realistic Scenario with Disturbance (Case 7) ---
f14 = figure('Name', 'Phase 5 - Realistic Scenario with Disturbance', 'Position', [310, 100, 800, 650]);
subplot(2, 1, 1);
plot(t, p5_config.ref_step, 'k--', 'LineWidth', 1.2); hold on;
plot(t, res0.theta, 'k:', 'LineWidth', 1.5);
plot(t, res7.theta, 'b-', 'LineWidth', 2);
grid on;
title(sprintf('Figure 14: Disturbed Realistic Pitch (Pulse T_d = 0.30 N.m at t=5.0 s, Peak Dev = %.4f rad)', ...
      res7.metrics.PeakDisturbanceDeviation));
ylabel('\theta(t) [rad]'); legend('Reference', 'Undisturbed Ideal', 'Realistic Disturbed', 'Location', 'southeast');
subplot(2, 1, 2);
plot(t, res7.u_actual, 'm-', 'LineWidth', 1.8); hold on;
plot(t, res7.disturbance, 'Color', [0.8 0.2 0.2], 'LineWidth', 1.8);
grid on; xlabel('Time [s]'); ylabel('Torques [N.m]');
legend('Applied Actuator Torque u(t)', 'External Disturbance T_d(t)', 'Location', 'northeast');
saveas(f14, fullfile(fig_dir, 'phase5_fig14_realistic_scenario_with_disturbance.png'));

% --- Figure 15: Metric Comparison Ideal vs Realistic Bar Chart ---
f15 = figure('Name', 'Phase 5 - Metric Comparison Bar Chart', 'Position', [330, 250, 800, 480]);
bar_data = [res0.metrics.RiseTime, res6.metrics.RiseTime; ...
            res0.metrics.SettlingTime, res6.metrics.SettlingTime; ...
            res0.metrics.Overshoot, res6.metrics.Overshoot; ...
            res0.metrics.RMS_Error * 10, res6.metrics.RMS_Error * 10; ...
            res0.metrics.RMS_ControlEffort, res6.metrics.RMS_ControlEffort];
b = bar(bar_data);
b(1).FaceColor = [0.2 0.4 0.8];
b(2).FaceColor = [0.85 0.3 0.2];
set(gca, 'XTickLabel', {'Rise Time [s]', 'Settling Time [s]', 'Overshoot [%]', 'RMS Error x10 [rad]', 'RMS Torque [N.m]'});
title('Figure 15: Quantitative Performance Degradation: Ideal vs Combined Realistic');
legend('Ideal Baseline (Case 0)', 'Combined Realistic (Case 6)', 'Location', 'northeast');
grid on;
saveas(f15, fullfile(fig_dir, 'phase5_fig15_metric_comparison_ideal_vs_realistic.png'));

% --- Figure 16: Anti-Windup Diagnostic Comparison (Case 8) ---
f16 = figure('Name', 'Phase 5 - Anti-Windup Diagnostic', 'Position', [350, 200, 800, 550]);
subplot(2, 1, 1);
plot(t, res_severe_nowindup.theta, 'r--', 'LineWidth', 2); hold on;
plot(t, results_cell{9}.theta, 'b-', 'LineWidth', 2);
grid on;
title('Figure 16: Anti-Windup Compensation Under Severe Torque Saturation (u_{max} = 0.30 N.m)');
ylabel('\theta(t) [rad]');
legend(sprintf('Standard PI (Overshoot = %.1f%%)', res_severe_nowindup.metrics.Overshoot), ...
       sprintf('PI with Anti-Windup (Overshoot = %.1f%%)', results_cell{9}.metrics.Overshoot), ...
       'Location', 'southeast');
subplot(2, 1, 2);
plot(t, res_severe_nowindup.u_actual, 'r--', 'LineWidth', 1.8); hold on;
plot(t, results_cell{9}.u_actual, 'b-', 'LineWidth', 1.8);
yline(p5_config_severe.u_sat_max, 'k:'); yline(-p5_config_severe.u_sat_max, 'k:');
grid on; xlabel('Time [s]'); ylabel('Actuator Torque [N.m]');
legend('Standard PI Torque', 'Anti-Windup Torque', 'Location', 'northeast');
saveas(f16, fullfile(fig_dir, 'phase5_fig16_anti_windup_diagnostic.png'));

fprintf('7. All 16 Hardware-Realistic Engineering Figures Saved to 06_RESULTS/figures/\n');

print_banner('Phase 5 Simulation & Validation Complete', ...
    'Results and figures persisted to 06_RESULTS/ and 07_DATA/');

%% =========================================================================
% LOCAL HELPER FUNCTIONS (SIMULATION ENGINE & METRICS)
% =========================================================================

function cfg = get_default_sim_flags()
    cfg = struct();
    cfg.enable_noise       = false;
    cfg.enable_sampling    = false;
    cfg.enable_filter      = false;
    cfg.enable_saturation  = false;
    cfg.enable_rate_limit  = false;
    cfg.enable_anti_windup = false;
    cfg.enable_disturbance = false;
end

function out = simulate_siso_loop(p5_config, sim_flags, Kp, Ki, plant_params, noise_vector)
    % Non-linear forward time-domain integration of SISO Quadruped Body-Pitch Loop
    dt   = p5_config.dt_sim;
    time = p5_config.time;
    N    = length(time);
    
    J = plant_params.J;
    B = plant_params.B;
    K = plant_params.K;
    
    % Preallocate time-domain signals
    theta_arr       = zeros(N, 1);
    omega_arr       = zeros(N, 1);
    theta_m_arr     = zeros(N, 1);
    error_arr       = zeros(N, 1);
    u_cmd_arr       = zeros(N, 1);
    u_actual_arr    = zeros(N, 1);
    dist_arr        = zeros(N, 1);
    is_sat_arr      = false(N, 1);
    is_rate_arr     = false(N, 1);
    
    % States initialization
    theta = 0.0;
    omega = 0.0;
    I_state = 0.0;             % Controller integrator state
    u_prev = 0.0;              % Previous actuator command for rate limiting
    theta_sampled = 0.0;       % ZOH sampled pitch measurement
    theta_filtered = 0.0;      % Filtered pitch measurement
    
    sample_interval_steps = round(p5_config.Ts_imu / dt);
    if sample_interval_steps < 1, sample_interval_steps = 1; end
    
    wc = p5_config.filter_wc;
    alpha_filter = dt * wc / (1 + dt * wc); % Discretized 1st-order low-pass
    
    ref_val = p5_config.ref_val;
    
    sat_events_count = 0;
    rate_events_count = 0;
    prev_sat = false;
    prev_rate = false;
    
    for k = 1:N
        t_now = time(k);
        
        % 1. Sensor measurement with optional noise
        if sim_flags.enable_noise
            theta_raw = theta + noise_vector(k);
        else
            theta_raw = theta;
        end
        
        % 2. Discrete IMU sampling (Zero-Order Hold)
        if sim_flags.enable_sampling
            if mod(k - 1, sample_interval_steps) == 0
                theta_sampled = theta_raw;
            end
        else
            theta_sampled = theta_raw;
        end
        
        % 3. Low-pass measurement filtering
        if sim_flags.enable_filter
            theta_filtered = (1 - alpha_filter) * theta_filtered + alpha_filter * theta_sampled;
            theta_meas = theta_filtered;
        else
            theta_meas = theta_sampled;
        end
        
        % 4. Error computation
        err = ref_val - theta_meas;
        
        % 5. PI Controller command computation
        u_cmd = Kp * err + I_state;
        
        % 6. Actuator Rate Limiting
        u_rate = u_cmd;
        rate_active = false;
        if sim_flags.enable_rate_limit
            delta_u = u_cmd - u_prev;
            max_delta = p5_config.R_max * dt;
            if delta_u > max_delta
                u_rate = u_prev + max_delta;
                rate_active = true;
            elseif delta_u < -max_delta
                u_rate = u_prev - max_delta;
                rate_active = true;
            end
        end
        
        % 7. Actuator Torque Saturation
        u_sat = u_rate;
        sat_active = false;
        if sim_flags.enable_saturation
            if u_rate > p5_config.u_sat_max
                u_sat = p5_config.u_sat_max;
                sat_active = true;
            elseif u_rate < -p5_config.u_sat_max
                u_sat = -p5_config.u_sat_max;
                sat_active = true;
            end
        end
        
        % 8. Integrator state update with anti-windup clamping
        delta_I = Ki * dt * err;
        if sim_flags.enable_anti_windup && sim_flags.enable_saturation
            % Clamping: freeze integration if saturated in direction of error
            if (sat_active && u_cmd > p5_config.u_sat_max && err > 0) || ...
               (sat_active && u_cmd < -p5_config.u_sat_max && err < 0)
                % Freeze integration
            else
                I_state = I_state + delta_I;
            end
        else
            % Standard unconstrained integration
            I_state = I_state + delta_I;
        end
        
        % 9. External disturbance
        Td = 0.0;
        if sim_flags.enable_disturbance
            if t_now >= p5_config.dist_t_start && t_now <= (p5_config.dist_t_start + p5_config.dist_duration)
                Td = p5_config.dist_mag;
            end
        end
        
        % Log signals
        theta_arr(k)    = theta;
        omega_arr(k)    = omega;
        theta_m_arr(k)  = theta_meas;
        error_arr(k)    = err;
        u_cmd_arr(k)    = u_cmd;
        u_actual_arr(k) = u_sat;
        dist_arr(k)     = Td;
        is_sat_arr(k)   = sat_active;
        is_rate_arr(k)  = rate_active;
        
        % Track event transitions
        if sat_active && ~prev_sat, sat_events_count = sat_events_count + 1; end
        if rate_active && ~prev_rate, rate_events_count = rate_events_count + 1; end
        prev_sat = sat_active;
        prev_rate = rate_active;
        u_prev = u_sat;
        
        % 10. Plant state forward integration (4th-order Runge-Kutta step)
        % Dynamics: d(theta)/dt = omega, d(omega)/dt = (u_sat + Td - B*omega - K*theta) / J
        Tc_total = u_sat + Td;
        
        k1_th = omega;
        k1_om = (Tc_total - B * omega - K * theta) / J;
        
        th_mid1 = theta + 0.5 * dt * k1_th;
        om_mid1 = omega + 0.5 * dt * k1_om;
        k2_th = om_mid1;
        k2_om = (Tc_total - B * om_mid1 - K * th_mid1) / J;
        
        th_mid2 = theta + 0.5 * dt * k2_th;
        om_mid2 = omega + 0.5 * dt * k2_om;
        k3_th = om_mid2;
        k3_om = (Tc_total - B * om_mid2 - K * th_mid2) / J;
        
        th_end = theta + dt * k3_th;
        om_end = omega + dt * k3_om;
        k4_th = om_end;
        k4_om = (Tc_total - B * om_end - K * th_end) / J;
        
        theta = theta + (dt / 6) * (k1_th + 2*k2_th + 2*k3_th + k4_th);
        omega = omega + (dt / 6) * (k1_om + 2*k2_om + 2*k3_om + k4_om);
    end
    
    % Compute standard step metrics using stepinfo
    s_info = stepinfo(theta_arr, time, ref_val);
    
    % Custom performance metrics
    ss_indices = round(0.8 * N):N; % Final 20% of simulation for steady state
    e_ss = abs(ref_val - mean(theta_arr(ss_indices)));
    e_peak = max(abs(ref_val - theta_arr));
    e_rms = sqrt(mean((ref_val - theta_arr).^2));
    u_max = max(abs(u_actual_arr));
    u_rms = sqrt(mean(u_actual_arr.^2));
    
    pct_sat = 100 * sum(is_sat_arr) / N;
    sat_duration = sum(is_sat_arr) * dt;
    
    % Disturbance recovery time
    rec_time = NaN;
    peak_dev = NaN;
    if sim_flags.enable_disturbance
        idx_dist = find(time >= p5_config.dist_t_start, 1);
        dist_theta = theta_arr(idx_dist:end);
        peak_dev = max(abs(dist_theta - ref_val));
        % Recovery within 2% band of ref_val
        within_band = abs(dist_theta - ref_val) <= 0.02 * ref_val;
        last_outside = find(~within_band, 1, 'last');
        if isempty(last_outside)
            rec_time = 0.0;
        else
            rec_time = time(idx_dist + last_outside) - p5_config.dist_t_start;
        end
    end
    
    % Assemble output structure
    out = struct();
    out.time         = time;
    out.theta        = theta_arr;
    out.omega        = omega_arr;
    out.theta_m      = theta_m_arr;
    out.error        = error_arr;
    out.u_cmd        = u_cmd_arr;
    out.u_actual     = u_actual_arr;
    out.disturbance  = dist_arr;
    out.is_saturated = is_sat_arr;
    out.is_rate_lim  = is_rate_arr;
    out.sat_duration = sat_duration;
    
    out.metrics = struct( ...
        'RiseTime', s_info.RiseTime, ...
        'SettlingTime', s_info.SettlingTime, ...
        'Overshoot', s_info.Overshoot, ...
        'Peak', s_info.Peak, ...
        'SteadyStateError', e_ss, ...
        'PeakError', e_peak, ...
        'RMS_Error', e_rms, ...
        'MaxControlEffort', u_max, ...
        'RMS_ControlEffort', u_rms, ...
        'PctTimeSaturated', pct_sat, ...
        'SaturationEvents', sat_events_count, ...
        'RateLimitEvents', rate_events_count, ...
        'DisturbanceRecoveryTime', rec_time, ...
        'PeakDisturbanceDeviation', peak_dev ...
    );
end
