%% =========================================================================
% QUADRUPED ROBOT BODY-PITCH ATTITUDE CONTROL
% PHASE 6 (PHASE 4.6) - ONE-LEG CONTACT-LOSS / FAULT ANALYSIS
%
% Architecture : Single-Input Single-Output (SISO) Body-Pitch Control Loop
% Controlled Var : Actual body pitch angle theta(t) [rad]
% Reference      : Desired body pitch angle theta_d(t) [rad]
% Feedback       : IMU body-pitch measurement theta_m(t) [rad]
% Control Action : Net body-pitch control torque Tc(t) [N.m]
% Plant Model    : J*theta_ddot + B*theta_dot + K*theta = Tc,available(t) + Td,contact(t)
%
% Objective:
%   Quantitatively evaluate the stability, transient degradation, recovery
%   capability, and fault-tolerance boundaries of the validated, frozen
%   Phase 4 SISO PI controller under one-leg ground contact loss.
%
% Reduced-Order Modelling Principles:
%   1. Reduced-order SISO equivalent representation:
%      - Corrective torque authority reduction: Tc,available(t) = alpha(t) * Tc(t)
%      - Ground-reaction asymmetry disturbance: Td,contact(t)
%   2. Front-leg loss (FL, FR) produces nose-down pitch bias (negative torque).
%   3. Rear-leg loss (RL, RR) produces nose-up pitch bias (positive torque).
%   4. Left/Right symmetry yields identical pitch dynamics under pure SISO pitch axis.
%
% CRITICAL PRINCIPLES:
%   - The Phase 4 controller gains (Kp_final, Ki_final) remain strictly FROZEN.
%   - No MIMO conversion: contact states are scenario fault inputs, NOT controller states.
%   - Plant parameters (J, B, K) come from get_robot_parameters().
% =========================================================================

ensure_clean_env(false);

print_banner('Phase 6 - One-Leg Contact-Loss / Fault Analysis', ...
    'SISO Quadruped Body-Pitch Attitude Stabilization');

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
           'Phase 6 execution halted.'], frozen_file);
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
% SECTION 2: CENTRALIZED PHASE 6 CONFIGURATION
% =========================================================================

p6_config = struct();

% Time configuration
p6_config.t_final   = 10.0;    % Total simulation horizon [s]
p6_config.dt_sim    = 0.001;   % Fine integration step size [s] (1000 Hz)
p6_config.time      = (0:p6_config.dt_sim:p6_config.t_final)';
N_steps             = length(p6_config.time);

% Reference pitch command: Zero pitch regulation or step command
% Here evaluated under nominal 1.0 rad step reference with fault injected during steady-state
p6_config.ref_val   = 1.0;     % Desired body pitch [rad]

% Fault Timing Configuration
p6_config.t_fault_start = 3.0; % Contact loss onset time [s] (after nominal transient settles)
p6_config.t_fault_dur   = 1.5; % Temporary contact loss duration [s] (recontact at 4.5 s)

% Fault Severity Assumptions (Reduced-Order Modeling Parameters)
% When one leg loses contact:
% 1. Corrective torque authority drops: alpha = 0.75 (25% reduction in available pitch torque)
% 2. Ground reaction force asymmetry induces an equivalent disturbance torque:
%    |Td_contact| = 0.25 N.m
p6_config.alpha_fault   = 0.75; % Torque authority factor during contact loss [Assumption]
p6_config.Td_fault_mag  = 0.25; % GRF asymmetry disturbance torque [N.m] [Assumption]

% Recovery Criterion Configuration
p6_config.recovery_band = 0.02 * p6_config.ref_val; % 2% of setpoint (0.02 rad = 1.15 deg)

% Actuator Saturation Limit (from Phase 5 Hardware-Realism)
p6_config.u_max_sat     = 0.40; % [N.m]
p6_config.R_max_rate    = 5.0;  % [N.m/s]

fprintf('2. Centralized Contact-Loss Scenario Configuration:\n');
fprintf('   - Fault Onset Time (t_fault) : %.2f s\n', p6_config.t_fault_start);
fprintf('   - Temporary Fault Duration   : %.2f s (Recontact at %.2f s)\n', ...
    p6_config.t_fault_dur, p6_config.t_fault_start + p6_config.t_fault_dur);
fprintf('   - Torque Authority (alpha)   : %.2f (%.0f%% remaining) [Assumption]\n', ...
    p6_config.alpha_fault, 100*p6_config.alpha_fault);
fprintf('   - Equivalent GRF Asymmetry Td: +/- %.2f N.m [Assumption]\n', p6_config.Td_fault_mag);
fprintf('   - Recovery Tolerance Band    : +/- %.4f rad (%.3f deg, 2%%)\n\n', ...
    p6_config.recovery_band, rad2deg(p6_config.recovery_band));

%% =========================================================================
% SECTION 3: CORE FAULT SIMULATION SCENARIOS
% =========================================================================

scenario_names = { ...
    'Case 0: Nominal (No Contact Loss)', ...
    'Case 1: Front-Left (FL) Temporary Loss', ...
    'Case 2: Front-Right (FR) Temporary Loss', ...
    'Case 3: Rear-Left (RL) Temporary Loss', ...
    'Case 4: Rear-Right (RR) Temporary Loss', ...
    'Case 5: Front-Leg Persistent Contact Loss', ...
    'Case 6: Rear-Leg Persistent Contact Loss', ...
    'Case 7: FL Loss with Actuator Saturation' ...
};

num_scenarios = length(scenario_names);
scenarios_cell = cell(num_scenarios, 1);

fprintf('3. Executing Core Fault Scenarios...\n');

% --- CASE 0: NOMINAL (NO FAULT) ---
s0 = get_default_fault_struct();
scenarios_cell{1} = simulate_contact_loss_case(p6_config, s0, Kp_final, Ki_final, params);
fprintf('   [x] Case 0 completed: Nominal baseline response.\n');

% --- CASE 1: FRONT-LEFT (FL) TEMPORARY LOSS ---
% Front leg loss causes nose-down pitch disturbance (-Td) and reduced torque authority
s1 = get_default_fault_struct();
s1.is_active      = true;
s1.leg_label      = 'FL';
s1.alpha          = p6_config.alpha_fault;
s1.Td_contact     = -p6_config.Td_fault_mag; % Negative: nose-down pitch torque
s1.duration       = p6_config.t_fault_dur;
scenarios_cell{2} = simulate_contact_loss_case(p6_config, s1, Kp_final, Ki_final, params);
fprintf('   [x] Case 1 completed: Front-Left (FL) temporary loss.\n');

% --- CASE 2: FRONT-RIGHT (FR) TEMPORARY LOSS ---
% In SISO pitch axis, FR produces identical pitch disturbance (-Td) to FL (lateral symmetry)
s2 = s1;
s2.leg_label      = 'FR';
scenarios_cell{3} = simulate_contact_loss_case(p6_config, s2, Kp_final, Ki_final, params);
fprintf('   [x] Case 2 completed: Front-Right (FR) temporary loss (symmetric to FL).\n');

% --- CASE 3: REAR-LEFT (RL) TEMPORARY LOSS ---
% Rear leg loss causes nose-up pitch disturbance (+Td) and reduced torque authority
s3 = get_default_fault_struct();
s3.is_active      = true;
s3.leg_label      = 'RL';
s3.alpha          = p6_config.alpha_fault;
s3.Td_contact     = +p6_config.Td_fault_mag; % Positive: nose-up pitch torque
s3.duration       = p6_config.t_fault_dur;
scenarios_cell{4} = simulate_contact_loss_case(p6_config, s3, Kp_final, Ki_final, params);
fprintf('   [x] Case 3 completed: Rear-Left (RL) temporary loss.\n');

% --- CASE 4: REAR-RIGHT (RR) TEMPORARY LOSS ---
% In SISO pitch axis, RR produces identical pitch disturbance (+Td) to RL (lateral symmetry)
s4 = s3;
s4.leg_label      = 'RR';
scenarios_cell{5} = simulate_contact_loss_case(p6_config, s4, Kp_final, Ki_final, params);
fprintf('   [x] Case 4 completed: Rear-Right (RR) temporary loss (symmetric to RL).\n');

% --- CASE 5: FRONT-LEG PERSISTENT CONTACT LOSS ---
s5 = s1;
s5.is_persistent  = true;
s5.duration       = Inf;
scenarios_cell{6} = simulate_contact_loss_case(p6_config, s5, Kp_final, Ki_final, params);
fprintf('   [x] Case 5 completed: Front-leg persistent contact loss.\n');

% --- CASE 6: REAR-LEG PERSISTENT CONTACT LOSS ---
s6 = s3;
s6.is_persistent  = true;
s6.duration       = Inf;
scenarios_cell{7} = simulate_contact_loss_case(p6_config, s6, Kp_final, Ki_final, params);
fprintf('   [x] Case 6 completed: Rear-leg persistent contact loss.\n');

% --- CASE 7: FL LOSS WITH ACTUATOR SATURATION ---
s7 = s1;
s7.enable_actuator_limits = true;
scenarios_cell{8} = simulate_contact_loss_case(p6_config, s7, Kp_final, Ki_final, params);
fprintf('   [x] Case 7 completed: FL loss with actuator torque saturation.\n\n');

%% =========================================================================
% SECTION 4: CONTACT-LOSS SEVERITY & DURATION SWEEPS
% =========================================================================

fprintf('4. Executing Parametric Severity & Duration Sweeps...\n');

% A. Severity Sweep (Varying Torque Authority alpha from 1.0 down to 0.2)
alpha_values = [1.00, 0.90, 0.80, 0.70, 0.60, 0.50, 0.40, 0.30, 0.20]';
N_alpha = length(alpha_values);
sev_peak_dev    = zeros(N_alpha, 1);
sev_rec_time    = zeros(N_alpha, 1);
sev_rms_err     = zeros(N_alpha, 1);
sev_max_torque  = zeros(N_alpha, 1);
sev_status      = cell(N_alpha, 1);

for k = 1:N_alpha
    s_sev = s1;
    s_sev.alpha = alpha_values(k);
    % Scaled disturbance proportional to loss of authority
    s_sev.Td_contact = -p6_config.Td_fault_mag * (1.0 - alpha_values(k) + 0.5);
    res_sev = simulate_contact_loss_case(p6_config, s_sev, Kp_final, Ki_final, params);
    
    sev_peak_dev(k)   = res_sev.metrics.PeakPitchDeviation;
    sev_rec_time(k)   = res_sev.metrics.RecoveryTime;
    sev_rms_err(k)    = res_sev.metrics.RMS_Error;
    sev_max_torque(k) = res_sev.metrics.MaxControlTorque;
    sev_status{k}     = res_sev.metrics.Classification;
end

severity_table = table(alpha_values, sev_peak_dev, sev_rec_time, sev_rms_err, ...
                       sev_max_torque, sev_status, ...
                       'VariableNames', {'TorqueAuthority_alpha', 'PeakDeviation_rad', ...
                                         'RecoveryTime_s', 'RMS_Error_rad', ...
                                         'MaxTorque_Nm', 'FaultClassification'});

% B. Duration Sweep (Varying Fault Duration T_loss from 0.2 s to 5.0 s)
duration_values = [0.2, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0]';
N_dur = length(duration_values);
dur_peak_dev    = zeros(N_dur, 1);
dur_rec_time    = zeros(N_dur, 1);
dur_rms_err     = zeros(N_dur, 1);
dur_max_torque  = zeros(N_dur, 1);
dur_status      = cell(N_dur, 1);

for k = 1:N_dur
    s_dur = s1;
    s_dur.duration = duration_values(k);
    res_dur = simulate_contact_loss_case(p6_config, s_dur, Kp_final, Ki_final, params);
    
    dur_peak_dev(k)   = res_dur.metrics.PeakPitchDeviation;
    dur_rec_time(k)   = res_dur.metrics.RecoveryTime;
    dur_rms_err(k)    = res_dur.metrics.RMS_Error;
    dur_max_torque(k) = res_dur.metrics.MaxControlTorque;
    dur_status{k}     = res_dur.metrics.Classification;
end

duration_table = table(duration_values, dur_peak_dev, dur_rec_time, dur_rms_err, ...
                       dur_max_torque, dur_status, ...
                       'VariableNames', {'FaultDuration_s', 'PeakDeviation_rad', ...
                                         'RecoveryTime_s', 'RMS_Error_rad', ...
                                         'MaxTorque_Nm', 'FaultClassification'});

fprintf('   [x] Severity & Duration parametric sweeps completed.\n\n');

%% =========================================================================
% SECTION 5: METRICS EXTRACTION & FAULT ANALYSIS SUMMARY TABLE
% =========================================================================

summary_rows = cell(num_scenarios, 10);

for i = 1:num_scenarios
    res = scenarios_cell{i};
    m = res.metrics;
    summary_rows(i, :) = { ...
        scenario_names{i}, ...
        m.PeakPitchDeviation, ...
        m.PeakError, ...
        m.RMS_Error, ...
        m.RecoveryTime, ...
        m.SteadyStateResidual, ...
        m.MaxControlTorque, ...
        m.RMS_ControlTorque, ...
        m.PctSaturated, ...
        m.Classification ...
    };
end

col_summary = { ...
    'Scenario', 'PeakDeviation_rad', 'PeakError_rad', 'RMS_Error_rad', ...
    'RecoveryTime_s', 'ResidualOffset_rad', 'MaxTorque_Nm', 'RMSTorque_Nm', ...
    'PctSaturated', 'FaultClassification' ...
};
fault_summary_table = cell2table(summary_rows, 'VariableNames', col_summary);

fprintf('5. Quantitative Summary of One-Leg Contact-Loss Scenarios:\n');
disp(fault_summary_table(:, 1:6));
disp(fault_summary_table(:, [1, 7:10]));

%% =========================================================================
% SECTION 6: IDENTIFY CRITICAL FAULT SCENARIO
% =========================================================================

% Critical fault criterion: highest peak pitch deviation among temporary faults
[max_dev_val, idx_crit] = max(fault_summary_table.PeakDeviation_rad(2:5));
critical_scenario_name  = scenario_names{idx_crit + 1};

fprintf('6. Critical Fault Scenario Identification:\n');
fprintf('   - Critical Scenario           : %s\n', critical_scenario_name);
fprintf('   - Maximum Pitch Deviation     : %.4f rad (%.2f deg)\n', max_dev_val, rad2deg(max_dev_val));
fprintf('   - Recovery Time               : %.3f s\n', fault_summary_table.RecoveryTime_s(idx_crit + 1));
fprintf('   - Justification               : Produces greatest transient pitch deviation under single-leg fault.\n\n');

%% =========================================================================
% SECTION 7: PERSIST RESULTS AND EXPORT TABLES
% =========================================================================

tables_dir = fullfile(paths.results, 'tables');
if ~exist(tables_dir, 'dir'), mkdir(tables_dir); end

exported_dir = fullfile(paths.results, 'exported');
if ~exist(exported_dir, 'dir'), mkdir(exported_dir); end

% Export CSV tables
writetable(fault_summary_table, fullfile(tables_dir, 'phase6_fault_analysis_summary.csv'));
writetable(severity_table, fullfile(tables_dir, 'phase6_severity_sweep.csv'));
writetable(duration_table, fullfile(tables_dir, 'phase6_duration_sweep.csv'));

% Build complete persistent result structure
phase6_results = struct();
phase6_results.config              = p6_config;
phase6_results.Kp_frozen           = Kp_final;
phase6_results.Ki_frozen           = Ki_final;
phase6_results.summary_table       = fault_summary_table;
phase6_results.severity_table      = severity_table;
phase6_results.duration_table      = duration_table;
phase6_results.nominal             = scenarios_cell{1};
phase6_results.FL                  = scenarios_cell{2};
phase6_results.FR                  = scenarios_cell{3};
phase6_results.RL                  = scenarios_cell{4};
phase6_results.RR                  = scenarios_cell{5};
phase6_results.FL_persistent       = scenarios_cell{6};
phase6_results.RL_persistent       = scenarios_cell{7};
phase6_results.FL_saturated        = scenarios_cell{8};
phase6_results.critical_scenario   = critical_scenario_name;
phase6_results.date_executed       = datestr(now);

save(fullfile(exported_dir, 'phase6_fault_analysis_results.mat'), 'phase6_results');

fprintf('7. Artifacts Saved Successfully:\n');
fprintf('   - Summary Table  : %s\n', fullfile(tables_dir, 'phase6_fault_analysis_summary.csv'));
fprintf('   - Severity Table : %s\n', fullfile(tables_dir, 'phase6_severity_sweep.csv'));
fprintf('   - Duration Table : %s\n', fullfile(tables_dir, 'phase6_duration_sweep.csv'));
fprintf('   - Data MAT       : %s\n\n', fullfile(exported_dir, 'phase6_fault_analysis_results.mat'));

%% =========================================================================
% SECTION 8: SCIENTIFIC FIGURE GENERATION (16 REQUIRED FIGURES)
% =========================================================================

fig_dir = fullfile(paths.results, 'figures');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

t = p6_config.time;
res_nom  = scenarios_cell{1};
res_fl   = scenarios_cell{2};
res_fr   = scenarios_cell{3};
res_rl   = scenarios_cell{4};
res_rr   = scenarios_cell{5};
res_fl_p = scenarios_cell{6};
res_sat  = scenarios_cell{8};

% --- Figure 1: Nominal Body-Pitch Response ---
f1 = figure('Name', 'Phase 6 - Nominal Pitch Response', 'Position', [50, 50, 750, 480]);
plot(t, res_nom.theta, 'b-', 'LineWidth', 2);
grid on;
title('Figure 1: Nominal Closed-Loop Pitch Response (No Fault)');
xlabel('Time [s]'); ylabel('\theta(t) [rad]');
legend('Nominal Pitch \theta(t)', 'Location', 'southeast');
saveas(f1, fullfile(fig_dir, 'phase6_fig01_nominal_pitch_response.png'));

% --- Figure 2: Contact-Loss Body-Pitch Response (FL Fault) ---
f2 = figure('Name', 'Phase 6 - Contact-Loss Response', 'Position', [70, 70, 750, 480]);
plot(t, res_fl.theta, 'r-', 'LineWidth', 2);
hold on;
xline(p6_config.t_fault_start, 'k--', 'Fault Onset');
xline(p6_config.t_fault_start + p6_config.t_fault_dur, 'g--', 'Recontact');
grid on;
title('Figure 2: Body-Pitch Response During Temporary Front-Left (FL) Contact Loss');
xlabel('Time [s]'); ylabel('\theta(t) [rad]');
legend('Pitch Response Under Fault', 'Location', 'southeast');
saveas(f2, fullfile(fig_dir, 'phase6_fig02_contact_loss_pitch_response.png'));

% --- Figure 3: Reference vs Actual Pitch Under Fault ---
f3 = figure('Name', 'Phase 6 - Reference vs Actual Pitch', 'Position', [90, 90, 750, 480]);
plot(t, p6_config.ref_val * ones(size(t)), 'k--', 'LineWidth', 1.2);
hold on;
plot(t, res_fl.theta, 'b-', 'LineWidth', 2);
yline(p6_config.ref_val + p6_config.recovery_band, 'm:');
yline(p6_config.ref_val - p6_config.recovery_band, 'm:');
grid on;
title('Figure 3: Reference Pitch vs Actual Pitch Under FL Contact Loss');
xlabel('Time [s]'); ylabel('\theta(t) [rad]');
legend('Reference \theta_d', 'Actual Pitch \theta(t)', 'Recovery Band (+/- 2%)', 'Location', 'southeast');
saveas(f3, fullfile(fig_dir, 'phase6_fig03_reference_vs_actual_pitch.png'));

% --- Figure 4: Tracking Error During Contact Loss ---
f4 = figure('Name', 'Phase 6 - Tracking Error', 'Position', [110, 110, 750, 450]);
plot(t, res_fl.error, 'r-', 'LineWidth', 2);
hold on;
xline(p6_config.t_fault_start, 'k--', 'Fault Onset');
xline(p6_config.t_fault_start + p6_config.t_fault_dur, 'g--', 'Recontact');
grid on;
title(sprintf('Figure 4: Tracking Error e(t) = \\theta_d - \\theta (Peak Deviation = %.4f rad)', ...
      res_fl.metrics.PeakPitchDeviation));
xlabel('Time [s]'); ylabel('Error e(t) [rad]');
saveas(f4, fullfile(fig_dir, 'phase6_fig04_tracking_error_during_fault.png'));

% --- Figure 5: Contact-Loss Event Indicator c_i(t) ---
f5 = figure('Name', 'Phase 6 - Contact Event Indicator', 'Position', [130, 130, 750, 400]);
stairs(t, res_fl.contact_state, 'LineWidth', 2, 'Color', [0.8 0.2 0.2]);
ylim([-0.2, 1.2]); grid on;
title('Figure 5: Discrete Leg Contact State Indicator c_i(t) (1 = Stance, 0 = Lost)');
xlabel('Time [s]'); ylabel('Contact State c_i(t)');
saveas(f5, fullfile(fig_dir, 'phase6_fig05_contact_event_indicator.png'));

% --- Figure 6: Equivalent Contact-Loss Disturbance Torque ---
f6 = figure('Name', 'Phase 6 - Contact Disturbance Torque', 'Position', [150, 150, 750, 450]);
plot(t, res_fl.Td_contact, 'Color', [0.8 0.3 0.1], 'LineWidth', 2);
grid on;
title(sprintf('Figure 6: Equivalent Ground-Reaction Asymmetry Disturbance Torque T_{d,contact}(t) (%.2f N.m)', ...
      p6_config.Td_fault_mag));
xlabel('Time [s]'); ylabel('Disturbance Torque [N.m]');
saveas(f6, fullfile(fig_dir, 'phase6_fig06_contact_disturbance_torque.png'));

% --- Figure 7: Commanded Controller Output Torque ---
f7 = figure('Name', 'Phase 6 - Commanded Control Torque', 'Position', [170, 170, 750, 450]);
plot(t, res_fl.u_cmd, 'm-', 'LineWidth', 2);
grid on;
title('Figure 7: Controller Commanded Pitch Torque u_{cmd}(t) During Fault Reaction');
xlabel('Time [s]'); ylabel('Commanded Torque [N.m]');
saveas(f7, fullfile(fig_dir, 'phase6_fig07_commanded_control_torque.png'));

% --- Figure 8: Actual Applied Control Torque (With Authority Reduction) ---
f8 = figure('Name', 'Phase 6 - Applied Control Torque', 'Position', [190, 190, 750, 480]);
plot(t, res_fl.u_cmd, 'k:', 'LineWidth', 1.2);
hold on;
plot(t, res_fl.u_actual, 'b-', 'LineWidth', 2);
grid on;
title(sprintf('Figure 8: Commanded vs Actual Applied Torque T_c(t) (\\alpha = %.2f during fault)', ...
      p6_config.alpha_fault));
xlabel('Time [s]'); ylabel('Torque [N.m]');
legend('Commanded u_{cmd}(t)', 'Available Applied T_c(t)', 'Location', 'northeast');
saveas(f8, fullfile(fig_dir, 'phase6_fig08_applied_control_torque.png'));

% --- Figure 9: Nominal vs Contact-Loss Response Comparison ---
f9 = figure('Name', 'Phase 6 - Nominal vs Fault Comparison', 'Position', [210, 210, 750, 480]);
plot(t, res_nom.theta, 'k--', 'LineWidth', 1.8);
hold on;
plot(t, res_fl.theta, 'b-', 'LineWidth', 2);
plot(t, res_rl.theta, 'r-', 'LineWidth', 2);
grid on;
title('Figure 9: Nominal Response vs Front-Leg (FL) and Rear-Leg (RL) Contact Loss');
xlabel('Time [s]'); ylabel('Pitch \theta(t) [rad]');
legend('Nominal (No Fault)', 'Front-Leg Loss (Nose-Down Bias)', 'Rear-Leg Loss (Nose-Up Bias)', 'Location', 'southeast');
saveas(f9, fullfile(fig_dir, 'phase6_fig09_nominal_vs_fault_comparison.png'));

% --- Figure 10: Recovery Region & Tolerance Band ---
f10 = figure('Name', 'Phase 6 - Recovery Tolerance Band', 'Position', [230, 230, 750, 480]);
plot(t, res_fl.theta, 'b-', 'LineWidth', 2);
hold on;
yline(p6_config.ref_val + p6_config.recovery_band, 'r--', 'Upper 2% Band');
yline(p6_config.ref_val - p6_config.recovery_band, 'r--', 'Lower 2% Band');
xline(p6_config.t_fault_start + p6_config.t_fault_dur + res_fl.metrics.RecoveryTime, 'g-', 'Recovery Instant');
grid on;
title(sprintf('Figure 10: Closed-Loop Fault Recovery Detail (Recovery Time = %.3f s)', ...
      res_fl.metrics.RecoveryTime));
xlabel('Time [s]'); ylabel('\theta(t) [rad]');
xlim([2.5, 7.5]);
legend('Actual Pitch', '2% Tolerance Band', 'Recovery Instant', 'Location', 'southeast');
saveas(f10, fullfile(fig_dir, 'phase6_fig10_recovery_tolerance_band.png'));

% --- Figure 11: Temporary vs Persistent Contact Loss ---
f11 = figure('Name', 'Phase 6 - Temporary vs Persistent Loss', 'Position', [250, 250, 750, 480]);
plot(t, res_nom.theta, 'k:', 'LineWidth', 1.5);
hold on;
plot(t, res_fl.theta, 'b-', 'LineWidth', 2);
plot(t, res_fl_p.theta, 'r--', 'LineWidth', 2);
grid on;
title('Figure 11: Transient Response: Temporary Contact Loss vs Persistent Contact Loss');
xlabel('Time [s]'); ylabel('\theta(t) [rad]');
legend('Nominal Baseline', 'Temporary Loss (1.5 s duration)', 'Persistent Contact Loss', 'Location', 'southeast');
saveas(f11, fullfile(fig_dir, 'phase6_fig11_temporary_vs_persistent_loss.png'));

% --- Figure 12: Four-Leg Comparison (FL, FR, RL, RR) ---
f12 = figure('Name', 'Phase 6 - Four-Leg Fault Comparison', 'Position', [270, 270, 800, 500]);
plot(t, res_fl.theta, 'b-', 'LineWidth', 2);
hold on;
plot(t, res_fr.theta, 'c--', 'LineWidth', 1.8);
plot(t, res_rl.theta, 'r-', 'LineWidth', 2);
plot(t, res_rr.theta, 'm--', 'LineWidth', 1.8);
grid on;
title('Figure 12: Four-Leg Fault Comparison Under SISO Pitch Model');
xlabel('Time [s]'); ylabel('Body Pitch \theta(t) [rad]');
legend('FL Loss (Front-Left)', 'FR Loss (Front-Right: Symmetric)', ...
       'RL Loss (Rear-Left)', 'RR Loss (Rear-Right: Symmetric)', 'Location', 'southeast');
saveas(f12, fullfile(fig_dir, 'phase6_fig12_four_leg_fault_comparison.png'));

% --- Figure 13: Severity-Sweep Comparison ---
f13 = figure('Name', 'Phase 6 - Severity-Sweep Comparison', 'Position', [290, 290, 800, 500]);
plot(t, res_nom.theta, 'k--', 'LineWidth', 1.5);
hold on;
alphas_to_plot = [1.0, 0.75, 0.50, 0.25];
for a_val = alphas_to_plot
    s_tmp = s1; s_tmp.alpha = a_val;
    r_tmp = simulate_contact_loss_case(p6_config, s_tmp, Kp_final, Ki_final, params);
    plot(t, r_tmp.theta, 'LineWidth', 1.8);
end
grid on;
title('Figure 13: Transient Pitch Response vs Torque Authority \alpha (100% to 25%)');
xlabel('Time [s]'); ylabel('\theta(t) [rad]');
legend('Nominal', '\alpha = 1.00', '\alpha = 0.75', '\alpha = 0.50', '\alpha = 0.25', 'Location', 'southeast');
saveas(f13, fullfile(fig_dir, 'phase6_fig13_severity_sweep_comparison.png'));

% --- Figure 14: Duration-Sweep Comparison ---
f14 = figure('Name', 'Phase 6 - Duration-Sweep Comparison', 'Position', [310, 310, 800, 500]);
durs_to_plot = [0.5, 1.0, 2.0, 4.0];
for d_val = durs_to_plot
    s_tmp = s1; s_tmp.duration = d_val;
    r_tmp = simulate_contact_loss_case(p6_config, s_tmp, Kp_final, Ki_final, params);
    plot(t, r_tmp.theta, 'LineWidth', 1.8);
    hold on;
end
grid on;
title('Figure 14: Pitch Deviation and Recovery vs Contact-Loss Duration T_{loss}');
xlabel('Time [s]'); ylabel('\theta(t) [rad]');
legend('T_{loss} = 0.5 s', 'T_{loss} = 1.0 s', 'T_{loss} = 2.0 s', 'T_{loss} = 4.0 s', 'Location', 'southeast');
saveas(f14, fullfile(fig_dir, 'phase6_fig14_duration_sweep_comparison.png'));

% --- Figure 15: Severity vs Recovery Metrics Map ---
f15 = figure('Name', 'Phase 6 - Severity Metrics Map', 'Position', [330, 200, 800, 500]);
subplot(2, 1, 1);
plot(alpha_values, sev_peak_dev, 'ro-', 'LineWidth', 2, 'MarkerFaceColor', 'r');
grid on; title('Figure 15: Parametric Sensitivity to Torque Authority \alpha');
ylabel('Peak Deviation [rad]');
subplot(2, 1, 2);
plot(alpha_values, sev_rec_time, 'bs-', 'LineWidth', 2, 'MarkerFaceColor', 'b');
grid on; xlabel('Available Torque Authority \alpha'); ylabel('Recovery Time [s]');
saveas(f15, fullfile(fig_dir, 'phase6_fig15_severity_vs_metrics_map.png'));

% --- Figure 16: Actuator Limits Interaction During Fault ---
f16 = figure('Name', 'Phase 6 - Actuator Limits Interaction', 'Position', [350, 150, 800, 550]);
subplot(2, 1, 1);
plot(t, res_fl.theta, 'b-', 'LineWidth', 2); hold on;
plot(t, res_sat.theta, 'r--', 'LineWidth', 2);
grid on; title('Figure 16: Fault Recovery Under Actuator Saturation (u_{max} = 0.40 N.m)');
ylabel('\theta(t) [rad]');
legend('Unsaturated Actuator', 'Torque-Saturated Actuator', 'Location', 'southeast');
subplot(2, 1, 2);
plot(t, res_sat.u_actual, 'm-', 'LineWidth', 1.8);
yline(p6_config.u_max_sat, 'k:'); yline(-p6_config.u_max_sat, 'k:');
grid on; xlabel('Time [s]'); ylabel('Torque T_c(t) [N.m]');
legend('Applied Torque (Saturated)', 'Location', 'northeast');
saveas(f16, fullfile(fig_dir, 'phase6_fig16_actuator_limits_interaction.png'));

fprintf('8. All 16 Engineering Figures Saved to 06_RESULTS/figures/\n');

print_banner('Phase 6 Fault Analysis Complete', ...
    'Results and figures persisted to 06_RESULTS/ and 07_DATA/');

%% =========================================================================
% LOCAL HELPER FUNCTIONS (SIMULATION ENGINE & FAULT CLASSIFICATION)
% =========================================================================

function s = get_default_fault_struct()
    s = struct();
    s.is_active              = false;
    s.is_persistent          = false;
    s.leg_label              = 'NONE';
    s.alpha                  = 1.0;
    s.Td_contact             = 0.0;
    s.duration               = 0.0;
    s.enable_actuator_limits = false;
end

function out = simulate_contact_loss_case(p6_config, fault_info, Kp, Ki, plant_params)
    dt   = p6_config.dt_sim;
    time = p6_config.time;
    N    = length(time);
    
    J = plant_params.J;
    B = plant_params.B;
    K = plant_params.K;
    
    theta_arr       = zeros(N, 1);
    omega_arr       = zeros(N, 1);
    error_arr       = zeros(N, 1);
    u_cmd_arr       = zeros(N, 1);
    u_actual_arr    = zeros(N, 1);
    Td_arr          = zeros(N, 1);
    contact_arr     = ones(N, 1);
    is_sat_arr      = false(N, 1);
    
    theta = 0.0;
    omega = 0.0;
    I_state = 0.0;
    u_prev = 0.0;
    
    ref_val = p6_config.ref_val;
    t_f_start = p6_config.t_fault_start;
    t_f_end   = t_f_start + fault_info.duration;
    
    for k = 1:N
        t_now = time(k);
        
        % Check fault status
        fault_on = false;
        if fault_info.is_active
            if fault_info.is_persistent
                fault_on = (t_now >= t_f_start);
            else
                fault_on = (t_now >= t_f_start && t_now < t_f_end);
            end
        end
        
        if fault_on
            contact_state = 0.0;
            alpha_now     = fault_info.alpha;
            Td_now        = fault_info.Td_contact;
        else
            contact_state = 1.0;
            alpha_now     = 1.0;
            Td_now        = 0.0;
        end
        
        % Error computation
        err = ref_val - theta;
        
        % PI Controller command
        u_cmd = Kp * err + I_state;
        
        % Actuator rate limiting if enabled
        u_rate = u_cmd;
        if fault_info.enable_actuator_limits
            delta_u = u_cmd - u_prev;
            max_delta = p6_config.R_max_rate * dt;
            if delta_u > max_delta
                u_rate = u_prev + max_delta;
            elseif delta_u < -max_delta
                u_rate = u_prev - max_delta;
            end
        end
        
        % Actuator saturation if enabled
        u_sat = u_rate;
        sat_active = false;
        if fault_info.enable_actuator_limits
            if u_rate > p6_config.u_max_sat
                u_sat = p6_config.u_max_sat;
                sat_active = true;
            elseif u_rate < -p6_config.u_max_sat
                u_sat = -p6_config.u_max_sat;
                sat_active = true;
            end
        end
        
        % Available torque authority reduction
        u_applied = alpha_now * u_sat;
        
        % Integrator update
        I_state = I_state + Ki * dt * err;
        
        % Log arrays
        theta_arr(k)    = theta;
        omega_arr(k)    = omega;
        error_arr(k)    = err;
        u_cmd_arr(k)    = u_cmd;
        u_actual_arr(k) = u_applied;
        Td_arr(k)       = Td_now;
        contact_arr(k)  = contact_state;
        is_sat_arr(k)   = sat_active;
        u_prev          = u_sat;
        
        % RK4 Forward Integration of Plant: J*theta_ddot + B*theta_dot + K*theta = u_applied + Td_now
        Tc_total = u_applied + Td_now;
        
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
    
    % Post-processing fault metrics
    idx_fault = find(time >= t_f_start, 1);
    post_fault_theta = theta_arr(idx_fault:end);
    peak_deviation   = max(abs(post_fault_theta - ref_val));
    peak_error       = max(abs(error_arr));
    rms_error        = sqrt(mean(error_arr.^2));
    max_torque       = max(abs(u_actual_arr));
    rms_torque       = sqrt(mean(u_actual_arr.^2));
    
    ss_indices = round(0.85 * N):N;
    ss_residual = abs(ref_val - mean(theta_arr(ss_indices)));
    
    % Recovery time definition:
    % Time from recontact (or fault onset if persistent) until theta remains within 2% band of ref_val
    rec_time = NaN;
    if fault_info.is_active
        if fault_info.is_persistent
            % Under persistent fault, recovery means settling to setpoint despite ongoing fault
            idx_eval = idx_fault;
            t_eval_start = t_f_start;
        else
            idx_eval = find(time >= t_f_end, 1);
            t_eval_start = t_f_end;
        end
        
        if ~isempty(idx_eval) && idx_eval < N
            eval_theta = theta_arr(idx_eval:end);
            outside_band = abs(eval_theta - ref_val) > p6_config.recovery_band;
            last_outside = find(outside_band, 1, 'last');
            
if isempty(last_outside)
    rec_time = 0.0;
elseif last_outside == numel(eval_theta)
    % Recovery was not achieved before simulation ended
    rec_time = NaN;
else
    % Convert relative index to absolute index
    recovery_idx = idx_eval + last_outside;
    rec_time = time(recovery_idx) - t_eval_start;
end

        end
    else
        rec_time = 0.0;
        peak_deviation = 0.0;
    end
    
    pct_sat = 100 * sum(is_sat_arr) / N;
    
    % Fault-Tolerance Classification (Section 22)
    if any(isnan(theta_arr)) || any(isinf(theta_arr)) || max(abs(theta_arr)) > 5.0
        classification = 'FAILURE (Unstable)';
    elseif ss_residual > 0.25
        classification = 'FAILURE (Persistent Offset > 0.25 rad)';
    elseif peak_deviation > 0.20 || pct_sat > 10.0 || rec_time > 4.0
        classification = 'CRITICAL (Severe Degradation)';
    elseif peak_deviation > 0.05 || rec_time > 1.5
        classification = 'DEGRADED (Stable, Exceeds Nominal)';
    else
        classification = 'NORMAL (Acceptable Nominal)';
    end
    
    out = struct();
    out.time          = time;
    out.theta         = theta_arr;
    out.omega         = omega_arr;
    out.error         = error_arr;
    out.u_cmd         = u_cmd_arr;
    out.u_actual      = u_actual_arr;
    out.Td_contact    = Td_arr;
    out.contact_state = contact_arr;
    
    out.metrics = struct( ...
        'PeakPitchDeviation', peak_deviation, ...
        'PeakError', peak_error, ...
        'RMS_Error', rms_error, ...
        'RecoveryTime', rec_time, ...
        'SteadyStateResidual', ss_residual, ...
        'MaxControlTorque', max_torque, ...
        'RMS_ControlTorque', rms_torque, ...
        'PctSaturated', pct_sat, ...
        'Classification', classification ...
    );
end
