%% =========================================================================
% QUADRUPED ROBOT BODY-PITCH CONTROL
% PHASE 4 - PI CONTROLLER DESIGN, SYSTEMATIC TUNING, PARETO SELECTION & FREEZING
%
% Architecture : Single-Input Single-Output (SISO) Body-Pitch Attitude Control
% Input (r)    : Desired body pitch angle theta_d(t) [rad]
% Output (y)   : Actual body pitch angle theta(t) [rad]
% Control (u)  : Net pitch control torque Tc(t) [N.m]
% Feedback     : Unit negative pitch angle feedback e(t) = theta_d(t) - theta(t)
%
% Scope (Phase 4 Only):
%   4.1 Model & PI Formulation with Analytical Characteristic Verification
%   4.2 Systematic 2D Grid Sweep (Kp, Ki) with Full Metric Extraction
%   4.3 Stability Filtering & Routh-Hurwitz Cross-Validation
%   4.4 Constraint Filtering (Overshoot, Settling, Rise Time)
%   4.5 2D Performance & Feasibility Maps
%   4.6 Multi-Objective Pareto Trade-Off Analysis
%   4.7 Deterministic Engineering Selection & Controller Freezing
%   4.8 Dedicated Final Validation & Frequency-Domain Margins
%   4.9 Comprehensive Comparison Against Baseline P-Controller
%   4.10 Numerical Export of Tables and Figures
% =========================================================================

% Ensure clean environment formatting without destroying workspace context
ensure_clean_env(false);

print_banner('Phase 4 - PI Controller Design & Optimization', ...
    'SISO Quadruped Body-Pitch Attitude Control');

%% =========================================================================
% SECTION 1: PLANT MODEL & PI FORMULATION
% =========================================================================

% Retrieve authoritative physical parameters and plant transfer function
params = get_robot_parameters();
J = params.J;
B = params.B;
K = params.K;

[G, sys_ss, s] = get_pitch_plant(params);

fprintf('1. Authoritative Physical Model Parameters:\n');
fprintf('   Body Pitch Inertia (J)       = %.4f kg.m^2\n', J);
fprintf('   Joint Damping Coeff (B)      = %.4f N.m.s/rad\n', B);
fprintf('   Restoring Stiffness (K)      = %.4f N.m/rad\n\n', K);

fprintf('2. Open-Loop Plant Transfer Function G(s):\n');
G

% Analytical Characteristic Equation & Routh-Hurwitz Stability Boundary
% Open-loop: L(s) = C(s)*G(s) = (Kp*s + Ki) / (s*(J*s^2 + B*s + K))
% Closed-loop characteristic polynomial: J*s^3 + B*s^2 + (K + Kp)*s + Ki = 0
% For K = 0: Delta(s) = J*s^3 + B*s^2 + Kp*s + Ki = 0
% Routh-Hurwitz stability requires:
%   a3 = J > 0, a2 = B > 0, a1 = Kp > 0, a0 = Ki > 0
%   b1 = (a2*a1 - a3*a0)/a2 = (B*Kp - J*Ki)/B > 0  ==>  Ki < (B/J)*Kp
max_stable_ratio = B / J; % = 0.10 / 0.05 = 2.00
fprintf('3. Analytical Stability Condition (Routh-Hurwitz):\n');
fprintf('   Closed-loop polynomial : %.4f*s^3 + %.4f*s^2 + Kp*s + Ki = 0\n', J, B);
fprintf('   Theoretical stability criterion : Kp > 0, Ki > 0, and Ki < %.2f * Kp\n\n', max_stable_ratio);

%% =========================================================================
% SECTION 2: TUNING CONFIGURATION & 2D PARAMETER SWEEP
% =========================================================================

% Authoritative Phase 4 Tuning Ranges (Preserved from validated project definition)
Kp_values = 0.01:0.01:0.50;
Ki_values = 0.01:0.01:0.50;

n_Kp = length(Kp_values);
n_Ki = length(Ki_values);
total_candidates = n_Kp * n_Ki;

% Performance constraints (Authoritative project limits)
rise_target      = 2.0;   % seconds (reported/desirable)
settling_target  = 5.0;   % seconds (2% settling band)
overshoot_target = 20.0;  % percent

stability_tolerance = 1e-8;
sim_time = linspace(0, 10, 1001); % Uniform 10-second evaluation grid

fprintf('4. Tuning Search Space:\n');
fprintf('   Kp search range : [%.2f, %.2f], step = %.2f (N = %d)\n', min(Kp_values), max(Kp_values), Kp_values(2)-Kp_values(1), n_Kp);
fprintf('   Ki search range : [%.2f, %.2f], step = %.2f (N = %d)\n', min(Ki_values), max(Ki_values), Ki_values(2)-Ki_values(1), n_Ki);
fprintf('   Total candidate configurations : %d\n\n', total_candidates);

% Preallocate metric 2D grids for performance maps
map_stable    = false(n_Ki, n_Kp);
map_feasible  = false(n_Ki, n_Kp);
map_rise      = NaN(n_Ki, n_Kp);
map_settling  = NaN(n_Ki, n_Kp);
map_overshoot = NaN(n_Ki, n_Kp);
map_peak      = NaN(n_Ki, n_Kp);
map_ess       = NaN(n_Ki, n_Kp);
map_e_rms     = NaN(n_Ki, n_Kp);
map_u_max     = NaN(n_Ki, n_Kp);
map_u_rms     = NaN(n_Ki, n_Kp);

% Preallocate linear array for table and Pareto search
candidates_data = cell(total_candidates, 14);
row_idx = 0;

fprintf('5. Executing systematic candidate evaluation sweep...\n');

for i_kp = 1:n_Kp
    Kp = Kp_values(i_kp);
    for i_ki = 1:n_Ki
        Ki = Ki_values(i_ki);
        row_idx = row_idx + 1;
        
        % Form candidate controller C(s) = Kp + Ki/s
        C_cand = Kp + Ki/s;
        
        % Closed-loop tracking T(s) = C*G / (1 + C*G)
        T_cand = feedback(C_cand * G, 1);
        
        % Closed-loop poles
        p_cand = pole(T_cand);
        
        % Stability criterion: all poles strictly in open left-half plane
        is_stable = all(real(p_cand) < -stability_tolerance);
        
        if ~is_stable
            candidates_data(row_idx, :) = {Kp, Ki, false, p_cand, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, false, false};
            continue;
        end
        
        map_stable(i_ki, i_kp) = true;
        
        % Step response info
        info_cand = stepinfo(T_cand);
        t_r = info_cand.RiseTime;
        t_s = info_cand.SettlingTime;
        Mp  = info_cand.Overshoot;
        y_pk = info_cand.Peak;
        
        % Numerical simulation for tracking error and control effort
        y_sim = step(T_cand, sim_time);
        e_sim = 1.0 - y_sim; % Reference is unit step
        
        % Steady-state error
        e_ss = abs(1.0 - dcgain(T_cand));
        e_rms = sqrt(mean(e_sim.^2));
        
        % Control effort transfer function: Tu(s) = C / (1 + C*G)
        Tu_cand = minreal(feedback(C_cand, G));
        u_sim = step(Tu_cand, sim_time);
        u_max = max(abs(u_sim));
        u_rms = sqrt(mean(u_sim.^2));
        
        % Record in map grids
        map_rise(i_ki, i_kp)      = t_r;
        map_settling(i_ki, i_kp)  = t_s;
        map_overshoot(i_ki, i_kp) = Mp;
        map_peak(i_ki, i_kp)      = y_pk;
        map_ess(i_ki, i_kp)       = e_ss;
        map_e_rms(i_ki, i_kp)     = e_rms;
        map_u_max(i_ki, i_kp)     = u_max;
        map_u_rms(i_ki, i_kp)     = u_rms;
        
        % Constraint satisfaction check
        is_feasible = (Mp <= overshoot_target) && (t_s <= settling_target) && (t_r <= rise_target);
        map_feasible(i_ki, i_kp) = is_feasible;
        
        candidates_data(row_idx, :) = {Kp, Ki, true, p_cand, t_r, t_s, Mp, y_pk, e_ss, e_rms, u_max, u_rms, is_feasible, false};
    end
end

% Assemble sweep results table
col_names = {'Kp', 'Ki', 'IsStable', 'Poles', 'RiseTime', 'SettlingTime', ...
             'Overshoot', 'Peak', 'SteadyStateError', 'RMS_Error', ...
             'MaxControlEffort', 'RMS_ControlEffort', 'IsFeasible', 'IsPareto'};
sweep_table = cell2table(candidates_data, 'VariableNames', col_names);

n_stable = sum(sweep_table.IsStable);
n_feasible = sum(sweep_table.IsFeasible);

fprintf('   Sweep complete.\n');
fprintf('   - Total candidates evaluated : %d\n', total_candidates);
fprintf('   - Stable candidates          : %d (%.1f%%)\n', n_stable, 100*n_stable/total_candidates);
fprintf('   - Fully feasible candidates  : %d (%.1f%% meeting all constraints)\n\n', ...
    n_feasible, 100*n_feasible/total_candidates);

%% =========================================================================
% SECTION 3: MULTI-OBJECTIVE PARETO TRADE-OFF ANALYSIS
% =========================================================================

% Pareto analysis is strictly conducted on FEASIBLE candidates
% Objectives (Minimization):
%   J1 : Rise Time (t_r)
%   J2 : Settling Time (t_s)
%   J3 : Percent Overshoot (Mp)

feasible_indices = find(sweep_table.IsFeasible);
n_feas = length(feasible_indices);

if n_feas == 0
    % Fallback if strict feasibility is empty: use stable candidates with minimal overshoot violation
    warning('No candidate meets all three simultaneous constraints. Relaxing to stable set for Pareto search.');
    feasible_indices = find(sweep_table.IsStable);
    n_feas = length(feasible_indices);
end

is_pareto_sub = true(n_feas, 1);

for i = 1:n_feas
    idx_i = feasible_indices(i);
    r_i = sweep_table.RiseTime(idx_i);
    s_i = sweep_table.SettlingTime(idx_i);
    o_i = sweep_table.Overshoot(idx_i);
    
    for j = 1:n_feas
        if i == j, continue; end
        idx_j = feasible_indices(j);
        r_j = sweep_table.RiseTime(idx_j);
        s_j = sweep_table.SettlingTime(idx_j);
        o_j = sweep_table.Overshoot(idx_j);
        
        % j dominates i if j is no worse in all objectives and strictly better in at least one
        if (r_j <= r_i && s_j <= s_i && o_j <= o_i) && ...
           (r_j <  r_i || s_j <  s_i || o_j <  o_i)
            is_pareto_sub(i) = false;
            break;
        end
    end
end

% Mark Pareto status in global sweep table
pareto_global_indices = feasible_indices(is_pareto_sub);
sweep_table.IsPareto(pareto_global_indices) = true;
pareto_table = sweep_table(pareto_global_indices, :);

n_pareto = height(pareto_table);
fprintf('6. Pareto Analysis Results:\n');
fprintf('   - Number of Pareto-optimal candidates: %d\n\n', n_pareto);

%% =========================================================================
% SECTION 4: DETERMINISTIC ENGINEERING SELECTION & CONTROLLER FREEZING
% =========================================================================

% Selection Philosophy (Preserved & Standardized):
%   1. Stability : Mandatory
%   2. Overshoot <= 20% : Primary criterion
%   3. Minimum Settling Time : Secondary criterion among acceptable overshoot
%   4. Fallback : Closest candidate to overshoot target

if any(pareto_table.Overshoot <= overshoot_target)
    cand_pool = pareto_table(pareto_table.Overshoot <= overshoot_target, :);
    [~, best_idx] = min(cand_pool.SettlingTime);
    selected_row = cand_pool(best_idx, :);
    selection_basis = 'Pareto Engineering Selection: Feasible Non-Dominated Candidate with Overshoot <= 20% and Minimum Settling Time';
elseif sum(sweep_table.IsStable & (sweep_table.Overshoot <= overshoot_target)) > 0
    cand_pool = sweep_table(sweep_table.IsStable & (sweep_table.Overshoot <= overshoot_target), :);
    [~, best_idx] = min(cand_pool.SettlingTime);
    selected_row = cand_pool(best_idx, :);
    selection_basis = 'Primary Selection: Meets Overshoot <= 20% with Minimum Settling Time';
else
    cand_pool = sweep_table(sweep_table.IsStable, :);
    ov_violation = max(0, cand_pool.Overshoot - overshoot_target);
    [~, best_idx] = min(ov_violation);
    selected_row = cand_pool(best_idx, :);
    selection_basis = 'Fallback Selection: Minimal Normalized Overshoot Violation';
end

Kp_final = selected_row.Kp;
Ki_final = selected_row.Ki;
C_final  = Kp_final + Ki_final/s;
T_final  = feedback(C_final * G, 1);
poles_final = pole(T_final);
info_final  = stepinfo(T_final);

% Determine Pareto extreme representatives for comparative display
[~, idx_fast_pareto] = min(pareto_table.SettlingTime);
fastest_pareto = pareto_table(idx_fast_pareto, :);

[~, idx_lowos_pareto] = min(pareto_table.Overshoot);
lowest_os_pareto = pareto_table(idx_lowos_pareto, :);

fprintf('7. Final Controller Selection:\n');
fprintf('   Basis        : %s\n', selection_basis);
fprintf('   Kp_final     = %.4f\n', Kp_final);
fprintf('   Ki_final     = %.4f\n', Ki_final);
fprintf('   Rise Time    = %.4f s\n', info_final.RiseTime);
fprintf('   Settling Time= %.4f s\n', info_final.SettlingTime);
fprintf('   Overshoot    = %.4f %%\n', info_final.Overshoot);
fprintf('   Peak         = %.4f rad\n', info_final.Peak);
fprintf('   Steady-State = %.8f rad\n', abs(1.0 - dcgain(T_final)));
fprintf('   Closed-loop poles:\n');
for p_idx = 1:length(poles_final)
    fprintf('     p%d = %.4f %+ .4fi\n', p_idx, real(poles_final(p_idx)), imag(poles_final(p_idx)));
end
fprintf('\n');

%% =========================================================================
% SECTION 5: INDEPENDENT CONTROLLER VALIDATION & FREQUENCY MARGINS
% =========================================================================

% Independent simulation outside the sweep
t_val = linspace(0, 8, 801);
[y_val, t_val_out] = step(T_final, t_val);
e_val = 1.0 - y_val;

Tu_final = minreal(feedback(C_final, G));
u_val = step(Tu_final, t_val);

val_u_max = max(abs(u_val));
val_u_rms = sqrt(mean(u_val.^2));
val_e_rms = sqrt(mean(e_val.^2));

% Verification check between sweep metrics and independent validation metrics
val_info = stepinfo(y_val, t_val_out);
diff_ts = abs(selected_row.SettlingTime - val_info.SettlingTime);
diff_mp = abs(selected_row.Overshoot - val_info.Overshoot);
fprintf('8. Independent Validation Cross-Check:\n');
fprintf('   - Stored Sweep Settling Time     : %.4f s\n', selected_row.SettlingTime);
fprintf('   - Re-simulated Settling Time     : %.4f s (Diff: %.4f s)\n', val_info.SettlingTime, diff_ts);
fprintf('   - Stored Sweep Overshoot         : %.4f %%\n', selected_row.Overshoot);
fprintf('   - Re-simulated Overshoot         : %.4f %% (Diff: %.4f %%)\n', val_info.Overshoot, diff_mp);
if diff_ts > 0.1 || diff_mp > 0.5
    warning('Discrepancy detected between sweep metrics and independent simulation!');
else
    fprintf('   - Validation Status              : PASS (Metrics agree within tolerance)\n\n');
end

% Frequency-domain stability margins via open-loop loop gain L(s) = C_final * G
L_final = minreal(C_final * G);
[Gm, Pm, Wcg, Wcp] = margin(L_final);
Gm_dB = 20*log10(Gm);

fprintf('9. Frequency-Domain Stability Margins (Loop Transfer L = C_final * G):\n');
if isinf(Gm)
    fprintf('   Gain Margin (Gm)        : Inf dB (no -180 deg phase crossover)\n');
else
    fprintf('   Gain Margin (Gm)        : %.2f dB at %.2f rad/s\n', Gm_dB, Wcg);
end
fprintf('   Phase Margin (Pm)       : %.2f deg at %.2f rad/s (Gain Crossover)\n\n', Pm, Wcp);

%% =========================================================================
% SECTION 6: COMPARISON WITH PHASE 3 BASELINE P-CONTROLLER
% =========================================================================

% Retrieve Phase 3 Baseline P-controller (Kp = 0.30)
Kp_baseline = 0.30;
C_P = Kp_baseline;
T_P = feedback(C_P * G, 1);
info_P = stepinfo(T_P);

y_P = step(T_P, t_val);
e_P = 1.0 - y_P;
Tu_P = minreal(feedback(C_P, G));
u_P = step(Tu_P, t_val);

P_ess   = abs(1.0 - dcgain(T_P));
P_e_rms = sqrt(mean(e_P.^2));
P_u_max = max(abs(u_P));
P_u_rms = sqrt(mean(u_P.^2));

comparison_matrix = { ...
    'Rise Time [s]',              info_P.RiseTime,           info_final.RiseTime; ...
    'Settling Time [s]',          info_P.SettlingTime,       info_final.SettlingTime; ...
    'Overshoot [%]',              info_P.Overshoot,          info_final.Overshoot; ...
    'Steady-State Error [rad]',   P_ess,                     abs(1.0 - dcgain(T_final)); ...
    'RMS Tracking Error [rad]',   P_e_rms,                   val_e_rms; ...
    'Peak Control Torque [N.m]',  P_u_max,                   val_u_max; ...
    'RMS Control Torque [N.m]',   P_u_rms,                   val_u_rms ...
};
p_vs_pi_table = cell2table(comparison_matrix, 'VariableNames', {'Metric', 'P_Controller_Kp_0_30', 'PI_Controller_Selected'});

fprintf('10. Quantitative Comparison: Baseline P vs Selected PI Controller:\n');
disp(p_vs_pi_table);

%% =========================================================================
% SECTION 7: CONTROLLER FREEZING & METADATA PERSISTENCE
% =========================================================================

paths = project_paths();
save_file = fullfile(paths.data, 'processed', 'frozen_pi_controller.mat');

% Structure of frozen controller metadata
frozen_state.Kp_final           = Kp_final;
frozen_state.Ki_final           = Ki_final;
frozen_state.C_final            = C_final;
frozen_state.T_final            = T_final;
frozen_state.poles_final        = poles_final;
frozen_state.info_final         = info_final;
frozen_state.selection_basis    = selection_basis;
frozen_state.stability_margin   = struct('GainMargin_dB', Gm_dB, 'PhaseMargin_deg', Pm, 'GainCrossover_rad_s', Wcp);
frozen_state.control_effort     = struct('u_max', val_u_max, 'u_rms', val_u_rms);
frozen_state.plant_parameters   = params;
frozen_state.date_frozen        = datestr(now);

% Safe save with verification
if exist(save_file, 'file')
    old_data = load(save_file);
    if isfield(old_data, 'Kp_final') && old_data.Kp_final == Kp_final && ...
       isfield(old_data, 'Ki_final') && old_data.Ki_final == Ki_final
        fprintf('11. Frozen controller check: Matches previously saved candidate (Kp=%.4f, Ki=%.4f).\n', Kp_final, Ki_final);
        fprintf('    Existing frozen controller file preserved without overwrite.\n\n');
    else
        fprintf('11. Updating frozen controller file with new validated candidate...\n');
        save(save_file, 'Kp_final', 'Ki_final', 'C_final', 'T_final', 'info_final', 'frozen_state');
        fprintf('    Successfully saved updated frozen controller to: %s\n\n', save_file);
    end
else
    fprintf('11. Saving new frozen controller artifact...\n');
    save(save_file, 'Kp_final', 'Ki_final', 'C_final', 'T_final', 'info_final', 'frozen_state');
    fprintf('    Successfully saved frozen controller to: %s\n\n', save_file);
end

%% =========================================================================
% SECTION 8: NUMERICAL EXPORTS & TABLES
% =========================================================================

tables_dir = fullfile(paths.results, 'tables');
if ~exist(tables_dir, 'dir'), mkdir(tables_dir); end

% Export Complete Sweep Table (without Pole arrays for CSV compatibility)
sweep_export = sweep_table(:, {'Kp', 'Ki', 'IsStable', 'RiseTime', 'SettlingTime', ...
                               'Overshoot', 'Peak', 'SteadyStateError', 'RMS_Error', ...
                               'MaxControlEffort', 'RMS_ControlEffort', 'IsFeasible', 'IsPareto'});
writetable(sweep_export, fullfile(tables_dir, 'complete_sweep_results.csv'));

% Export Final Candidate Summary Table
final_metrics_table = table(Kp_final, Ki_final, info_final.RiseTime, info_final.SettlingTime, ...
                            info_final.Overshoot, info_final.Peak, abs(1.0 - dcgain(T_final)), ...
                            val_e_rms, val_u_max, val_u_rms, Pm, ...
                            'VariableNames', {'Kp', 'Ki', 'RiseTime_s', 'SettlingTime_s', ...
                                              'Overshoot_pct', 'Peak_rad', 'SteadyStateError_rad', ...
                                              'RMS_Error_rad', 'MaxControlEffort_Nm', 'RMS_ControlEffort_Nm', ...
                                              'PhaseMargin_deg'});
writetable(final_metrics_table, fullfile(tables_dir, 'final_candidate_metrics.csv'));
writetable(p_vs_pi_table, fullfile(tables_dir, 'p_vs_pi_comparison.csv'));

fprintf('11. Numerical Tables Exported to 06_RESULTS/tables/:\n');
fprintf('    - complete_sweep_results.csv\n');
fprintf('    - final_candidate_metrics.csv\n');
fprintf('    - p_vs_pi_comparison.csv\n\n');

%% =========================================================================
% SECTION 9: SCIENTIFIC FIGURE GENERATION
% =========================================================================

fig_dir = fullfile(paths.results, 'figures');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end

% --- Figure 1: 2D Stability & Routh-Hurwitz Boundary Map ---
f1 = figure('Name', 'Phase 4 - Stability Map', 'Position', [50, 50, 700, 500]);
imagesc(Kp_values, Ki_values, map_stable);
set(gca, 'YDir', 'normal');
colormap(f1, [0.85 0.3 0.3; 0.3 0.75 0.3]); % Red = Unstable, Green = Stable
hold on;
plot(Kp_values, max_stable_ratio * Kp_values, 'k--', 'LineWidth', 2);
title('Figure 1: Closed-Loop Stability Map & Analytical Routh Boundary');
xlabel('Proportional Gain K_p');
ylabel('Integral Gain K_i');
legend('Numerical Stability Region', 'Analytical Boundary K_i = 2 K_p', 'Location', 'northwest');
grid on;
saveas(f1, fullfile(fig_dir, 'phase4_fig01_stability_map.png'));

% --- Figure 2: Rise Time Map ---
f2 = figure('Name', 'Phase 4 - Rise Time Map', 'Position', [70, 70, 700, 500]);
contourf(Kp_values, Ki_values, map_rise, 20, 'LineColor', 'none');
colorbar;
title('Figure 2: Rise Time Map t_r(K_p, K_i) [seconds]');
xlabel('Proportional Gain K_p');
ylabel('Integral Gain K_i');
hold on;
contour(Kp_values, Ki_values, map_rise, [rise_target rise_target], 'r-', 'LineWidth', 2);
legend('Rise Time [s]', sprintf('Target Boundary t_r = %.1f s', rise_target), 'Location', 'northwest');
grid on;
saveas(f2, fullfile(fig_dir, 'phase4_fig02_rise_time_map.png'));

% --- Figure 3: Settling Time Map ---
f3 = figure('Name', 'Phase 4 - Settling Time Map', 'Position', [90, 90, 700, 500]);
contourf(Kp_values, Ki_values, map_settling, 20, 'LineColor', 'none');
colorbar;
title('Figure 3: Settling Time Map t_s(K_p, K_i) [seconds]');
xlabel('Proportional Gain K_p');
ylabel('Integral Gain K_i');
hold on;
contour(Kp_values, Ki_values, map_settling, [settling_target settling_target], 'r-', 'LineWidth', 2);
legend('Settling Time [s]', sprintf('Target Boundary t_s = %.1f s', settling_target), 'Location', 'northwest');
grid on;
saveas(f3, fullfile(fig_dir, 'phase4_fig03_settling_time_map.png'));

% --- Figure 4: Overshoot Map ---
f4 = figure('Name', 'Phase 4 - Overshoot Map', 'Position', [110, 110, 700, 500]);
contourf(Kp_values, Ki_values, map_overshoot, 20, 'LineColor', 'none');
colorbar;
title('Figure 4: Percent Overshoot Map M_p(K_p, K_i) [%]');
xlabel('Proportional Gain K_p');
ylabel('Integral Gain K_i');
hold on;
contour(Kp_values, Ki_values, map_overshoot, [overshoot_target overshoot_target], 'r-', 'LineWidth', 2);
legend('Overshoot [%]', sprintf('Target Boundary M_p = %.1f%%', overshoot_target), 'Location', 'northwest');
grid on;
saveas(f4, fullfile(fig_dir, 'phase4_fig04_overshoot_map.png'));

% --- Figure 5: Steady-State Error Map ---
f5 = figure('Name', 'Phase 4 - Steady-State Error Map', 'Position', [130, 130, 700, 500]);
contourf(Kp_values, Ki_values, map_ess, 20, 'LineColor', 'none');
colorbar;
title('Figure 5: Steady-State Error Map e_{ss}(K_p, K_i) [rad]');
xlabel('Proportional Gain K_p');
ylabel('Integral Gain K_i');
grid on;
saveas(f5, fullfile(fig_dir, 'phase4_fig05_steady_state_error_map.png'));

% --- Figure 6: RMS Error Map ---
f6 = figure('Name', 'Phase 4 - RMS Tracking Error Map', 'Position', [150, 150, 700, 500]);
contourf(Kp_values, Ki_values, map_e_rms, 20, 'LineColor', 'none');
colorbar;
title('Figure 6: RMS Tracking Error Map e_{RMS}(K_p, K_i) [rad]');
xlabel('Proportional Gain K_p');
ylabel('Integral Gain K_i');
grid on;
saveas(f6, fullfile(fig_dir, 'phase4_fig06_rms_error_map.png'));

% --- Figure 7: Maximum Control Effort Map ---
f7 = figure('Name', 'Phase 4 - Max Control Effort Map', 'Position', [170, 170, 700, 500]);
contourf(Kp_values, Ki_values, map_u_max, 20, 'LineColor', 'none');
colorbar;
title('Figure 7: Peak Control Effort Map u_{max}(K_p, K_i) [N.m]');
xlabel('Proportional Gain K_p');
ylabel('Integral Gain K_i');
grid on;
saveas(f7, fullfile(fig_dir, 'phase4_fig07_control_effort_map.png'));

% --- Figure 8: Feasibility Map ---
f8 = figure('Name', 'Phase 4 - Feasibility Map', 'Position', [190, 190, 700, 500]);
feas_classification = zeros(n_Ki, n_Kp);
feas_classification(map_stable) = 1;      % Stable but violating
feas_classification(map_feasible) = 2;    % Fully feasible
imagesc(Kp_values, Ki_values, feas_classification);
set(gca, 'YDir', 'normal');
colormap(f8, [0.8 0.2 0.2; 0.95 0.85 0.3; 0.2 0.7 0.3]); % Red, Yellow, Green
colorbar('Ticks', [0.33, 1.0, 1.66], 'TickLabels', {'Unstable', 'Stable Non-Feasible', 'Fully Feasible'});
hold on;
plot(Kp_final, Ki_final, 'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'c', 'LineWidth', 2);
title('Figure 8: Constraint Feasibility Map & Selected Candidate');
xlabel('Proportional Gain K_p');
ylabel('Integral Gain K_i');
legend('', sprintf('Selected Candidate (Kp=%.2f, Ki=%.2f)', Kp_final, Ki_final), 'Location', 'northwest');
grid on;
saveas(f8, fullfile(fig_dir, 'phase4_fig08_feasibility_map.png'));

% --- Figure 9: 3D Pareto Front Visualization ---
f9 = figure('Name', 'Phase 4 - Pareto Front', 'Position', [210, 210, 800, 600]);
scatter3(sweep_table.RiseTime(feasible_indices), ...
         sweep_table.SettlingTime(feasible_indices), ...
         sweep_table.Overshoot(feasible_indices), ...
         25, [0.7 0.7 0.7], 'filled');
hold on;
scatter3(pareto_table.RiseTime, pareto_table.SettlingTime, pareto_table.Overshoot, ...
         70, [0.1 0.3 0.9], 'filled');
scatter3(selected_row.RiseTime, selected_row.SettlingTime, selected_row.Overshoot, ...
         160, 'p', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
grid on;
xlabel('Rise Time t_r [s]');
ylabel('Settling Time t_s [s]');
zlabel('Overshoot M_p [%]');
title('Figure 9: 3D Pareto Performance Trade-off Space');
legend('Feasible Design Space', 'Pareto-Optimal Frontier', ...
       sprintf('Selected Controller (Kp=%.2f, Ki=%.2f)', Kp_final, Ki_final), 'Location', 'best');
view(45, 30);
saveas(f9, fullfile(fig_dir, 'phase4_fig09_pareto_front.png'));

% --- Figure 10: Final PI Step Response ---
f10 = figure('Name', 'Phase 4 - Final PI Step Response', 'Position', [230, 230, 750, 480]);
plot(t_val_out, ones(size(t_val_out)), 'k--', 'LineWidth', 1.2);
hold on;
plot(t_val_out, y_val, 'b-', 'LineWidth', 2);
yline(1.0 + info_final.Overshoot/100, 'g:', sprintf('Peak = %.3f rad (Mp=%.1f%%)', info_final.Peak, info_final.Overshoot));
yline(1.02, 'r:', 'Upper Settling Bound (+2%)');
yline(0.98, 'r:', 'Lower Settling Bound (-2%)');
grid on;
title(sprintf('Figure 10: Validated PI Step Response \\theta(t) (K_p=%.2f, K_i=%.2f)', Kp_final, Ki_final));
xlabel('Time [s]');
ylabel('Body Pitch \\theta(t) [rad]');
legend('Reference \\theta_d', 'Actual Pitch \\theta', 'Location', 'southeast');
saveas(f10, fullfile(fig_dir, 'phase4_fig10_final_step_response.png'));

% --- Figure 11: Final PI Tracking Error ---
f11 = figure('Name', 'Phase 4 - PI Tracking Error', 'Position', [250, 250, 750, 450]);
plot(t_val_out, e_val, 'r-', 'LineWidth', 2);
grid on;
title(sprintf('Figure 11: Validated PI Tracking Error e(t) (e_{ss} = %.6f rad, e_{RMS} = %.4f rad)', ...
      abs(1.0 - dcgain(T_final)), val_e_rms));
xlabel('Time [s]');
ylabel('Tracking Error e(t) [rad]');
saveas(f11, fullfile(fig_dir, 'phase4_fig11_tracking_error.png'));

% --- Figure 12: Final PI Control Effort ---
f12 = figure('Name', 'Phase 4 - PI Control Effort', 'Position', [270, 270, 750, 450]);
plot(t_val_out, u_val, 'm-', 'LineWidth', 2);
grid on;
title(sprintf('Figure 12: Validated PI Control Torque u(t) (u_{max} = %.2f N.m, u_{RMS} = %.2f N.m)', ...
      val_u_max, val_u_rms));
xlabel('Time [s]');
ylabel('Torque T_c(t) [N.m]');
saveas(f12, fullfile(fig_dir, 'phase4_fig12_control_effort.png'));

% --- Figure 13: Final PI Closed-Loop Poles ---
f13 = figure('Name', 'Phase 4 - Closed-Loop Poles', 'Position', [290, 290, 700, 500]);
pzmap(T_final);
grid on;
title(sprintf('Figure 13: Final Closed-Loop Pole Locations in s-Plane (K_p=%.2f, K_i=%.2f)', Kp_final, Ki_final));
saveas(f13, fullfile(fig_dir, 'phase4_fig13_closed_loop_poles.png'));

% --- Figure 14: Step Response Comparison P vs PI ---
f14 = figure('Name', 'Phase 4 - P vs PI Step Response', 'Position', [310, 310, 800, 520]);
plot(t_val_out, ones(size(t_val_out)), 'k--', 'LineWidth', 1.2);
hold on;
plot(t_val_out, y_P, 'r--', 'LineWidth', 2);
plot(t_val_out, y_val, 'b-', 'LineWidth', 2);
grid on;
title('Figure 14: Step Response Comparison — Baseline P (K_p=0.30) vs Validated PI');
xlabel('Time [s]');
ylabel('Body Pitch \\theta(t) [rad]');
legend('Unit Reference \\theta_d', ...
       sprintf('Phase 3 P-Controller (e_{ss}=%.2f rad)', P_ess), ...
       sprintf('Phase 4 PI-Controller (e_{ss}=%.4f rad)', abs(1.0 - dcgain(T_final))), ...
       'Location', 'southeast');
saveas(f14, fullfile(fig_dir, 'phase4_fig14_p_vs_pi_step_response.png'));

% --- Figure 15: P vs PI Metric Comparison Bar Chart ---
f15 = figure('Name', 'Phase 4 - P vs PI Metric Comparison', 'Position', [330, 330, 800, 480]);
bar_metrics = [info_P.RiseTime, info_final.RiseTime; ...
               info_P.SettlingTime, info_final.SettlingTime; ...
               info_P.Overshoot, info_final.Overshoot; ...
               P_ess, abs(1.0 - dcgain(T_final)); ...
               P_e_rms, val_e_rms];
b = bar(bar_metrics);
b(1).FaceColor = [0.8 0.3 0.3];
b(2).FaceColor = [0.2 0.5 0.8];
set(gca, 'XTickLabel', {'Rise Time [s]', 'Settling Time [s]', 'Overshoot [%]', 'e_{ss} [rad]', 'e_{RMS} [rad]'});
title('Figure 15: Quantitative Performance Comparison: P vs PI Controller');
legend('Baseline P (K_p=0.30)', sprintf('Selected PI (K_p=%.2f, K_i=%.2f)', Kp_final, Ki_final), 'Location', 'northeast');
grid on;
saveas(f15, fullfile(fig_dir, 'phase4_fig15_p_vs_pi_metric_comparison.png'));

% --- Figure 16: Open-Loop Bode & Stability Margins ---
f16 = figure('Name', 'Phase 4 - Loop Frequency Response', 'Position', [350, 350, 750, 550]);
margin(L_final);
grid on;
title('Figure 16: Open-Loop Frequency Response & Stability Margins L(s) = C_{final} G');
saveas(f16, fullfile(fig_dir, 'phase4_fig16_bode_stability_margins.png'));

fprintf('12. All 16 Engineering Figures Successfully Saved to 06_RESULTS/figures/\n');

print_banner('Phase 4 Execution Complete', ...
    sprintf('Controller Frozen: C(s) = %.4f + %.4f/s | Output saved to 07_DATA/processed/', Kp_final, Ki_final));
