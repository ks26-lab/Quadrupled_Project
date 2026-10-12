%% ============================================================
% QUADRUPED ROBOT BODY-PITCH CONTROL
% PHASE 4 - PI CONTROLLER DESIGN AND VALIDATION
%
% SISO CONTROL SYSTEM
%
% Input  : Desired body pitch angle theta_d
% Output : Actual body pitch angle theta
%
% Phase 4:
%   4.1 Initial PI Controller
%   4.2 Systematic PI Gain Tuning
%   4.3 Pareto Performance Trade-off Analysis
%   4.4 Automatic Final PI Candidate Selection
%
% Final controller will be frozen after Phase 4.4.
% ============================================================

clear;
clc;
close all;


%% ============================================================
% PHASE 4.1 - SYSTEM DEFINITION
% ============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('PHASE 4.1 - INITIAL PI CONTROLLER DESIGN\n');
fprintf('============================================================\n');


%% ------------------------------------------------------------
% 1. Physical model parameters
% ------------------------------------------------------------

J = 0.05;       % Body pitch inertia [kg.m^2]
B = 0.10;       % Pitch damping coefficient [N.m.s/rad]
K = 0.00;       % Restoring stiffness [N.m/rad]


fprintf('\nPlant parameters:\n');

fprintf('J = %.4f kg.m^2\n', J);
fprintf('B = %.4f N.m.s/rad\n', B);
fprintf('K = %.4f N.m/rad\n', K);


%% ------------------------------------------------------------
% 2. Laplace variable
% ------------------------------------------------------------

s = tf('s');


%% ------------------------------------------------------------
% 3. Plant transfer function
%
% Physical model:
%
% J*theta_ddot + B*theta_dot + K*theta = Tc
%
% Therefore:
%
% G(s) = Theta(s)/Tc(s)
%      = 1/(J*s^2 + B*s + K)
% ------------------------------------------------------------

G = 1/(J*s^2 + B*s + K);


fprintf('\nPlant G(s):\n');

G


%% ------------------------------------------------------------
% 4. Initial PI controller
%
% C(s) = Kp + Ki/s
% ------------------------------------------------------------

Kp_initial = 0.25;
Ki_initial = 0.20;


C_PI = Kp_initial + Ki_initial/s;


fprintf('\nInitial PI Controller C(s):\n');

C_PI


%% ------------------------------------------------------------
% 5. Initial closed-loop transfer function
% ------------------------------------------------------------

T_PI = feedback(C_PI * G, 1);


fprintf('\nInitial closed-loop PI transfer function:\n');

T_PI


%% ------------------------------------------------------------
% 6. Initial closed-loop poles
% ------------------------------------------------------------

poles_PI = pole(T_PI);

fprintf('\nClosed-loop poles:\n');

disp(poles_PI);


%% ------------------------------------------------------------
% 7. Initial stability check
% ------------------------------------------------------------

stability_tolerance = 1e-8;


if all(real(poles_PI) < -stability_tolerance)

    fprintf('System is asymptotically stable.\n');

elseif any(real(poles_PI) > stability_tolerance)

    fprintf('System is unstable.\n');

else

    fprintf('System has poles on or very close to the imaginary axis.\n');

end


%% ------------------------------------------------------------
% 8. Initial PI step response
% ------------------------------------------------------------

%figure;

step(T_PI);

grid on;

title('Phase 4.1 - Initial PI Step Response');

xlabel('Time (s)');
ylabel('Body Pitch (rad)');


%% ------------------------------------------------------------
% 9. Initial PI performance
% ------------------------------------------------------------

info_PI = stepinfo(T_PI);


fprintf('\nInitial PI step-response information:\n');

fprintf('Rise Time      = %.4f s\n', ...
    info_PI.RiseTime);

fprintf('Settling Time  = %.4f s\n', ...
    info_PI.SettlingTime);

fprintf('Overshoot      = %.4f %%\n', ...
    info_PI.Overshoot);

fprintf('Peak           = %.4f rad\n', ...
    info_PI.Peak);

fprintf('Peak Time      = %.4f s\n', ...
    info_PI.PeakTime);


%% ------------------------------------------------------------
% 10. Reference steady-state error
% ------------------------------------------------------------

reference = 1;

dc_gain_PI = dcgain(T_PI);

reference_steady_state_error = ...
    abs(reference - dc_gain_PI);


fprintf('\nReference steady-state error = %.8f rad\n', ...
    reference_steady_state_error);


%% ============================================================
% PHASE 4.2 - SYSTEMATIC PI GAIN TUNING
% ============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('PHASE 4.2 - SYSTEMATIC PI GAIN TUNING\n');
fprintf('============================================================\n');


%% ------------------------------------------------------------
% 11. Gain search ranges
% ------------------------------------------------------------

Kp_values = 0.01:0.01:0.50;
Ki_values = 0.01:0.01:0.50;


fprintf('\nKp search range: %.2f to %.2f\n', ...
    min(Kp_values), max(Kp_values));

fprintf('Ki search range: %.2f to %.2f\n', ...
    min(Ki_values), max(Ki_values));


%% ------------------------------------------------------------
% 12. Performance constraints
%
% Original targets:
%
% Rise time <= 2 s
% Settling time <= 5 s
% Overshoot <= 20 %
%
% These will be evaluated, but the final selection will use
% the new priority-based strategy.
% ------------------------------------------------------------

rise_target = 2.0;
settling_target = 5.0;
overshoot_target = 20.0;


%% ------------------------------------------------------------
% 13. Storage arrays
% ------------------------------------------------------------

results = [];

stable_count = 0;


%% ------------------------------------------------------------
% 14. Systematic gain search
% ------------------------------------------------------------

for Kp = Kp_values

    for Ki = Ki_values

        % PI controller
        C_test = Kp + Ki/s;

        % Closed-loop system
        T_test = feedback(C_test * G, 1);

        % Poles
        poles_test = pole(T_test);

        % Stability check
        is_stable = all(real(poles_test) < ...
            -stability_tolerance);

        if ~is_stable
            continue;
        end


        stable_count = stable_count + 1;


        % Step-response information
        info_test = stepinfo(T_test);


        % Store result
        results = [results; ...
            Kp, ...
            Ki, ...
            info_test.RiseTime, ...
            info_test.SettlingTime, ...
            info_test.Overshoot, ...
            info_test.Peak, ...
            info_test.PeakTime];


    end

end


%% ------------------------------------------------------------
% 15. Convert results into table
% ------------------------------------------------------------

results_table = array2table( ...
    results, ...
    'VariableNames', { ...
    'Kp', ...
    'Ki', ...
    'RiseTime', ...
    'SettlingTime', ...
    'Overshoot', ...
    'Peak', ...
    'PeakTime'});


fprintf('\nNumber of stable PI candidates: %d\n', ...
    height(results_table));


%% ------------------------------------------------------------
% 16. Check original three-target requirement
% ------------------------------------------------------------

all_targets = ...
    results_table.RiseTime <= rise_target & ...
    results_table.SettlingTime <= settling_target & ...
    results_table.Overshoot <= overshoot_target;


feasible_all = results_table(all_targets, :);


fprintf('\nNumber of candidates meeting ALL original targets: %d\n', ...
    height(feasible_all));


if height(feasible_all) == 0

    fprintf('\nNo candidate satisfies all original targets.\n');

    fprintf('The controller will therefore be selected using\n');
    fprintf('a prioritized performance strategy.\n');

end


%% ------------------------------------------------------------
% 17. Display feasible candidates if available
% ------------------------------------------------------------

if height(feasible_all) > 0

    fprintf('\nCandidates satisfying all targets:\n');

    disp(feasible_all);

else

    fprintf('\nNo fully feasible PI gain combination.\n');

end


%% ============================================================
% PHASE 4.3 - PI PERFORMANCE TRADE-OFF ANALYSIS
% ============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('PHASE 4.3 - PI PERFORMANCE TRADE-OFF ANALYSIS\n');
fprintf('============================================================\n');


%% ------------------------------------------------------------
% 18. Pareto-optimal candidates
%
% Objectives:
%   Minimize Rise Time
%   Minimize Settling Time
%   Minimize Overshoot
% ------------------------------------------------------------

N = height(results_table);

is_pareto = true(N,1);


for i = 1:N

    for j = 1:N

        if i == j
            continue;
        end


        candidate_j_dominates_i = ...
            results_table.RiseTime(j) <= ...
                results_table.RiseTime(i) && ...
            results_table.SettlingTime(j) <= ...
                results_table.SettlingTime(i) && ...
            results_table.Overshoot(j) <= ...
                results_table.Overshoot(i) && ...
            ( ...
            results_table.RiseTime(j) < ...
                results_table.RiseTime(i) || ...
            results_table.SettlingTime(j) < ...
                results_table.SettlingTime(i) || ...
            results_table.Overshoot(j) < ...
                results_table.Overshoot(i) ...
            );


        if candidate_j_dominates_i

            is_pareto(i) = false;

            break;

        end

    end

end


pareto_table = results_table(is_pareto,:);


fprintf('\nNumber of Pareto-optimal candidates: %d\n', ...
    height(pareto_table));


fprintf('\nPareto-optimal PI gain combinations:\n');

disp(pareto_table(:, ...
    {'Kp','Ki','RiseTime','SettlingTime','Overshoot'}));


%% ------------------------------------------------------------
% 19. Fastest Pareto candidate
% ------------------------------------------------------------

[~, idx_fast] = ...
    min(pareto_table.SettlingTime);


fastest_candidate = ...
    pareto_table(idx_fast,:);


fprintf('\n');
fprintf('============================================================\n');
fprintf('FASTEST PARETO CANDIDATE\n');
fprintf('============================================================\n');

fprintf('\nKp = %.4f\n', ...
    fastest_candidate.Kp);

fprintf('Ki = %.4f\n', ...
    fastest_candidate.Ki);

fprintf('Rise Time = %.4f s\n', ...
    fastest_candidate.RiseTime);

fprintf('Settling Time = %.4f s\n', ...
    fastest_candidate.SettlingTime);

fprintf('Overshoot = %.4f %%\n', ...
    fastest_candidate.Overshoot);


%% ------------------------------------------------------------
% 20. Lowest-overshoot Pareto candidate
% ------------------------------------------------------------

[~, idx_lowOS] = ...
    min(pareto_table.Overshoot);


lowest_OS_candidate = ...
    pareto_table(idx_lowOS,:);


fprintf('\n');
fprintf('============================================================\n');
fprintf('LOWEST-OVERSHOOT PARETO CANDIDATE\n');
fprintf('============================================================\n');

fprintf('\nKp = %.4f\n', ...
    lowest_OS_candidate.Kp);

fprintf('Ki = %.4f\n', ...
    lowest_OS_candidate.Ki);

fprintf('Rise Time = %.4f s\n', ...
    lowest_OS_candidate.RiseTime);

fprintf('Settling Time = %.4f s\n', ...
    lowest_OS_candidate.SettlingTime);

fprintf('Overshoot = %.4f %%\n', ...
    lowest_OS_candidate.Overshoot);


%% ------------------------------------------------------------
% 21. Pareto-front plot
% ------------------------------------------------------------

%figure;

scatter3( ...
    pareto_table.RiseTime, ...
    pareto_table.SettlingTime, ...
    pareto_table.Overshoot, ...
    50, ...
    'filled');

grid on;

xlabel('Rise Time (s)');
ylabel('Settling Time (s)');
zlabel('Overshoot (%)');

title('Phase 4.3 - PI Pareto Front');


%% ============================================================
% PHASE 4.4 - AUTOMATIC FINAL PI CANDIDATE SELECTION
% ============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('PHASE 4.4 - AUTOMATIC FINAL PI CANDIDATE SELECTION\n');
fprintf('============================================================\n');


%% ------------------------------------------------------------
% 22. Selection philosophy
%
% Priority:
%
%   1. Stability              -> mandatory
%   2. Overshoot <= 20%       -> primary criterion
%   3. Minimum settling time  -> secondary criterion
%
% Rise time is reported but is not a hard rejection criterion.
%
% FALLBACK:
%
% If no candidate satisfies Overshoot <= 20%,
% select the candidate closest to the overshoot target.
% ------------------------------------------------------------

max_overshoot = 20.0;


fprintf('\nSelection hierarchy:\n');

fprintf('1. Stable system\n');
fprintf('2. Overshoot <= %.2f %%\n', max_overshoot);
fprintf('3. Minimum settling time\n');

fprintf('\nRise time is treated as a reported metric.\n');


%% ------------------------------------------------------------
% 23. Find candidates satisfying primary criterion
% ------------------------------------------------------------

feasible_primary = ...
    results_table( ...
    results_table.Overshoot <= max_overshoot, :);


fprintf('\nCandidates satisfying Overshoot <= %.2f %%: %d\n', ...
    max_overshoot, ...
    height(feasible_primary));


%% ------------------------------------------------------------
% 24. Automatic selection / fallback
% ------------------------------------------------------------

if height(feasible_primary) > 0

    % ---------------------------------------------------------
    % Normal case:
    % Select minimum settling time among acceptable
    % overshoot candidates.
    % ---------------------------------------------------------

    [~, idx_selected] = ...
        min(feasible_primary.SettlingTime);


    selected_candidate = ...
        feasible_primary(idx_selected,:);


    selection_status = ...
        'Primary selection: Overshoot constraint satisfied';


else

    % ---------------------------------------------------------
    % FALLBACK CASE
    %
    % No candidate satisfies overshoot <= 20%.
    %
    % Select candidate with smallest normalized overshoot
    % violation.
    % ---------------------------------------------------------

    fprintf('\nWARNING:\n');

    fprintf(['No candidate satisfies the overshoot ', ...
        'constraint.\n']);

    fprintf('Activating closest-candidate fallback...\n');


    if height(results_table) == 0

        error(['No stable PI candidates were found. ', ...
            'Gain search cannot continue.']);

    end


    % Overshoot violation
    overshoot_violation = ...
        max( ...
        0, ...
        results_table.Overshoot - max_overshoot) ...
        ./ max_overshoot;


    % Find smallest violation
    [~, idx_selected] = ...
        min(overshoot_violation);


    selected_candidate = ...
        results_table(idx_selected,:);


    selection_status = ...
        ['Fallback selection: closest candidate ', ...
         'to overshoot target'];

end


%% ------------------------------------------------------------
% 25. Display selected candidate
% ------------------------------------------------------------

fprintf('\n');
fprintf('============================================================\n');
fprintf('AUTOMATIC PI CANDIDATE SELECTION RESULT\n');
fprintf('============================================================\n');


fprintf('\nSelection status:\n');

fprintf('%s\n', selection_status);


fprintf('\nSelected PI gains:\n');

fprintf('Kp = %.4f\n', ...
    selected_candidate.Kp);

fprintf('Ki = %.4f\n', ...
    selected_candidate.Ki);


fprintf('\nSelected performance:\n');

fprintf('Rise Time     = %.4f s\n', ...
    selected_candidate.RiseTime);

fprintf('Settling Time = %.4f s\n', ...
    selected_candidate.SettlingTime);

fprintf('Overshoot     = %.4f %%\n', ...
    selected_candidate.Overshoot);


%% ------------------------------------------------------------
% 26. Freeze selected gains
% ------------------------------------------------------------

Kp_final = selected_candidate.Kp;
Ki_final = selected_candidate.Ki;


fprintf('\n');
fprintf('FINAL PI GAINS TO BE FROZEN:\n');

fprintf('Kp_final = %.4f\n', Kp_final);

fprintf('Ki_final = %.4f\n', Ki_final);


%% ------------------------------------------------------------
% 27. Construct final PI controller
% ------------------------------------------------------------

C_final = Kp_final + Ki_final/s;


fprintf('\nFinal PI Controller C_final(s):\n');

C_final


%% ------------------------------------------------------------
% 28. Final closed-loop transfer function
% ------------------------------------------------------------

T_final = feedback(C_final * G, 1);


fprintf('\nFinal closed-loop transfer function T_final(s):\n');

T_final


%% ------------------------------------------------------------
% 29. Final closed-loop poles
% ------------------------------------------------------------

poles_final = pole(T_final);


fprintf('\nFinal closed-loop poles:\n');

disp(poles_final);


%% ------------------------------------------------------------
% 30. Final stability verification
% ------------------------------------------------------------

if all(real(poles_final) < -stability_tolerance)

    fprintf('Final system is asymptotically stable.\n');

elseif any(real(poles_final) > stability_tolerance)

    fprintf('Final system is unstable.\n');

else

    fprintf(['Final system has poles on or very close ', ...
        'to the imaginary axis.\n']);

end


%% ------------------------------------------------------------
% 31. Final step response
% ------------------------------------------------------------

%   figure;

step(T_final);

grid on;

title('Phase 4.4 - Final Selected PI Step Response');

xlabel('Time (s)');
ylabel('Body Pitch (rad)');


%% ------------------------------------------------------------
% 32. Final performance information
% ------------------------------------------------------------

info_final = stepinfo(T_final);


fprintf('\n');
fprintf('============================================================\n');
fprintf('FINAL PI PERFORMANCE\n');
fprintf('============================================================\n');


fprintf('\nRise Time      = %.4f s\n', ...
    info_final.RiseTime);

fprintf('Settling Time  = %.4f s\n', ...
    info_final.SettlingTime);

fprintf('Overshoot      = %.4f %%\n', ...
    info_final.Overshoot);

fprintf('Peak           = %.4f rad\n', ...
    info_final.Peak);

fprintf('Peak Time      = %.4f s\n', ...
    info_final.PeakTime);


%% ------------------------------------------------------------
% 33. Final steady-state tracking error
% ------------------------------------------------------------

dc_gain_final = dcgain(T_final);

steady_state_error = ...
    abs(reference - dc_gain_final);


fprintf('\n');
fprintf('FINAL STEADY-STATE TRACKING\n');
fprintf('----------------------------\n');

fprintf('Final output       = %.8f rad\n', ...
    dc_gain_final);

fprintf('Steady-state error = %.8f rad\n', ...
    steady_state_error);


%% ------------------------------------------------------------
% 34. Compare final candidate with Pareto extremes
% ------------------------------------------------------------

T_fast = feedback( ...
    (fastest_candidate.Kp + ...
     fastest_candidate.Ki/s) * G, ...
    1);


T_lowOS = feedback( ...
    (lowest_OS_candidate.Kp + ...
     lowest_OS_candidate.Ki/s) * G, ...
    1);


info_fast = stepinfo(T_fast);

info_lowOS = stepinfo(T_lowOS);


%% ------------------------------------------------------------
% 35. Candidate comparison
% ------------------------------------------------------------

fprintf('\n');
fprintf('============================================================\n');
fprintf('PI CANDIDATE COMPARISON\n');
fprintf('============================================================\n');


fprintf('\nFastest Pareto candidate:\n');

fprintf('Kp = %.4f\n', ...
    fastest_candidate.Kp);

fprintf('Ki = %.4f\n', ...
    fastest_candidate.Ki);

fprintf('Rise Time     = %.4f s\n', ...
    info_fast.RiseTime);

fprintf('Settling Time = %.4f s\n', ...
    info_fast.SettlingTime);

fprintf('Overshoot     = %.4f %%\n', ...
    info_fast.Overshoot);


fprintf('\nFINAL SELECTED CANDIDATE:\n');

fprintf('Kp = %.4f\n', ...
    Kp_final);

fprintf('Ki = %.4f\n', ...
    Ki_final);

fprintf('Rise Time     = %.4f s\n', ...
    info_final.RiseTime);

fprintf('Settling Time = %.4f s\n', ...
    info_final.SettlingTime);

fprintf('Overshoot     = %.4f %%\n', ...
    info_final.Overshoot);


fprintf('\nLowest-overshoot Pareto candidate:\n');

fprintf('Kp = %.4f\n', ...
    lowest_OS_candidate.Kp);

fprintf('Ki = %.4f\n', ...
    lowest_OS_candidate.Ki);

fprintf('Rise Time     = %.4f s\n', ...
    info_lowOS.RiseTime);

fprintf('Settling Time = %.4f s\n', ...
    info_lowOS.SettlingTime);

fprintf('Overshoot     = %.4f %%\n', ...
    info_lowOS.Overshoot);


%% ------------------------------------------------------------
% 36. Comparison plot
% ------------------------------------------------------------

%figure;

step(T_fast, T_final, T_lowOS);

grid on;

title('Phase 4.4 - PI Candidate Comparison');

xlabel('Time (s)');
ylabel('Body Pitch (rad)');

legend( ...
    sprintf('Fastest Pareto: Kp=%.2f, Ki=%.2f', ...
        fastest_candidate.Kp, ...
        fastest_candidate.Ki), ...
    sprintf('FINAL: Kp=%.2f, Ki=%.2f', ...
        Kp_final, ...
        Ki_final), ...
    sprintf('Lowest Overshoot: Kp=%.2f, Ki=%.2f', ...
        lowest_OS_candidate.Kp, ...
        lowest_OS_candidate.Ki), ...
    'Location', 'best');


%% ============================================================
% PHASE 4.4 COMPLETE
% ============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('PHASE 4.4 COMPLETE - PI CONTROLLER FROZEN\n');
fprintf('============================================================\n');

fprintf('\nFrozen controller:\n');

fprintf('Kp_final = %.4f\n', Kp_final);
fprintf('Ki_final = %.4f\n', Ki_final);


fprintf('\nSelection basis:\n');

fprintf('Stable system\n');
fprintf('Overshoot-priority selection\n');
fprintf('Minimum settling time among acceptable candidates\n');
fprintf('Closest-candidate fallback if required\n');


fprintf('\nNext stage:\n');

fprintf('PHASE 4.5 - DISTURBANCE REJECTION VALIDATION\n');

fprintf('\n============================================================\n');

%% ============================================================
% PHASE 4.5 - DISTURBANCE REJECTION VALIDATION
%
% Disturbance enters the plant as an equivalent pitch torque.
%
% No controller gains or physical disturbance magnitudes are
% manually re-entered here.
% ============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('PHASE 4.5 - DISTURBANCE REJECTION VALIDATION\n');
fprintf('============================================================\n');


%% ------------------------------------------------------------
% 37. Disturbance transfer function
%
% theta/Td = G/(1 + C_final*G)
% ------------------------------------------------------------

G_disturbance = minreal( ...
    G / (1 + C_final * G));


fprintf('\nDisturbance transfer function Gd(s):\n');

G_disturbance


%% ------------------------------------------------------------
% 38. Theoretical disturbance steady-state gain
% ------------------------------------------------------------

disturbance_dc_gain = dcgain(G_disturbance);


fprintf('\nTheoretical disturbance DC gain:\n');

fprintf('Gd(0) = %.8f rad/(N.m)\n', ...
    disturbance_dc_gain);


%% ------------------------------------------------------------
% 39. Generate nominal reference response
%
% MATLAB automatically determines an appropriate simulation
% time vector from the closed-loop dynamics.
% ------------------------------------------------------------

[theta_nominal, t_nominal] = ...
    step(T_final);


%% ------------------------------------------------------------
% 40. Control effort generated by nominal reference
%
% U/R = C/(1 + C*G)
% ------------------------------------------------------------

T_control_reference = ...
    minreal( ...
    feedback(C_final, G));


[u_nominal, ~] = ...
    step(T_control_reference, t_nominal);


%% ------------------------------------------------------------
% 41. Automatically determine disturbance magnitude
%
% Disturbance magnitude is defined from the maximum nominal
% control torque generated by the frozen controller.
%
% This avoids arbitrarily selecting a disturbance magnitude.
% ------------------------------------------------------------

disturbance_torque = ...
    max(abs(u_nominal));


fprintf('\nAutomatically generated disturbance level:\n');

fprintf('|Td| = %.8f N.m\n', ...
    disturbance_torque);


%% ------------------------------------------------------------
% 42. Automatically determine disturbance-response settling time
% ------------------------------------------------------------

info_disturbance = stepinfo(G_disturbance);

disturbance_settling_time = ...
    info_disturbance.SettlingTime;


%% ------------------------------------------------------------
% 42.1. Disturbance application time
% ------------------------------------------------------------

disturbance_start = ...
    info_final.SettlingTime;


%% ------------------------------------------------------------
% 42.2. Automatically determine total simulation duration
% ------------------------------------------------------------

simulation_horizon = ...
    disturbance_start + ...
    disturbance_settling_time;


%% ------------------------------------------------------------
% 42.3. Generate simulation time vector
% ------------------------------------------------------------

t = linspace( ...
    0, ...
    simulation_horizon, ...
    length(t_nominal) * 5).';


%% ------------------------------------------------------------
% 43. Nominal response over extended time
% ------------------------------------------------------------

[theta_nominal, ~] = ...
    step(T_final, t);





%% ============================================================
% TEST A - SUSTAINED / CONSTANT DISTURBANCE
% ============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('TEST A - SUSTAINED DISTURBANCE\n');
fprintf('============================================================\n');


%% ------------------------------------------------------------
% 45. Construct sustained disturbance
% ------------------------------------------------------------

Td_sustained = ...
    disturbance_torque * ...
    (t >= disturbance_start);


%% ------------------------------------------------------------
% 46. Simulate disturbance-induced pitch deviation
% ------------------------------------------------------------

theta_disturbance_sustained = ...
    lsim( ...
    G_disturbance, ...
    Td_sustained, ...
    t);


%% ------------------------------------------------------------
% 47. Total body-pitch response
% ------------------------------------------------------------

theta_total_sustained = ...
    theta_nominal + ...
    theta_disturbance_sustained;


%% ------------------------------------------------------------
% 48. Sustained disturbance metrics
% ------------------------------------------------------------

peak_deviation_sustained = ...
    max(abs(theta_disturbance_sustained));


final_deviation_sustained = ...
    abs(theta_disturbance_sustained(end));


theoretical_final_deviation = ...
    abs( ...
    disturbance_dc_gain * ...
    disturbance_torque);


fprintf('\nPeak pitch deviation:\n');

fprintf('%.8f rad\n', ...
    peak_deviation_sustained);


fprintf('\nSimulated final disturbance deviation:\n');

fprintf('%.8f rad\n', ...
    final_deviation_sustained);


fprintf('\nTheoretical final disturbance deviation:\n');

fprintf('%.8f rad\n', ...
    theoretical_final_deviation);


%% ------------------------------------------------------------
% 49. Sustained disturbance plot
% ------------------------------------------------------------

figure;

plot( ...
    t, ...
    theta_nominal, ...
    'LineWidth', 1.5);

hold on;

plot( ...
    t, ...
    theta_total_sustained, ...
    'LineWidth', 1.5);

grid on;

xlabel('Time (s)');
ylabel('Body Pitch (rad)');

title('Phase 4.5 - Sustained Disturbance Rejection');

legend( ...
    'Nominal response', ...
    'Response with sustained disturbance', ...
    'Location', 'best');


%% ============================================================
% TEST B - TRANSIENT DISTURBANCE
% ============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('TEST B - TRANSIENT DISTURBANCE\n');
fprintf('============================================================\n');


%% ------------------------------------------------------------
% 50. Automatically determine disturbance duration
%
% Disturbance duration is based on the measured nominal
% rise time of the frozen controller.
% ------------------------------------------------------------

disturbance_duration = ...
    info_final.RiseTime;


disturbance_end = ...
    disturbance_start + ...
    disturbance_duration;


%% ------------------------------------------------------------
% 51. Construct transient disturbance
% ------------------------------------------------------------

Td_transient = ...
    disturbance_torque * ...
    ( ...
    t >= disturbance_start & ...
    t <= disturbance_end ...
    );


%% ------------------------------------------------------------
% 52. Simulate transient disturbance response
% ------------------------------------------------------------

theta_disturbance_transient = ...
    lsim( ...
    G_disturbance, ...
    Td_transient, ...
    t);


%% ------------------------------------------------------------
% 53. Total transient response
% ------------------------------------------------------------

theta_total_transient = ...
    theta_nominal + ...
    theta_disturbance_transient;


%% ------------------------------------------------------------
% 54. Transient disturbance metrics
% ------------------------------------------------------------

peak_deviation_transient = ...
    max(abs(theta_disturbance_transient));


final_deviation_transient = ...
    abs(theta_disturbance_transient(end));


fprintf('\nDisturbance duration:\n');

fprintf('%.8f s\n', ...
    disturbance_duration);


fprintf('\nPeak transient pitch deviation:\n');

fprintf('%.8f rad\n', ...
    peak_deviation_transient);


fprintf('\nFinal transient disturbance deviation:\n');

fprintf('%.8f rad\n', ...
    final_deviation_transient);


%% ------------------------------------------------------------
% 55. Transient disturbance plot
% ------------------------------------------------------------

figure;

plot( ...
    t, ...
    theta_nominal, ...
    'LineWidth', 1.5);

hold on;

plot( ...
    t, ...
    theta_total_transient, ...
    'LineWidth', 1.5);

grid on;

xlabel('Time (s)');
ylabel('Body Pitch (rad)');

title('Phase 4.5 - Transient Disturbance Rejection');

legend( ...
    'Nominal response', ...
    'Response with transient disturbance', ...
    'Location', 'best');


%% ============================================================
% TEST C - DISTURBANCE DEVIATION ONLY
% ============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('TEST C - DISTURBANCE RESPONSE ONLY\n');
fprintf('============================================================\n');


%% ------------------------------------------------------------
% 56. Plot disturbance-induced deviation
% ------------------------------------------------------------

figure;

plot( ...
    t, ...
    theta_disturbance_sustained, ...
    'LineWidth', 1.5);

hold on;

plot( ...
    t, ...
    theta_disturbance_transient, ...
    'LineWidth', 1.5);

grid on;

xlabel('Time (s)');
ylabel('\Delta Body Pitch (rad)');

title('Phase 4.5 - Disturbance-Induced Pitch Deviation');

legend( ...
    'Sustained disturbance', ...
    'Transient disturbance', ...
    'Location', 'best');


%% ============================================================
% 57. Theoretical vs simulated validation
% ============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('THEORETICAL vs SIMULATED DISTURBANCE VALIDATION\n');
fprintf('============================================================\n');


fprintf('\nTheoretical final deviation = %.10f rad\n', ...
    theoretical_final_deviation);

fprintf('Simulated final deviation   = %.10f rad\n', ...
    final_deviation_sustained);


validation_difference = ...
    abs( ...
    final_deviation_sustained - ...
    theoretical_final_deviation);


fprintf('Absolute difference          = %.10f rad\n', ...
    validation_difference);


%% ============================================================
% PHASE 4.5 COMPLETE
% ============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('PHASE 4.5 DISTURBANCE VALIDATION COMPLETE\n');
fprintf('============================================================\n');

fprintf('\nFrozen controller used:\n');

fprintf('Kp_final = %.8f\n', Kp_final);

fprintf('Ki_final = %.8f\n', Ki_final);


fprintf('\nAutomatically generated disturbance:\n');

fprintf('Td = %.8f N.m\n', ...
    disturbance_torque);


fprintf('\nSustained disturbance peak deviation:\n');

fprintf('%.8f rad\n', ...
    peak_deviation_sustained);


fprintf('\nSustained disturbance final deviation:\n');

fprintf('%.8f rad\n', ...
    final_deviation_sustained);


fprintf('\nTransient disturbance peak deviation:\n');

fprintf('%.8f rad\n', ...
    peak_deviation_transient);


fprintf('\nTransient disturbance final deviation:\n');

fprintf('%.8f rad\n', ...
    final_deviation_transient);


fprintf('\n============================================================\n');

%% ============================================================
% 58. AUTOMATIC DISTURBANCE RECOVERY ANALYSIS
% ============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('AUTOMATIC DISTURBANCE RECOVERY ANALYSIS\n');
fprintf('============================================================\n');


%% ------------------------------------------------------------
% 58.1 Define recovery tolerance automatically
%
% Recovery is evaluated relative to the peak disturbance
% deviation. A 2%% settling band is used, consistent with
% MATLAB's standard step-response settling convention.
% ------------------------------------------------------------

recovery_tolerance = ...
    0.02 * peak_deviation_sustained;


%% ------------------------------------------------------------
% 58.2 Find disturbance recovery time
%
% Recovery begins after the disturbance is applied.
% The response must remain inside the tolerance band afterward.
% ------------------------------------------------------------

recovery_error = ...
    abs(theta_disturbance_sustained);


inside_recovery_band = ...
    recovery_error <= recovery_tolerance;


recovery_time = NaN;


for i = 1:length(t)

    if t(i) < disturbance_start

        continue;

    end

    if all(inside_recovery_band(i:end))

        recovery_time = ...
            t(i) - disturbance_start;

        break;

    end

end


%% ------------------------------------------------------------
% 58.3 Report recovery time
% ------------------------------------------------------------

if isnan(recovery_time)

    fprintf('\nRecovery time could not be determined.\n');

else

    fprintf('\nRecovery time = %.8f s\n', ...
        recovery_time);

end


fprintf('\nRecovery tolerance = %.8f rad\n', ...
    recovery_tolerance);


%% ============================================================
% 59. DISTURBANCE PERFORMANCE SUMMARY
% ============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('PHASE 4.5 DISTURBANCE PERFORMANCE SUMMARY\n');
fprintf('============================================================\n');

fprintf('\nController:\n');

fprintf('Kp_final = %.8f\n', ...
    Kp_final);

fprintf('Ki_final = %.8f\n', ...
    Ki_final);


fprintf('\nDisturbance:\n');

fprintf('Automatically generated magnitude = %.8f N.m\n', ...
    disturbance_torque);


fprintf('\nSustained disturbance:\n');

fprintf('Peak pitch deviation       = %.8f rad\n', ...
    peak_deviation_sustained);

fprintf('Final pitch deviation      = %.8f rad\n', ...
    final_deviation_sustained);

fprintf('Theoretical final deviation = %.8f rad\n', ...
    theoretical_final_deviation);

fprintf('Theory-simulation difference = %.10f rad\n', ...
    validation_difference);


if ~isnan(recovery_time)

    fprintf('Recovery time              = %.8f s\n', ...
        recovery_time);

end


fprintf('\nTransient disturbance:\n');

fprintf('Disturbance duration       = %.8f s\n', ...
    disturbance_duration);

fprintf('Peak pitch deviation       = %.8f rad\n', ...
    peak_deviation_transient);

fprintf('Final pitch deviation      = %.8f rad\n', ...
    final_deviation_transient);


%% ============================================================
% 60. FINAL PHASE 4.5 VALIDATION CHECK
% ============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('PHASE 4.5 VALIDATION CHECK\n');
fprintf('============================================================\n');


%% ------------------------------------------------------------
% 60.1 Theoretical DC disturbance rejection
% ------------------------------------------------------------

if abs(disturbance_dc_gain) <= stability_tolerance

    disturbance_dc_check = true;

    fprintf('\n[PASS] Theoretical DC disturbance rejection.\n');

else

    disturbance_dc_check = false;

    fprintf('\n[CHECK] Non-zero theoretical disturbance DC gain.\n');

end


%% ------------------------------------------------------------
% 60.2 Theory vs simulation
% ------------------------------------------------------------

if validation_difference <= recovery_tolerance

    theory_simulation_check = true;

    fprintf('[PASS] Theoretical and simulated final disturbance\n');
    fprintf('       deviations agree within the recovery tolerance.\n');

else

    theory_simulation_check = false;

    fprintf('[CHECK] Theory and simulation difference exceeds\n');
    fprintf('       the recovery tolerance.\n');

end


%% ------------------------------------------------------------
% 60.3 Recovery check
% ------------------------------------------------------------

if ~isnan(recovery_time)

    recovery_check = true;

    fprintf('[PASS] Disturbance recovery detected.\n');

else

    recovery_check = false;

    fprintf('[CHECK] Disturbance recovery not detected within\n');
    fprintf('       the simulation horizon.\n');

end


%% ============================================================
% 61. PHASE 4.5 FINAL STATUS
% ============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('PHASE 4.5 FINAL STATUS\n');
fprintf('============================================================\n');


if disturbance_dc_check && ...
   theory_simulation_check && ...
   recovery_check

    fprintf('\nPHASE 4.5 VALIDATION: PASS\n');

    fprintf('\nThe frozen PI controller demonstrates:\n');
    fprintf('1. Theoretical rejection of constant disturbances.\n');
    fprintf('2. Simulated disturbance recovery.\n');
    fprintf('3. Agreement between theory and simulation.\n');

else

    fprintf('\nPHASE 4.5 VALIDATION: REQUIRES REVIEW\n');

    fprintf('\nAt least one validation criterion requires\n');
    fprintf('additional investigation before freezing Phase 4.5.\n');

end


fprintf('\n============================================================\n');
fprintf('END OF PHASE 4.5\n');
fprintf('============================================================\n');

matlabroot