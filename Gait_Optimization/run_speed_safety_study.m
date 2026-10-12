function study_results = run_speed_safety_study(speed_candidates)
% RUN_SPEED_SAFETY_STUDY Evaluates multiple gait candidates in closed-loop
% simulation with the frozen SISO pitch balance controller.
%
% Records:
%   - Forward speed, Stride length, Frequency
%   - Peak Joint Angular Velocity [rad/s]
%   - Peak Joint Torque [N.m]
%   - Peak Pitch Deviation [rad and deg]
%   - Pitch RMSE [rad]
%   - Bus Current and Power Demand [W, A]
%   - Actuator-limit violations

if nargin < 1 || isempty(speed_candidates)
    % Test suite of candidate speeds across gait parameter space:
    % Format: [step_length, frequency, step_height]
    speed_candidates = [
        0.08,  0.80,  0.035;   % Crawl / Slow Trot: 0.064 m/s
        0.10,  1.00,  0.040;   % Baseline Trot:     0.100 m/s
        0.12,  1.25,  0.038;   % Moderate Fast:     0.150 m/s
        0.14,  1.50,  0.035;   % Proposed Optimal:  0.210 m/s
        0.16,  1.75,  0.032;   % High Speed:        0.280 m/s
        0.18,  2.00,  0.030;   % Extreme Speed:     0.360 m/s
        0.20,  2.50,  0.028    % Boundary / Limit:  0.500 m/s
    ];
end

root_dir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root_dir, 'Gait_Optimization'));
addpath(fullfile(root_dir, 'Simulink_Gait'));
addpath(fullfile(root_dir, 'Simulink_Gait', 'adapters'));

params = get_dynamics_and_actuator_limits();
N_cand = size(speed_candidates, 1);

fprintf('========================================================================================\n');
fprintf('  QUADRUPED GAIT SPEED-SAFETY STUDY: CLOSED-LOOP EVALUATION\n');
fprintf('========================================================================================\n');
fprintf(' Candidate | Speed [m/s] | Stride [m] | Freq [Hz] | Max Omega [rad/s] | Max Tau [N.m] | Pitch RMSE [deg] | Feasible?\n');
fprintf('----------------------------------------------------------------------------------------\n');

results_table = cell(N_cand, 11);
study_data = cell(N_cand, 1);

dt = 0.005;
t_sim = 4.0;
time_vec = (0:dt:t_sim)';
N_steps = length(time_vec);

for c = 1:N_cand
    L_stride = speed_candidates(c, 1);
    f_gait   = speed_candidates(c, 2);
    h_step   = speed_candidates(c, 3);
    v_speed  = L_stride * f_gait;
    
    % Analytical dynamics and motor limits
    dyn = calculate_gait_dynamics(L_stride, f_gait, h_step, 0.50, params);
    
    % Closed-loop simulation with frozen SISO balance controller & trot perturbation
    theta = 0.0;
    theta_dot = 0.0;
    integrator = 0.0;
    
    theta_hist = zeros(N_steps, 1);
    u_pitch_hist = zeros(N_steps, 1);
    
    % Gait phase generator
    offsets = [0.0; 0.5; 0.5; 0.0];
    foot0 = params.standing_h * [-1; -1; -1; -1]; % baseline z
    nominalFoot = [0.15, 0.10, -params.standing_h;
                   0.15, -0.10, -params.standing_h;
                  -0.15, 0.10, -params.standing_h;
                  -0.15, -0.10, -params.standing_h];
    
    for k = 1:N_steps
        t = time_vec(k);
        
        % CPG
        [phases, stance, ~] = cpg_gait_generator(t, f_gait, offsets);
        
        % Inherent walking body pitch ripple from foot stance-shift
        % (Fore-aft stance transition produces dynamic pitch torque ripple)
        tau_ripple = 0.04 * (v_speed / 0.10) * sin(2 * pi * f_gait * t);
        
        % Transient impact disturbance at t=2.0s (Phase 3/4.5 specification)
        if t >= 2.0 && t <= 2.3
            tau_dist = 0.10;
        else
            tau_dist = 0.00;
        end
        
        % SISO PI Controller (Frozen baseline Kp=0.50, Ki=0.20, u_sat=0.40)
        e = 0.0 - theta;
        int_cand = integrator + e * dt;
        u_unsat = params.J_pitch * 0.0 + 0.50 * e + 0.20 * int_cand;
        u_pitch = max(-0.40, min(0.40, u_unsat));
        if abs(u_unsat) <= 0.40
            integrator = int_cand;
        end
        
        % Plant integration (RK4)
        tau_net = u_pitch + tau_dist + tau_ripple;
        f_dyn = @(th, th_d) [th_d; (tau_net - params.B_pitch * th_d - params.K_pitch * th) / params.J_pitch];
        k1 = f_dyn(theta, theta_dot);
        k2 = f_dyn(theta + 0.5*dt*k1(1), theta_dot + 0.5*dt*k1(2));
        k3 = f_dyn(theta + 0.5*dt*k2(1), theta_dot + 0.5*dt*k2(2));
        k4 = f_dyn(theta + dt*k3(1), theta_dot + dt*k3(2));
        
        theta     = theta + (dt/6) * (k1(1) + 2*k2(1) + 2*k3(1) + k4(1));
        theta_dot = theta_dot + (dt/6) * (k1(2) + 2*k2(2) + 2*k3(2) + k4(2));
        
        theta_hist(k) = theta;
        u_pitch_hist(k) = u_pitch;
    end
    
    pitch_rmse = sqrt(mean(theta_hist.^2));
    pitch_peak = max(abs(theta_hist));
    
    % Evaluate violations
    viol_omega = dyn.omega_max > params.actuator.omega_max;
    viol_tau   = dyn.tau_max > params.actuator.tau_stall;
    viol_stab  = (pitch_peak * 180 / pi) > 6.0; % Pitch deviation > 6 deg considered unstable
    
    is_feasible = ~(viol_omega || viol_tau || viol_stab);
    
    status_str = 'PASS (FEASIBLE)';
    if ~is_feasible
        reasons = {};
        if viol_omega, reasons{end+1} = 'Omega_Limit'; end
        if viol_tau, reasons{end+1} = 'Torque_Stall'; end
        if viol_stab, reasons{end+1} = 'Pitch_Instability'; end
        status_str = ['FAIL (', strjoin(reasons, ', '), ')'];
    end
    
    fprintf('   C%d       |   %.3f     |   %.3f    |   %.2f    |      %.2f         |    %.2f       |     %.2f deg       | %s\n', ...
        c, v_speed, L_stride, f_gait, dyn.omega_max, dyn.tau_max, pitch_rmse*180/pi, status_str);
    
    results_table(c, :) = { ...
        sprintf('Candidate_%d', c), v_speed, L_stride, f_gait, h_step, ...
        dyn.omega_max, dyn.tau_max, dyn.P_mech_avg, dyn.I_bus_avg, ...
        pitch_rmse * (180/pi), is_feasible ...
    };

    cand_entry.dyn = dyn;
    cand_entry.theta_hist = theta_hist;
    cand_entry.u_pitch_hist = u_pitch_hist;
    cand_entry.is_feasible = is_feasible;
    study_data{c} = cand_entry;
end

fprintf('----------------------------------------------------------------------------------------\n\n');

col_names = {'Candidate', 'Speed_mps', 'Stride_m', 'Freq_Hz', 'Height_m', ...
             'Max_Omega_rad_s', 'Max_Torque_Nm', 'Avg_Mech_Power_W', 'Avg_Bus_Current_A', ...
             'Pitch_RMSE_deg', 'Is_Feasible'};
T_summary = cell2table(results_table, 'VariableNames', col_names);

% Export CSV table
output_dir = fullfile(root_dir, 'Gait_Optimization', 'results');
if ~exist(output_dir, 'dir'), mkdir(output_dir); end
csv_path = fullfile(output_dir, 'speed_safety_study_results.csv');
writetable(T_summary, csv_path);
fprintf('Exported Speed-Safety Study Table to: %s\n', csv_path);

study_results.table = T_summary;
study_results.data  = study_data;
study_results.params = params;
end
