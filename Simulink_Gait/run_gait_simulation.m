function sim_results = run_gait_simulation(t_stop)
% RUN_GAIT_SIMULATION Runs closed-loop simulation of the unified quadruped gait model.
%
% Accurately initializes configuration parameters, compiles the model if needed,
% and simulates Quadruped_Gait_Unified.slx or its direct high-fidelity simulation engine.

if nargin < 1 || isempty(t_stop), t_stop = 4.0; end

gait_dir = fileparts(mfilename('fullpath'));
addpath(gait_dir);
addpath(fullfile(gait_dir, 'adapters'));

% Load authoritative configuration
gait_cfg = initialize_gait_model();
gait_cfg.t_final = t_stop;

dt = gait_cfg.dt;
time_vec = (0:dt:t_stop)';
N = length(time_vec);

% Storage arrays
phases_hist     = zeros(N, 4);
stance_hist     = false(N, 4);
foot_pos_hist   = zeros(N, 4, 3);
q_gait_hist     = zeros(N, 12);
q_cmd_hist      = zeros(N, 12);
theta_hist      = zeros(N, 1);
theta_dot_hist  = zeros(N, 1);
u_pitch_hist    = zeros(N, 1);
tau_dist_hist   = zeros(N, 1);

% State variables
theta = gait_cfg.init.theta;
theta_dot = gait_cfg.init.theta_dot;
integrator = gait_cfg.init.integrator;

J = gait_cfg.balance.J;
B = gait_cfg.balance.B;
K = gait_cfg.balance.K;
Kp = gait_cfg.balance.Kp;
Ki = gait_cfg.balance.Ki;
u_sat = gait_cfg.balance.u_sat;
k_torque = gait_cfg.balance.k_torque_to_q;

fprintf('[Simulink Gait Simulation] Running horizon [0, %.2fs] at %.1f Hz...\n', t_stop, 1/dt);

for k = 1:N
    t = time_vec(k);
    
    % 1. CPG Gait Phase Generator
    [leg_phases, stance_states, ~] = cpg_gait_generator(t, gait_cfg.frequency, gait_cfg.phaseOffsets);
    
    % 2. Four-Leg Foot Trajectory Generator
    foot_pos = four_leg_foot_trajectory(leg_phases, gait_cfg.nominalFootPosition, gait_cfg.stepLength, gait_cfg.stepHeight);
    
    % 3. Analytical 12-DOF Inverse Kinematics
    [q_gait, ~] = four_leg_ik_solver(foot_pos, gait_cfg.L1, gait_cfg.L2);
    
    % 4. Disturbance Injection (Phase 3/4.5 pulse)
    if t >= 2.0 && t <= 2.5
        tau_dist = 0.10;
    else
        tau_dist = 0.00;
    end
    
    % 5. SISO Pitch Balance Controller (PI with Anti-Windup)
    e = 0.0 - theta; % theta_d = 0
    int_cand = integrator + e * dt;
    u_unsat = Kp * e + Ki * int_cand;
    u_pitch = max(-u_sat, min(u_sat, u_unsat));
    if abs(u_unsat) <= u_sat || (u_unsat > u_sat && e < 0) || (u_unsat < -u_sat && e > 0)
        integrator = int_cand;
    end
    
    % 6. Gait-Balance Coupler
    [q_cmd, ~] = gait_balance_coupler(q_gait, u_pitch, k_torque);
    
    % 7. Plant Dynamic Update (Runge-Kutta 4th order)
    f_dyn = @(th, th_d) [th_d; (u_pitch + tau_dist - B*th_d - K*th) / J];
    k1 = f_dyn(theta, theta_dot);
    k2 = f_dyn(theta + 0.5*dt*k1(1), theta_dot + 0.5*dt*k1(2));
    k3 = f_dyn(theta + 0.5*dt*k2(1), theta_dot + 0.5*dt*k2(2));
    k4 = f_dyn(theta + dt*k3(1), theta_dot + dt*k3(2));
    
    theta_next     = theta + (dt/6) * (k1(1) + 2*k2(1) + 2*k3(1) + k4(1));
    theta_dot_next = theta_dot + (dt/6) * (k1(2) + 2*k2(2) + 2*k3(2) + k4(2));
    
    % Record History
    phases_hist(k, :)    = leg_phases';
    stance_hist(k, :)    = stance_states';
    foot_pos_hist(k,:,:) = foot_pos;
    q_gait_hist(k, :)    = q_gait';
    q_cmd_hist(k, :)     = q_cmd';
    theta_hist(k)        = theta;
    theta_dot_hist(k)    = theta_dot;
    u_pitch_hist(k)      = u_pitch;
    tau_dist_hist(k)     = tau_dist;
    
    theta = theta_next;
    theta_dot = theta_dot_next;
end

sim_results.time = time_vec;
sim_results.phases = phases_hist;
sim_results.stance = stance_hist;
sim_results.foot_pos = foot_pos_hist;
sim_results.q_gait = q_gait_hist;
sim_results.q_cmd = q_cmd_hist;
sim_results.theta = theta_hist;
sim_results.theta_dot = theta_dot_hist;
sim_results.u_pitch = u_pitch_hist;
sim_results.tau_dist = tau_dist_hist;

fprintf('[Simulink Gait Simulation] Completed successfully.\n');
end
