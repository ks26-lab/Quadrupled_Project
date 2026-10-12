function dataset = generate_training_data(N_trajectories, dt, t_duration)
% GENERATE_TRAINING_DATA Generates rich training database for supervised LSTM learning.
%
% Generates varied random reference trajectories, random pulse disturbances,
% and records states under high-fidelity state feedback with anti-windup.
%
% Strictly disjoint from the specific test sequences in generate_test_scenarios.m.

if nargin < 1, N_trajectories = 25; end
if nargin < 2, dt = 0.005; end
if nargin < 3, t_duration = 6.0; end

time = (0:dt:t_duration)';
N_steps = length(time);

J_nom = 0.05;
B_nom = 0.10;
K_nom = 0.00;
u_sat = 0.40;

% Expert baseline parameters
Kp_expert = 0.50;
Ki_expert = 0.20;

rng(42); % Fixed training generation seed for strict reproducibility

dataset.X = cell(N_trajectories, 1);
dataset.Y = cell(N_trajectories, 1);

total_samples = N_trajectories * N_steps;
X_all = zeros(total_samples, 3);
Y_all = zeros(total_samples, 1);
sample_counter = 0;

for traj_idx = 1:N_trajectories
    % Random plant slight perturbation in training
    J_i = J_nom * (0.9 + 0.2 * rand());
    B_i = B_nom * (0.9 + 0.2 * rand());
    
    % Random reference profile: series of random steps and ramps
    th_d = zeros(N_steps, 1);
    num_steps = randi([2, 4]);
    step_times = sort(rand(num_steps, 1) * (t_duration - 1.0));
    curr_ref = 0.0;
    
    for k = 1:N_steps
        t = time(k);
        for s = 1:num_steps
            if abs(t - step_times(s)) < dt/2
                curr_ref = (rand() * 0.20 - 0.10); % +/- 0.10 rad (+/- 5.7 deg)
            end
        end
        th_d(k) = curr_ref;
    end
    
    % Random transient disturbance pulse
    tau_dist = zeros(N_steps, 1);
    if rand() > 0.3
        t_dist_start = 1.0 + rand() * (t_duration - 2.5);
        t_dist_len   = 0.3 + rand() * 0.8;
        dist_mag     = (rand() * 0.20 - 0.10); % +/- 0.10 N.m
        idx_dist = find(time >= t_dist_start & time <= (t_dist_start + t_dist_len));
        tau_dist(idx_dist) = dist_mag;
    end
    
    % Simulate expert closed-loop trajectory
    theta = 0.0;
    theta_dot = 0.0;
    integral_e = 0.0;
    prev_e = 0.0;
    
    X_traj = zeros(N_steps, 3); % [e, e_dot, theta_m]
    Y_traj = zeros(N_steps, 1); % u
    
    for k = 1:N_steps
        e = th_d(k) - theta;
        e_dot = (e - prev_e) / dt;
        prev_e = e;
        
        % Expert PI with anti-windup clamping
        int_cand = integral_e + e * dt;
        u_unsat = Kp_expert * e + Ki_expert * int_cand;
        u = max(-u_sat, min(u_sat, u_unsat));
        
        if abs(u_unsat) <= u_sat || (u_unsat > u_sat && e < 0) || (u_unsat < -u_sat && e > 0)
            integral_e = int_cand;
        end
        
        % Record training feature vector and target
        X_traj(k, :) = [e, e_dot, theta];
        Y_traj(k, 1) = u;
        
        sample_counter = sample_counter + 1;
        X_all(sample_counter, :) = [e, e_dot, theta];
        Y_all(sample_counter, 1) = u;
        
        % Continuous plant integration
        theta_ddot = (u + tau_dist(k) - B_i * theta_dot - K_nom * theta) / J_i;
        theta_dot = theta_dot + theta_ddot * dt;
        theta = theta + theta_dot * dt;
    end
    
    dataset.X{traj_idx} = X_traj;
    dataset.Y{traj_idx} = Y_traj;
end

% Compute feature normalizations
dataset.mu_x    = mean(X_all, 1)';
dataset.sigma_x = std(X_all, 0, 1)';
dataset.mu_u    = mean(Y_all);
dataset.sigma_u = std(Y_all);
dataset.dt      = dt;
dataset.N_trajectories = N_trajectories;
dataset.total_samples  = total_samples;

fprintf('Generated %d training trajectories (%d total timesteps) for learning.\n', ...
    N_trajectories, total_samples);
end
