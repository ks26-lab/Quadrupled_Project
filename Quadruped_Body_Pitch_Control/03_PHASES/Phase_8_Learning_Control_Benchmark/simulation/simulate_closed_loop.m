function out = simulate_closed_loop(ctrl_type, ctrl_obj, test_scenario, plant_params, sim_cfg)
% SIMULATE_CLOSED_LOOP Universal Runge-Kutta 4th Order SISO Simulation Engine.
%
% Accurately integrates continuous pitch plant:
%   J * theta_ddot + B * theta_dot + K * theta = u_sat(t) + tau_dist(t)
%
% Arguments:
%   ctrl_type     : 'PI', 'LSTM', or 'TD3'
%   ctrl_obj      : controller state struct
%   test_scenario : struct with fields:
%                     .time     : time vector [s]
%                     .theta_d  : desired pitch trajectory [rad]
%                     .tau_dist : disturbance torque profile [N.m]
%                     .theta_0  : initial pitch [rad] (optional, default 0)
%                     .omega_0  : initial pitch velocity [rad/s] (optional, default 0)
%   plant_params  : struct with fields .J, .B, .K (defaults to robot_parameters)
%   sim_cfg       : struct with sensor noise, rate limits, etc. (optional)

if nargin < 4 || isempty(plant_params)
    plant_params = struct('J', 0.05, 'B', 0.10, 'K', 0.00);
end
if nargin < 5 || isempty(sim_cfg)
    sim_cfg = struct();
end

time = test_scenario.time;
N = length(time);
dt = time(2) - time(1);

J = plant_params.J;
B = plant_params.B;
K = plant_params.K;

% Preallocate histories
theta_hist     = zeros(N, 1);
theta_dot_hist = zeros(N, 1);
u_hist         = zeros(N, 1);
e_hist         = zeros(N, 1);
step_times     = zeros(N, 1);

% Initial states
if isfield(test_scenario, 'theta_0'), theta = test_scenario.theta_0; else, theta = 0.0; end
if isfield(test_scenario, 'omega_0'), theta_dot = test_scenario.omega_0; else, theta_dot = 0.0; end

ctrl = ctrl_obj;

% Optional sensor noise
if isfield(sim_cfg, 'noise_sigma') && sim_cfg.noise_sigma > 0
    noise = sim_cfg.noise_sigma * randn(N, 1);
else
    noise = zeros(N, 1);
end

% Rate limit setup
has_rate_limit = isfield(sim_cfg, 'R_max') && sim_cfg.R_max > 0;
if has_rate_limit
    R_max = sim_cfg.R_max;
    u_prev = 0.0;
end

for k = 1:N
    t = time(k);
    theta_d = test_scenario.theta_d(k);
    tau_dist = test_scenario.tau_dist(k);
    
    % IMU measurement
    theta_m = theta + noise(k);
    
    % Controller inference with timing
    tic;
    switch upper(ctrl_type)
        case 'PI'
            [u_cmd, ctrl] = pi_step(ctrl, theta_d, theta_m);
        case 'LSTM'
            [u_cmd, ctrl] = lstm_step(ctrl, theta_d, theta_m);
        case 'TD3'
            [u_cmd, ctrl] = td3_step(ctrl, theta_d, theta_m);
        otherwise
            error('Unknown controller type: %s', ctrl_type);
    end
    t_step = toc;
    step_times(k) = t_step;
    
    % Apply slew rate limit if configured
    if has_rate_limit
        du = (u_cmd - u_prev);
        du_clamped = max(-R_max * dt, min(R_max * dt, du));
        u_actual = u_prev + du_clamped;
        u_prev = u_actual;
    else
        u_actual = u_cmd;
    end
    
    % Record state at start of interval
    theta_hist(k)     = theta;
    theta_dot_hist(k) = theta_dot;
    u_hist(k)         = u_actual;
    e_hist(k)         = theta_d - theta;
    
    % Advance continuous state using Runge-Kutta 4th Order
    % State vector: x = [theta; theta_dot]
    % dx/dt = [theta_dot; (u_actual + tau_dist - B*theta_dot - K*theta)/J]
    if k < N
        f_dyn = @(th, th_d) [th_d; (u_actual + tau_dist - B*th_d - K*th) / J];
        
        k1 = f_dyn(theta, theta_dot);
        k2 = f_dyn(theta + 0.5*dt*k1(1), theta_dot + 0.5*dt*k1(2));
        k3 = f_dyn(theta + 0.5*dt*k2(1), theta_dot + 0.5*dt*k2(2));
        k4 = f_dyn(theta + dt*k3(1), theta_dot + dt*k3(2));
        
        theta     = theta + (dt/6) * (k1(1) + 2*k2(1) + 2*k3(1) + k4(1));
        theta_dot = theta_dot + (dt/6) * (k1(2) + 2*k2(2) + 2*k3(2) + k4(2));
    end
end

% Package results
out.time          = time;
out.theta_d       = test_scenario.theta_d;
out.theta         = theta_hist;
out.theta_dot     = theta_dot_hist;
out.u             = u_hist;
out.error         = e_hist;
out.tau_dist      = test_scenario.tau_dist;
out.step_times_us = step_times * 1e6; % microseconds
out.mean_step_us  = mean(step_times(2:end)) * 1e6; % exclude 1st step warm-up
end
