function test_suite = generate_test_scenarios()
% GENERATE_TEST_SCENARIOS Generates 4 rigorous benchmark test scenarios.
%
% Strictly separated from training conditions:
%   Test 1: Multi-Step & Dynamic Trajectory Tracking (no external disturbance)
%   Test 2: Sustained External Torque Disturbance Rejection (at zero pitch command)
%   Test 3: Ground Contact Loss / Asymmetric Leg Slip Fault (transient high-torque impact)
%   Test 4: Plant Uncertainty & Parameter Mismatch (evaluates tracking under changed J and B)

dt = 0.005; % 200 Hz
t_final = 8.0;
time = (0:dt:t_final)';
N = length(time);

%% Test 1: Dynamic Reference Tracking
% Reference: 0 rad -> +5 deg (+0.0872 rad) at 1s -> -3 deg (-0.0523 rad) at 3s -> 0.5 Hz sine from 5s
th_d1 = zeros(N, 1);
for k = 1:N
    t = time(k);
    if t < 1.0
        th_d1(k) = 0.0;
    elseif t < 3.0
        th_d1(k) = 5.0 * (pi / 180); % +5 degrees
    elseif t < 5.0
        th_d1(k) = -3.0 * (pi / 180); % -3 degrees
    else
        th_d1(k) = 4.0 * (pi / 180) * sin(2 * pi * 0.5 * (t - 5.0));
    end
end
tau_dist1 = zeros(N, 1);

test_suite.test1 = struct( ...
    'name', 'Test 1: Dynamic Trajectory Tracking', ...
    'time', time, ...
    'theta_d', th_d1, ...
    'tau_dist', tau_dist1, ...
    'plant_params', struct('J', 0.05, 'B', 0.10, 'K', 0.00), ...
    'theta_0', 0.0, ...
    'omega_0', 0.0 ...
);

%% Test 2: External Torque Disturbance Rejection
% Reference: desired pitch = 0.0 rad (stabilization)
% Disturbance: Sustained step torque Td = +0.15 N.m applied from t = 2.0 s to 4.5 s
th_d2 = zeros(N, 1);
tau_dist2 = zeros(N, 1);
idx_dist2 = find(time >= 2.0 & time <= 4.5);
tau_dist2(idx_dist2) = 0.15; % +0.15 N.m sustained pitch disturbance

test_suite.test2 = struct( ...
    'name', 'Test 2: Sustained Torque Disturbance Rejection', ...
    'time', time, ...
    'theta_d', th_d2, ...
    'tau_dist', tau_dist2, ...
    'plant_params', struct('J', 0.05, 'B', 0.10, 'K', 0.00), ...
    'theta_0', 0.0, ...
    'omega_0', 0.0 ...
);

%% Test 3: Contact-Loss / Asymmetric Leg Slip Fault
% Reference: desired pitch = 0.0 rad
% Contact loss fault: sudden severe disturbance pulse Td = -0.25 N.m from 2.0 s to 3.5 s (1.5 s duration)
% matches Phase 4.6 contact-loss torque scale
th_d3 = zeros(N, 1);
tau_dist3 = zeros(N, 1);
idx_dist3 = find(time >= 2.0 & time <= 3.5);
tau_dist3(idx_dist3) = -0.25;

test_suite.test3 = struct( ...
    'name', 'Test 3: Contact Loss / Foot Slip Fault', ...
    'time', time, ...
    'theta_d', th_d3, ...
    'tau_dist', tau_dist3, ...
    'plant_params', struct('J', 0.05, 'B', 0.07, 'K', 0.00), ... % 30% reduction in ground damping
    'theta_0', 0.0, ...
    'omega_0', 0.0 ...
);

%% Test 4: Plant Uncertainty & Parameter Mismatch
% Reference: +4 deg step at 1.0 s -> -4 deg at 4.0 s
% Plant: J increased by +35% (J = 0.0675), B decreased by -30% (B = 0.070)
% Disturbance: mild random terrain ripple (0.02 N.m)
th_d4 = zeros(N, 1);
idx_step4a = find(time >= 1.0 & time < 4.0);
idx_step4b = find(time >= 4.0);
th_d4(idx_step4a) = 4.0 * (pi / 180);
th_d4(idx_step4b) = -4.0 * (pi / 180);

rng(101);
tau_dist4 = 0.02 * sin(2 * pi * 1.5 * time);

test_suite.test4 = struct( ...
    'name', 'Test 4: Plant Uncertainty & Robustness', ...
    'time', time, ...
    'theta_d', th_d4, ...
    'tau_dist', tau_dist4, ...
    'plant_params', struct('J', 0.0675, 'B', 0.070, 'K', 0.00), ... % +35% J, -30% B
    'theta_0', 0.0, ...
    'omega_0', 0.0 ...
);

end
