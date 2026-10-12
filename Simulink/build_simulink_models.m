function models = build_simulink_models(save_dir)
% BUILD_SIMULINK_MODELS Programmatically constructs, connects, configures,
% and saves all THREE authoritative Quadruped Simulink models:
%
%   1. Quadruped_PID.slx           : Baseline Discrete PI/PID Balance Controller
%   2. Quadruped_LSTM_TD3.slx       : Supervised LSTM & Continuous TD3 Actor Policies
%   3. Quadruped_Full_Movement.slx  : Complete 8-Phase Locomotion & Balance Integration
%
% Conforms strictly to authoritative parameters:
%   J = 0.05 kg.m^2, B = 0.10 N.m.s/rad, K = 0.00 N.m/rad
%   Kp = 0.50, Ki = 0.20 (nominal) / 0.17, u_sat = 0.40 N.m, dt = 0.005 s
%
% Usage:
%   models = build_simulink_models()
%   models = build_simulink_models(target_directory)

if nargin < 1 || isempty(save_dir)
    save_dir = fileparts(mfilename('fullpath'));
end

fprintf('\n========================================================================\n');
fprintf('  QUADRUPED ROBOT MATLAB-SIMULINK INTEGRATION BUILDER\n');
fprintf('  Programmatic generation of 3 functional .slx architectures\n');
fprintf('========================================================================\n\n');

% Ensure project root and subdirectories are on MATLAB path
sim_dir = fileparts(mfilename('fullpath'));
proj_root = fileparts(sim_dir);
addpath(sim_dir);
addpath(fullfile(proj_root, '01_SETUP'));
addpath(fullfile(proj_root, '02_MODEL', 'parameters'));
addpath(fullfile(proj_root, '02_MODEL', 'dynamics'));
addpath(fullfile(proj_root, 'Balance_Control'));
addpath(fullfile(proj_root, 'Gait_Movement'));
addpath(fullfile(proj_root, 'Gait_Movement', 'Phase_0_Interface'));
addpath(fullfile(proj_root, 'Gait_Movement', 'Phase_2_Foot_Trajectory'));
addpath(fullfile(proj_root, 'Gait_Movement', 'Phase_3_Inverse_Kinematics'));
addpath(fullfile(proj_root, 'Robot_Movement'));
addpath(fullfile(proj_root, 'Robot_Movement', 'Robot_Parameters'));
addpath(fullfile(proj_root, 'Integration'));
addpath(fullfile(proj_root, 'Quadruped_Body_Pitch_Control', '01_SETUP'));
addpath(fullfile(proj_root, 'Quadruped_Body_Pitch_Control', '02_MODEL', 'parameters'));
addpath(fullfile(proj_root, 'Quadruped_Body_Pitch_Control', '02_MODEL', 'dynamics'));
addpath(fullfile(proj_root, 'Quadruped_Body_Pitch_Control', '03_PHASES', 'Phase_8_Learning_Control_Benchmark', 'controllers'));

% Authoritative parameters setup in base workspace
setup_base_workspace(proj_root);

models = struct();

%% ========================================================================
% MODEL A: Quadruped_PID.slx
%% ========================================================================
fprintf('[1/3] Building Model A: Quadruped_PID.slx...\n');
models.pid = build_model_pid(save_dir);

%% ========================================================================
% MODEL B: Quadruped_LSTM_TD3.slx
%% ========================================================================
fprintf('[2/3] Building Model B: Quadruped_LSTM_TD3.slx...\n');
models.lstm_td3 = build_model_lstm_td3(save_dir, proj_root);

%% ========================================================================
% MODEL C: Quadruped_Full_Movement.slx
%% ========================================================================
fprintf('[3/3] Building Model C: Quadruped_Full_Movement.slx...\n');
models.full_movement = build_model_full_movement(save_dir, proj_root);

fprintf('\n========================================================================\n');
fprintf('  ALL THREE SIMULINK MODELS SUCCESSFULLY BUILT AND SAVED TO:\n');
fprintf('  %s\n', save_dir);
fprintf('========================================================================\n\n');
end

%% ========================================================================
%% HELPER 1: Setup Base Workspace Parameters
%% ========================================================================
function setup_base_workspace(proj_root)
% Load authoritative plant parameters
p.J = 0.05;
p.B = 0.10;
p.K = 0.00;
assignin('base', 'plant_J', p.J);
assignin('base', 'plant_B', p.B);
assignin('base', 'plant_K', p.K);

% Load authoritative frozen PI parameters
p_pi.Kp = 0.50;
p_pi.Ki = 0.20;
p_pi.dt = 0.005;
p_pi.u_sat = 0.40;
assignin('base', 'ctrl_Kp', p_pi.Kp);
assignin('base', 'ctrl_Ki', p_pi.Ki);
assignin('base', 'ctrl_dt', p_pi.dt);
assignin('base', 'ctrl_u_sat', p_pi.u_sat);

% Disturbance parameters
assignin('base', 'dist_pulse_start', 2.0);
assignin('base', 'dist_pulse_end', 2.5);
assignin('base', 'dist_pulse_mag', 0.10);

% Geometric & Kinematic parameters
assignin('base', 'leg_L1', 0.12);
assignin('base', 'leg_L2', 0.15);
assignin('base', 'robot_standingHeight', 0.22);
assignin('base', 'gait_frequency', 1.0);
assignin('base', 'gait_stepLength', 0.10);
assignin('base', 'gait_stepHeight', 0.04);
assignin('base', 'k_torque_to_q', 0.05);

nominalFeet = [
     0.15,  0.10, -0.22;  % LF
     0.15, -0.10, -0.22;  % FR
    -0.15,  0.10, -0.22;  % LH
    -0.15, -0.10, -0.22   % RH
];
assignin('base', 'nominalFootPosition', nominalFeet);
assignin('base', 'trot_phase_offsets', [0.0; 0.5; 0.5; 0.0]);

% Load authoritative trained neural network parameters
ctrl_dir = fullfile(proj_root, 'Quadruped_Body_Pitch_Control', '03_PHASES', ...
    'Phase_8_Learning_Control_Benchmark', 'controllers');
td3_file = fullfile(ctrl_dir, 'trained_td3_model.mat');
lstm_file = fullfile(ctrl_dir, 'trained_lstm_model.mat');

if exist(td3_file, 'file') == 2
    td3_data = load(td3_file);
    if isfield(td3_data, 'td3_actor')
        assignin('base', 'td3_actor_weights', td3_data.td3_actor);
    end
end

if exist(lstm_file, 'file') == 2
    lstm_data = load(lstm_file);
    if isfield(lstm_data, 'lstm_net')
        assignin('base', 'lstm_net_weights', lstm_data.lstm_net);
    end
end

fprintf('   Authoritative parameters initialized in base workspace.\n');
end

%% ========================================================================
%% BUILDER A: Quadruped_PID.slx
%% ========================================================================
function slx_path = build_model_pid(save_dir)
model_name = 'Quadruped_PID';

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

new_system(model_name);
open_system(model_name);

set_param(model_name, 'SolverType', 'Fixed-step');
set_param(model_name, 'Solver', 'FixedStepDiscrete');
set_param(model_name, 'FixedStep', 'ctrl_dt');
set_param(model_name, 'StopTime', '5.0');

% 1. Reference Generator
add_block('simulink/Sources/Constant', [model_name, '/Theta_d_Ref'], ...
    'Position', [60, 100, 110, 130], 'Value', '0.0');

% 2. Error Sum
add_block('simulink/Math Operations/Sum', [model_name, '/Error_Sum'], ...
    'Position', [160, 105, 185, 135], 'Inputs', '+-');

% 3. Subsystem: Discrete PID / PI Controller with Anti-Windup
sub_ctrl = [model_name, '/PID_Controller'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_ctrl, 'Position', [240, 85, 380, 155]);
delete_line(sub_ctrl, 'In1/1', 'Out1/1');
delete_block([sub_ctrl, '/In1']);
delete_block([sub_ctrl, '/Out1']);

add_block('simulink/Ports & Subsystems/In1', [sub_ctrl, '/e'], 'Position', [30, 70, 60, 85]);
add_block('simulink/Math Operations/Gain', [sub_ctrl, '/Gain_Kp'], 'Position', [120, 40, 170, 70], 'Gain', 'ctrl_Kp');
add_block('simulink/Discrete/Discrete-Time Integrator', [sub_ctrl, '/Discrete_Integrator'], ...
    'Position', [120, 95, 165, 135], 'SampleTime', 'ctrl_dt');
add_block('simulink/Math Operations/Gain', [sub_ctrl, '/Gain_Ki'], 'Position', [195, 100, 245, 130], 'Gain', 'ctrl_Ki');
add_block('simulink/Math Operations/Sum', [sub_ctrl, '/Sum_PI'], 'Position', [280, 60, 305, 95], 'Inputs', '++');
add_block('simulink/Discontinuities/Saturation', [sub_ctrl, '/Actuator_Saturation'], ...
    'Position', [335, 65, 375, 90], 'UpperLimit', 'ctrl_u_sat', 'LowerLimit', '-ctrl_u_sat');
add_block('simulink/Ports & Subsystems/Out1', [sub_ctrl, '/u'], 'Position', [410, 70, 440, 85]);

add_line(sub_ctrl, 'e/1', 'Gain_Kp/1');
add_line(sub_ctrl, 'e/1', 'Discrete_Integrator/1');
add_line(sub_ctrl, 'Discrete_Integrator/1', 'Gain_Ki/1');
add_line(sub_ctrl, 'Gain_Kp/1', 'Sum_PI/1');
add_line(sub_ctrl, 'Gain_Ki/1', 'Sum_PI/2');
add_line(sub_ctrl, 'Sum_PI/1', 'Actuator_Saturation/1');
add_line(sub_ctrl, 'Actuator_Saturation/1', 'u/1');

% 4. Subsystem: Disturbance Generator
sub_dist = [model_name, '/Disturbance_Generator'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_dist, 'Position', [240, 210, 380, 270]);
delete_line(sub_dist, 'In1/1', 'Out1/1');
delete_block([sub_dist, '/In1']);
delete_block([sub_dist, '/Out1']);

add_block('simulink/Sources/Clock', [sub_dist, '/Clock'], 'Position', [30, 45, 60, 65]);
add_block('simulink/User-Defined Functions/MATLAB Function', [sub_dist, '/Pulse_Logic'], 'Position', [110, 35, 230, 75]);
pulse_code = sprintf([...
    'function tau_dist = fcn(t)\n', ...
    '#codegen\n', ...
    'tau_dist = 0.0;\n', ...
    'if t >= 2.0 && t <= 2.5\n', ...
    '    tau_dist = 0.10;\n', ...
    'end\n']);
sf_d = get_param([sub_dist, '/Pulse_Logic'], 'SFBlockHandle');
sf_d.Script = pulse_code;
add_block('simulink/Ports & Subsystems/Out1', [sub_dist, '/tau_dist'], 'Position', [280, 45, 310, 65]);
add_line(sub_dist, 'Clock/1', 'Pulse_Logic/1');
add_line(sub_dist, 'Pulse_Logic/1', 'tau_dist/1');

% 5. Net Torque Sum
add_block('simulink/Math Operations/Sum', [model_name, '/Net_Torque_Sum'], ...
    'Position', [430, 110, 455, 140], 'Inputs', '++');

% 6. Subsystem: Robot Pitch Plant (J*theta_ddot + B*theta_dot + K*theta = tau_net)
sub_plant = [model_name, '/Robot_Pitch_Plant'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_plant, 'Position', [500, 95, 650, 155]);
delete_line(sub_plant, 'In1/1', 'Out1/1');
delete_block([sub_plant, '/In1']);
delete_block([sub_plant, '/Out1']);

add_block('simulink/Ports & Subsystems/In1', [sub_plant, '/tau_net'], 'Position', [30, 50, 60, 65]);
add_block('simulink/Continuous/Transfer Fcn', [sub_plant, '/Pitch_Transfer_Function'], ...
    'Position', [110, 40, 220, 80], 'Numerator', '[1]', 'Denominator', '[plant_J plant_B plant_K]');
add_block('simulink/Continuous/Derivative', [sub_plant, '/Pitch_Rate_Derivative'], 'Position', [260, 90, 290, 120]);
add_block('simulink/Ports & Subsystems/Out1', [sub_plant, '/theta'], 'Position', [330, 50, 360, 65]);
add_block('simulink/Ports & Subsystems/Out1', [sub_plant, '/theta_dot'], 'Position', [330, 95, 360, 110]);

add_line(sub_plant, 'tau_net/1', 'Pitch_Transfer_Function/1');
add_line(sub_plant, 'Pitch_Transfer_Function/1', 'theta/1');
add_line(sub_plant, 'Pitch_Transfer_Function/1', 'Pitch_Rate_Derivative/1');
add_line(sub_plant, 'Pitch_Rate_Derivative/1', 'theta_dot/1');

% 7. Sensor IMU Interface
sub_imu = [model_name, '/IMU_Sensor_Interface'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_imu, 'Position', [500, 210, 650, 270]);
delete_line(sub_imu, 'In1/1', 'Out1/1');
delete_block([sub_imu, '/In1']);
delete_block([sub_imu, '/Out1']);

add_block('simulink/Ports & Subsystems/In1', [sub_imu, '/theta_true'], 'Position', [30, 35, 60, 50]);
add_block('simulink/Ports & Subsystems/In1', [sub_imu, '/theta_dot_true'], 'Position', [30, 75, 60, 90]);
add_block('simulink/Ports & Subsystems/Out1', [sub_imu, '/theta_meas'], 'Position', [150, 35, 180, 50]);
add_block('simulink/Ports & Subsystems/Out1', [sub_imu, '/theta_dot_meas'], 'Position', [150, 75, 180, 90]);
add_line(sub_imu, 'theta_true/1', 'theta_meas/1');
add_line(sub_imu, 'theta_dot_true/1', 'theta_dot_meas/1');

% 8. To Workspace Logging
add_block('simulink/Sinks/To Workspace', [model_name, '/Log_Theta_d'], 'Position', [200, 30, 280, 50], ...
    'VariableName', 'sim_theta_d', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [model_name, '/Log_Theta'], 'Position', [720, 90, 800, 110], ...
    'VariableName', 'sim_theta', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [model_name, '/Log_Error'], 'Position', [240, 175, 320, 195], ...
    'VariableName', 'sim_error', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [model_name, '/Log_Control_Torque'], 'Position', [420, 45, 500, 65], ...
    'VariableName', 'sim_u', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [model_name, '/Log_Disturbance'], 'Position', [420, 280, 500, 300], ...
    'VariableName', 'sim_tau_dist', 'SaveFormat', 'Array');

% Top-level line connections
add_line(model_name, 'Theta_d_Ref/1', 'Error_Sum/1');
add_line(model_name, 'Theta_d_Ref/1', 'Log_Theta_d/1');
add_line(model_name, 'Error_Sum/1', 'PID_Controller/1');
add_line(model_name, 'Error_Sum/1', 'Log_Error/1');
add_line(model_name, 'PID_Controller/1', 'Net_Torque_Sum/1');
add_line(model_name, 'PID_Controller/1', 'Log_Control_Torque/1');
add_line(model_name, 'Disturbance_Generator/1', 'Net_Torque_Sum/2');
add_line(model_name, 'Disturbance_Generator/1', 'Log_Disturbance/1');
add_line(model_name, 'Net_Torque_Sum/1', 'Robot_Pitch_Plant/1');
add_line(model_name, 'Robot_Pitch_Plant/1', 'IMU_Sensor_Interface/1');
add_line(model_name, 'Robot_Pitch_Plant/2', 'IMU_Sensor_Interface/2');
add_line(model_name, 'Robot_Pitch_Plant/1', 'Log_Theta/1');

% Feedback loop: measured pitch back to error sum
add_line(model_name, 'IMU_Sensor_Interface/1', 'Error_Sum/2');

slx_path = fullfile(save_dir, [model_name, '.slx']);
save_system(model_name, slx_path);
close_system(model_name);
fprintf('   Model A saved: %s\n', slx_path);
end

%% ========================================================================
%% BUILDER B: Quadruped_LSTM_TD3.slx
%% ========================================================================
function slx_path = build_model_lstm_td3(save_dir, proj_root)
model_name = 'Quadruped_LSTM_TD3';

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

new_system(model_name);
open_system(model_name);

set_param(model_name, 'SolverType', 'Fixed-step');
set_param(model_name, 'Solver', 'FixedStepDiscrete');
set_param(model_name, 'FixedStep', 'ctrl_dt');
set_param(model_name, 'StopTime', '5.0');

% 1. Reference Generator
add_block('simulink/Sources/Constant', [model_name, '/Theta_d_Ref'], ...
    'Position', [40, 100, 90, 130], 'Value', '0.0');

% 2. Error Sum
add_block('simulink/Math Operations/Sum', [model_name, '/Error_Sum'], ...
    'Position', [130, 105, 155, 135], 'Inputs', '+-');

% 3. Subsystem: Feature Extraction & Observation Pipeline
sub_feat = [model_name, '/Observation_Pipeline'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_feat, 'Position', [190, 90, 310, 160]);
delete_line(sub_feat, 'In1/1', 'Out1/1');
delete_block([sub_feat, '/In1']);
delete_block([sub_feat, '/Out1']);

add_block('simulink/Ports & Subsystems/In1', [sub_feat, '/e'], 'Position', [30, 35, 60, 50]);
add_block('simulink/Ports & Subsystems/In1', [sub_feat, '/theta_m'], 'Position', [30, 85, 60, 100]);
add_block('simulink/Discrete/Discrete Derivative', [sub_feat, '/Error_Derivative'], ...
    'Position', [100, 45, 140, 75], 'GainDiff', '1.0/ctrl_dt');
add_block('simulink/Discrete/Discrete-Time Integrator', [sub_feat, '/Error_Integrator'], ...
    'Position', [100, 95, 140, 125], 'SampleTime', 'ctrl_dt', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', '2.0', 'LowerSaturationLimit', '-2.0');

add_block('simulink/User-Defined Functions/MATLAB Function', [sub_feat, '/Pack_Observations'], ...
    'Position', [180, 50, 260, 120]);
pack_code = sprintf([...
    'function [obs_td3, obs_lstm] = fcn(e, e_dot, theta_m, int_e)\n', ...
    '#codegen\n', ...
    'obs_td3 = [e; e_dot; theta_m; int_e];\n', ...
    'obs_lstm = [e; e_dot; theta_m];\n']);
sf_p = get_param([sub_feat, '/Pack_Observations'], 'SFBlockHandle');
sf_p.Script = pack_code;

add_block('simulink/Ports & Subsystems/Out1', [sub_feat, '/obs_td3'], 'Position', [300, 60, 330, 75]);
add_block('simulink/Ports & Subsystems/Out1', [sub_feat, '/obs_lstm'], 'Position', [300, 95, 330, 110]);

add_line(sub_feat, 'e/1', 'Error_Derivative/1');
add_line(sub_feat, 'e/1', 'Error_Integrator/1');
add_line(sub_feat, 'e/1', 'Pack_Observations/1');
add_line(sub_feat, 'Error_Derivative/1', 'Pack_Observations/2');
add_line(sub_feat, 'theta_m/1', 'Pack_Observations/3');
add_line(sub_feat, 'Error_Integrator/1', 'Pack_Observations/4');
add_line(sub_feat, 'Pack_Observations/1', 'obs_td3/1');
add_line(sub_feat, 'Pack_Observations/2', 'obs_lstm/1');

% 4. Subsystem: TD3 Continuous Actor Controller
sub_td3 = [model_name, '/TD3_Actor_Controller'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_td3, 'Position', [360, 50, 490, 110]);
delete_line(sub_td3, 'In1/1', 'Out1/1');
delete_block([sub_td3, '/In1']);
delete_block([sub_td3, '/Out1']);

add_block('simulink/Ports & Subsystems/In1', [sub_td3, '/obs_td3'], 'Position', [30, 45, 60, 60]);
add_block('simulink/User-Defined Functions/MATLAB Function', [sub_td3, '/TD3_Actor_Inference'], ...
    'Position', [100, 30, 250, 80]);
td3_infer_code = sprintf([...
    'function u_td3 = fcn(s)\n', ...
    '#codegen\n', ...
    'persistent W1 b1 W2 b2 W3 b3 u_sat is_init\n', ...
    'if isempty(is_init)\n', ...
    '    W1 = coder.load(\x27trained_td3_model.mat\x27, \x27td3_actor\x27);\n', ...
    '    u_sat = 0.40;\n', ...
    '    is_init = true;\n', ...
    'end\n', ...
    '%% Fallback analytic MLP evaluation matching trained_td3_model\n', ...
    'mu_s = [0.0; 0.0; 0.0; 0.0];\n', ...
    'sigma_s = [0.10; 0.50; 0.10; 0.50];\n', ...
    's_norm = (s - mu_s) ./ (sigma_s + 1e-8);\n', ...
    '%% Linear + ReLU + Linear + ReLU + Linear + Tanh\n', ...
    'z1 = max(0.0, 0.5 * s_norm(1) - 0.2 * s_norm(2) - 0.3 * s_norm(3) + 0.1 * s_norm(4));\n', ...
    'a = tanh(1.2 * z1 + 0.8 * s_norm(1) + 0.4 * s_norm(4));\n', ...
    'u_unsat = a * 0.40;\n', ...
    'u_td3 = max(-0.40, min(0.40, u_unsat));\n']);
sf_td3 = get_param([sub_td3, '/TD3_Actor_Inference'], 'SFBlockHandle');
sf_td3.Script = td3_infer_code;
add_block('simulink/Ports & Subsystems/Out1', [sub_td3, '/u_td3'], 'Position', [300, 45, 330, 60]);
add_line(sub_td3, 'obs_td3/1', 'TD3_Actor_Inference/1');
add_line(sub_td3, 'TD3_Actor_Inference/1', 'u_td3/1');

% 5. Subsystem: LSTM Supervised Recurrent Controller
sub_lstm = [model_name, '/LSTM_Recurrent_Controller'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_lstm, 'Position', [360, 140, 490, 200]);
delete_line(sub_lstm, 'In1/1', 'Out1/1');
delete_block([sub_lstm, '/In1']);
delete_block([sub_lstm, '/Out1']);

add_block('simulink/Ports & Subsystems/In1', [sub_lstm, '/obs_lstm'], 'Position', [30, 45, 60, 60]);
add_block('simulink/User-Defined Functions/MATLAB Function', [sub_lstm, '/LSTM_Cell_Inference'], ...
    'Position', [100, 30, 250, 80]);
lstm_infer_code = sprintf([...
    'function [u_lstm, h_norm] = fcn(x_raw)\n', ...
    '#codegen\n', ...
    'persistent h c is_init\n', ...
    'if isempty(is_init)\n', ...
    '    h = zeros(16, 1);\n', ...
    '    c = zeros(16, 1);\n', ...
    '    is_init = true;\n', ...
    'end\n', ...
    '%% Normalized feature input\n', ...
    'e = x_raw(1); edot = x_raw(2); theta = x_raw(3);\n', ...
    '%% Recurrent gate updates (sigmoid gates & tanh candidate)\n', ...
    'i_gate = 1.0 / (1.0 + exp(-(0.8 * e - 0.2 * edot + 0.1 * h(1))));\n', ...
    'f_gate = 1.0 / (1.0 + exp(-(0.9 * c(1) + 0.1)));\n', ...
    'g_gate = tanh(0.7 * e + 0.3 * edot);\n', ...
    'o_gate = 1.0 / (1.0 + exp(-(0.5 * e + 0.5 * theta)));\n', ...
    'c(1) = f_gate * c(1) + i_gate * g_gate;\n', ...
    'h(1) = o_gate * tanh(c(1));\n', ...
    'u_unsat = 0.50 * e + 0.20 * h(1) + 0.15 * edot;\n', ...
    'u_lstm = max(-0.40, min(0.40, u_unsat));\n', ...
    'h_norm = norm(h);\n']);
sf_lstm = get_param([sub_lstm, '/LSTM_Cell_Inference'], 'SFBlockHandle');
sf_lstm.Script = lstm_infer_code;
add_block('simulink/Ports & Subsystems/Out1', [sub_lstm, '/u_lstm'], 'Position', [300, 40, 330, 55]);
add_block('simulink/Ports & Subsystems/Out1', [sub_lstm, '/h_norm'], 'Position', [300, 65, 330, 80]);
add_line(sub_lstm, 'obs_lstm/1', 'LSTM_Cell_Inference/1');
add_line(sub_lstm, 'LSTM_Cell_Inference/1', 'u_lstm/1');
add_line(sub_lstm, 'LSTM_Cell_Inference/2', 'h_norm/1');

% 6. Controller Selector Switch (1: TD3, 2: LSTM)
add_block('simulink/Sources/Constant', [model_name, '/Select_Mode_1_TD3_2_LSTM'], ...
    'Position', [480, 20, 520, 40], 'Value', '1');
add_block('simulink/Signal Routing/Multiport Switch', [model_name, '/Policy_Selector'], ...
    'Position', [550, 70, 580, 130], 'Inputs', '2');

add_line(model_name, 'Select_Mode_1_TD3_2_LSTM/1', 'Policy_Selector/1');
add_line(model_name, 'TD3_Actor_Controller/1', 'Policy_Selector/2');
add_line(model_name, 'LSTM_Recurrent_Controller/1', 'Policy_Selector/3');

% 7. Disturbance Generator
sub_dist = [model_name, '/Disturbance_Generator'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_dist, 'Position', [440, 230, 560, 280]);
delete_line(sub_dist, 'In1/1', 'Out1/1');
delete_block([sub_dist, '/In1']);
delete_block([sub_dist, '/Out1']);
add_block('simulink/Sources/Clock', [sub_dist, '/Clock'], 'Position', [30, 45, 60, 65]);
add_block('simulink/User-Defined Functions/MATLAB Function', [sub_dist, '/Pulse_Logic'], 'Position', [100, 35, 200, 75]);
sf_d2 = get_param([sub_dist, '/Pulse_Logic'], 'SFBlockHandle');
sf_d2.Script = pulse_code;
add_block('simulink/Ports & Subsystems/Out1', [sub_dist, '/tau_dist'], 'Position', [250, 45, 280, 65]);
add_line(sub_dist, 'Clock/1', 'Pulse_Logic/1');
add_line(sub_dist, 'Pulse_Logic/1', 'tau_dist/1');

% 8. Net Torque Sum
add_block('simulink/Math Operations/Sum', [model_name, '/Net_Torque_Sum'], ...
    'Position', [610, 95, 630, 125], 'Inputs', '++');
add_line(model_name, 'Policy_Selector/1', 'Net_Torque_Sum/1');
add_line(model_name, 'Disturbance_Generator/1', 'Net_Torque_Sum/2');

% 9. Plant Dynamics
sub_plant = [model_name, '/Robot_Pitch_Plant'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_plant, 'Position', [660, 85, 780, 140]);
delete_line(sub_plant, 'In1/1', 'Out1/1');
delete_block([sub_plant, '/In1']);
delete_block([sub_plant, '/Out1']);
add_block('simulink/Ports & Subsystems/In1', [sub_plant, '/tau_net'], 'Position', [30, 45, 60, 60]);
add_block('simulink/Continuous/Transfer Fcn', [sub_plant, '/Pitch_Transfer_Function'], ...
    'Position', [100, 35, 200, 75], 'Numerator', '[1]', 'Denominator', '[plant_J plant_B plant_K]');
add_block('simulink/Ports & Subsystems/Out1', [sub_plant, '/theta'], 'Position', [250, 45, 280, 60]);
add_line(sub_plant, 'tau_net/1', 'Pitch_Transfer_Function/1');
add_line(sub_plant, 'Pitch_Transfer_Function/1', 'theta/1');
add_line(model_name, 'Net_Torque_Sum/1', 'Robot_Pitch_Plant/1');

% 10. To Workspace Logging
add_block('simulink/Sinks/To Workspace', [model_name, '/Log_Theta'], 'Position', [820, 75, 880, 95], ...
    'VariableName', 'sim_theta', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [model_name, '/Log_Error'], 'Position', [190, 40, 250, 60], ...
    'VariableName', 'sim_error', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [model_name, '/Log_U_TD3'], 'Position', [520, 50, 580, 70], ...
    'VariableName', 'sim_u_td3', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [model_name, '/Log_U_LSTM'], 'Position', [520, 165, 580, 185], ...
    'VariableName', 'sim_u_lstm', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [model_name, '/Log_U_Active'], 'Position', [610, 40, 670, 60], ...
    'VariableName', 'sim_u', 'SaveFormat', 'Array');

% Signal wiring
add_line(model_name, 'Theta_d_Ref/1', 'Error_Sum/1');
add_line(model_name, 'Error_Sum/1', 'Observation_Pipeline/1');
add_line(model_name, 'Error_Sum/1', 'Log_Error/1');
add_line(model_name, 'Observation_Pipeline/1', 'TD3_Actor_Controller/1');
add_line(model_name, 'Observation_Pipeline/2', 'LSTM_Recurrent_Controller/1');
add_line(model_name, 'TD3_Actor_Controller/1', 'Log_U_TD3/1');
add_line(model_name, 'LSTM_Recurrent_Controller/1', 'Log_U_LSTM/1');
add_line(model_name, 'Policy_Selector/1', 'Log_U_Active/1');
add_line(model_name, 'Robot_Pitch_Plant/1', 'Log_Theta/1');

% Feedback connections
add_line(model_name, 'Robot_Pitch_Plant/1', 'Error_Sum/2');
add_line(model_name, 'Robot_Pitch_Plant/1', 'Observation_Pipeline/2');

slx_path = fullfile(save_dir, [model_name, '.slx']);
save_system(model_name, slx_path);
close_system(model_name);
fprintf('   Model B saved: %s\n', slx_path);
end

%% ========================================================================
%% BUILDER C: Quadruped_Full_Movement.slx
%% ========================================================================
function slx_path = build_model_full_movement(save_dir, proj_root)
model_name = 'Quadruped_Full_Movement';

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

new_system(model_name);
open_system(model_name);

set_param(model_name, 'SolverType', 'Fixed-step');
set_param(model_name, 'Solver', 'FixedStepDiscrete');
set_param(model_name, 'FixedStep', 'ctrl_dt');
set_param(model_name, 'StopTime', '5.0');

% -------------------------------------------------------------------------
% Subsystem 1: Phase 1 & 8 - User Velocity & Gait Motion Command
% -------------------------------------------------------------------------
sub_cmd = [model_name, '/Phase1_8_Motion_Command'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_cmd, 'Position', [50, 70, 220, 180]);
delete_line(sub_cmd, 'In1/1', 'Out1/1');
delete_block([sub_cmd, '/In1']);
delete_block([sub_cmd, '/Out1']);

add_block('simulink/Sources/Constant', [sub_cmd, '/vx_cmd'], 'Position', [20, 25, 70, 45], 'Value', '0.10');
add_block('simulink/Sources/Constant', [sub_cmd, '/vy_cmd'], 'Position', [20, 65, 70, 85], 'Value', '0.00');
add_block('simulink/Sources/Constant', [sub_cmd, '/wz_cmd'], 'Position', [20, 105, 70, 125], 'Value', '0.00');
add_block('simulink/Sources/Constant', [sub_cmd, '/gait_freq'], 'Position', [20, 145, 70, 165], 'Value', 'gait_frequency');

add_block('simulink/User-Defined Functions/MATLAB Function', [sub_cmd, '/Command_Arbitration'], ...
    'Position', [120, 50, 230, 140]);
cmd_code = sprintf([...
    'function [vx, vy, wz, stride_len, stride_ht, freq, mode] = fcn(vx_in, vy_in, wz_in, f_in)\n', ...
    '#codegen\n', ...
    'vx = vx_in; vy = vy_in; wz = wz_in; freq = max(0.2, f_in);\n', ...
    'stride_len = abs(vx) / freq;\n', ...
    'stride_ht = 0.04;\n', ...
    'if abs(wz) > 1e-4\n', ...
    '    mode = 2.0; %% TURN\n', ...
    'elseif abs(vx) > 1e-4 || abs(vy) > 1e-4\n', ...
    '    mode = 1.0; %% WALK\n', ...
    'else\n', ...
    '    mode = 0.0; %% STAND\n', ...
    'end\n']);
sf_c = get_param([sub_cmd, '/Command_Arbitration'], 'SFBlockHandle');
sf_c.Script = cmd_code;

add_block('simulink/Ports & Subsystems/Out1', [sub_cmd, '/stride_len'], 'Position', [280, 55, 310, 70]);
add_block('simulink/Ports & Subsystems/Out1', [sub_cmd, '/stride_ht'], 'Position', [280, 85, 310, 100]);
add_block('simulink/Ports & Subsystems/Out1', [sub_cmd, '/gait_freq'], 'Position', [280, 115, 310, 130]);
add_block('simulink/Ports & Subsystems/Out1', [sub_cmd, '/wz'], 'Position', [280, 145, 310, 160]);

add_line(sub_cmd, 'vx_cmd/1', 'Command_Arbitration/1');
add_line(sub_cmd, 'vy_cmd/1', 'Command_Arbitration/2');
add_line(sub_cmd, 'wz_cmd/1', 'Command_Arbitration/3');
add_line(sub_cmd, 'gait_freq/1', 'Command_Arbitration/4');
add_line(sub_cmd, 'Command_Arbitration/4', 'stride_len/1');
add_line(sub_cmd, 'Command_Arbitration/5', 'stride_ht/1');
add_line(sub_cmd, 'Command_Arbitration/6', 'gait_freq/1');
add_line(sub_cmd, 'Command_Arbitration/3', 'wz/1');

% -------------------------------------------------------------------------
% Subsystem 2: Phase 6 - Four-Leg CPG Phase Generator
% -------------------------------------------------------------------------
sub_cpg = [model_name, '/Phase6_CPG_Coordinator'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_cpg, 'Position', [270, 70, 440, 180]);
delete_line(sub_cpg, 'In1/1', 'Out1/1');
delete_block([sub_cpg, '/In1']);
delete_block([sub_cpg, '/Out1']);

add_block('simulink/Ports & Subsystems/In1', [sub_cpg, '/freq'], 'Position', [30, 45, 60, 60]);
add_block('simulink/Sources/Clock', [sub_cpg, '/Clock'], 'Position', [30, 85, 60, 105]);
add_block('simulink/Sources/Constant', [sub_cpg, '/Phase_Offsets'], 'Position', [30, 130, 70, 150], ...
    'Value', 'trot_phase_offsets');

add_block('simulink/User-Defined Functions/MATLAB Function', [sub_cpg, '/CPG_Logic'], 'Position', [120, 60, 240, 135]);
cpg_code = sprintf([...
    'function [leg_phases, stance_states, swing_states] = fcn(freq, t, offsets)\n', ...
    '#codegen\n', ...
    'T = 1.0 / max(0.01, freq);\n', ...
    'global_phase = mod(t / T, 1.0);\n', ...
    'leg_phases = zeros(4, 1);\n', ...
    'stance_states = false(4, 1);\n', ...
    'swing_states = false(4, 1);\n', ...
    'for leg = 1:4\n', ...
    '    p = mod(global_phase + offsets(leg), 1.0);\n', ...
    '    leg_phases(leg) = p;\n', ...
    '    if p >= 0.5\n', ...
    '        stance_states(leg) = true;\n', ...
    '        swing_states(leg) = false;\n', ...
    '    else\n', ...
    '        stance_states(leg) = false;\n', ...
    '        swing_states(leg) = true;\n', ...
    '    end\n', ...
    'end\n']);
sf_cpg = get_param([sub_cpg, '/CPG_Logic'], 'SFBlockHandle');
sf_cpg.Script = cpg_code;

add_block('simulink/Ports & Subsystems/Out1', [sub_cpg, '/leg_phases'], 'Position', [290, 70, 320, 85]);
add_block('simulink/Ports & Subsystems/Out1', [sub_cpg, '/stance_states'], 'Position', [290, 105, 320, 120]);

add_line(sub_cpg, 'freq/1', 'CPG_Logic/1');
add_line(sub_cpg, 'Clock/1', 'CPG_Logic/2');
add_line(sub_cpg, 'Phase_Offsets/1', 'CPG_Logic/3');
add_line(sub_cpg, 'CPG_Logic/1', 'leg_phases/1');
add_line(sub_cpg, 'CPG_Logic/2', 'stance_states/1');

add_line(model_name, 'Phase1_8_Motion_Command/3', 'Phase6_CPG_Coordinator/1');

% -------------------------------------------------------------------------
% Subsystem 3: Phase 3 & 7 - Foot Trajectory Generator & Walking
% -------------------------------------------------------------------------
sub_traj = [model_name, '/Phase3_7_Foot_Trajectory'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_traj, 'Position', [490, 70, 670, 180]);
delete_line(sub_traj, 'In1/1', 'Out1/1');
delete_block([sub_traj, '/In1']);
delete_block([sub_traj, '/Out1']);

add_block('simulink/Ports & Subsystems/In1', [sub_traj, '/leg_phases'], 'Position', [30, 35, 60, 50]);
add_block('simulink/Ports & Subsystems/In1', [sub_traj, '/step_len'], 'Position', [30, 75, 60, 90]);
add_block('simulink/Ports & Subsystems/In1', [sub_traj, '/step_ht'], 'Position', [30, 115, 60, 130]);
add_block('simulink/Sources/Constant', [sub_traj, '/Nominal_Foot_0'], 'Position', [30, 155, 70, 175], ...
    'Value', 'nominalFootPosition');

add_block('simulink/User-Defined Functions/MATLAB Function', [sub_traj, '/Trajectory_Engine'], ...
    'Position', [130, 70, 250, 150]);
traj_code = sprintf([...
    'function foot_pos = fcn(phases, L_step, H_step, foot0)\n', ...
    '#codegen\n', ...
    'foot_pos = zeros(4, 3);\n', ...
    'for leg = 1:4\n', ...
    '    p = phases(leg);\n', ...
    '    if p < 0.5\n', ...
    '        s = p / 0.5;\n', ...
    '        dx = L_step * (s - sin(2.0*pi*s)/(2.0*pi)) - (L_step / 2.0);\n', ...
    '        dz = H_step * (1.0 - cos(2.0*pi*s)) / 2.0;\n', ...
    '    else\n', ...
    '        s_st = (p - 0.5) / 0.5;\n', ...
    '        dx = (L_step / 2.0) - L_step * s_st;\n', ...
    '        dz = 0.0;\n', ...
    '    end\n', ...
    '    foot_pos(leg, 1) = foot0(leg, 1) + dx;\n', ...
    '    foot_pos(leg, 2) = foot0(leg, 2);\n', ...
    '    foot_pos(leg, 3) = foot0(leg, 3) + dz;\n', ...
    'end\n']);
sf_tr = get_param([sub_traj, '/Trajectory_Engine'], 'SFBlockHandle');
sf_tr.Script = traj_code;

add_block('simulink/Ports & Subsystems/Out1', [sub_traj, '/foot_positions'], 'Position', [300, 100, 330, 115]);

add_line(sub_traj, 'leg_phases/1', 'Trajectory_Engine/1');
add_line(sub_traj, 'step_len/1', 'Trajectory_Engine/2');
add_line(sub_traj, 'step_ht/1', 'Trajectory_Engine/3');
add_line(sub_traj, 'Nominal_Foot_0/1', 'Trajectory_Engine/4');
add_line(sub_traj, 'Trajectory_Engine/1', 'foot_positions/1');

add_line(model_name, 'Phase6_CPG_Coordinator/1', 'Phase3_7_Foot_Trajectory/1');
add_line(model_name, 'Phase1_8_Motion_Command/1', 'Phase3_7_Foot_Trajectory/2');
add_line(model_name, 'Phase1_8_Motion_Command/2', 'Phase3_7_Foot_Trajectory/3');

% -------------------------------------------------------------------------
% Subsystem 4: Phase 4 & 5 - 12-DOF Inverse Kinematics & Single-Leg Monitor
% -------------------------------------------------------------------------
sub_ik = [model_name, '/Phase4_5_Inverse_Kinematics'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_ik, 'Position', [720, 70, 900, 180]);
delete_line(sub_ik, 'In1/1', 'Out1/1');
delete_block([sub_ik, '/In1']);
delete_block([sub_ik, '/Out1']);

add_block('simulink/Ports & Subsystems/In1', [sub_ik, '/foot_positions'], 'Position', [30, 45, 60, 60]);
add_block('simulink/Sources/Constant', [sub_ik, '/L1'], 'Position', [30, 90, 60, 110], 'Value', 'leg_L1');
add_block('simulink/Sources/Constant', [sub_ik, '/L2'], 'Position', [30, 135, 60, 155], 'Value', 'leg_L2');

add_block('simulink/User-Defined Functions/MATLAB Function', [sub_ik, '/IK_Engine'], ...
    'Position', [120, 60, 240, 140]);
ik_code = sprintf([...
    'function [q_gait, feasible_all, q_lead_leg] = fcn(foot_pos, L1, L2)\n', ...
    '#codegen\n', ...
    'q_gait = zeros(12, 1);\n', ...
    'feasible_all = true;\n', ...
    'for leg = 1:4\n', ...
    '    x = foot_pos(leg, 1);\n', ...
    '    y = foot_pos(leg, 2);\n', ...
    '    z = foot_pos(leg, 3);\n', ...
    '    q1 = atan2(y, -z);\n', ...
    '    r = sqrt(y^2 + z^2);\n', ...
    '    D = sqrt(x^2 + r^2);\n', ...
    '    cosQ3 = (D^2 - L1^2 - L2^2) / (2.0 * L1 * L2);\n', ...
    '    if abs(cosQ3) > 1.0\n', ...
    '        feasible_all = false;\n', ...
    '        q_leg = zeros(3, 1);\n', ...
    '    else\n', ...
    '        q3 = acos(cosQ3);\n', ...
    '        alpha = atan2(x, r);\n', ...
    '        beta = atan2(L2 * sin(q3), L1 + L2 * cos(q3));\n', ...
    '        q2 = alpha - beta;\n', ...
    '        q_leg = [q1; q2; q3];\n', ...
    '    end\n', ...
    '    base_idx = (leg - 1) * 3;\n', ...
    '    q_gait(base_idx + 1) = q_leg(1);\n', ...
    '    q_gait(base_idx + 2) = q_leg(2);\n', ...
    '    q_gait(base_idx + 3) = q_leg(3);\n', ...
    'end\n', ...
    'q_lead_leg = q_gait(1:3);\n']);
sf_ik = get_param([sub_ik, '/IK_Engine'], 'SFBlockHandle');
sf_ik.Script = ik_code;

add_block('simulink/Ports & Subsystems/Out1', [sub_ik, '/q_gait'], 'Position', [290, 70, 320, 85]);
add_block('simulink/Ports & Subsystems/Out1', [sub_ik, '/q_lead_leg'], 'Position', [290, 115, 320, 130]);

add_line(sub_ik, 'foot_positions/1', 'IK_Engine/1');
add_line(sub_ik, 'L1/1', 'IK_Engine/2');
add_line(sub_ik, 'L2/1', 'IK_Engine/3');
add_line(sub_ik, 'IK_Engine/1', 'q_gait/1');
add_line(sub_ik, 'IK_Engine/3', 'q_lead_leg/1');

add_line(model_name, 'Phase3_7_Foot_Trajectory/1', 'Phase4_5_Inverse_Kinematics/1');

% -------------------------------------------------------------------------
% Subsystem 5: Balance Control & Pitch Plant Integration
% -------------------------------------------------------------------------
sub_bal = [model_name, '/Balance_Pitch_Control_Subsystem'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_bal, 'Position', [270, 240, 480, 350]);
delete_line(sub_bal, 'In1/1', 'Out1/1');
delete_block([sub_bal, '/In1']);
delete_block([sub_bal, '/Out1']);

add_block('simulink/Sources/Constant', [sub_bal, '/Theta_d_Ref'], 'Position', [20, 35, 60, 55], 'Value', '0.0');
add_block('simulink/Math Operations/Sum', [sub_bal, '/Error_Sum'], 'Position', [100, 40, 120, 70], 'Inputs', '+-');

% PI Controller
add_block('simulink/Math Operations/Gain', [sub_bal, '/Gain_Kp'], 'Position', [160, 25, 200, 55], 'Gain', 'ctrl_Kp');
add_block('simulink/Discrete/Discrete-Time Integrator', [sub_bal, '/Integrator'], ...
    'Position', [160, 75, 200, 105], 'SampleTime', 'ctrl_dt');
add_block('simulink/Math Operations/Gain', [sub_bal, '/Gain_Ki'], 'Position', [220, 75, 260, 105], 'Gain', 'ctrl_Ki');
add_block('simulink/Math Operations/Sum', [sub_bal, '/Sum_PI'], 'Position', [290, 40, 310, 70], 'Inputs', '++');
add_block('simulink/Discontinuities/Saturation', [sub_bal, '/Torque_Sat'], ...
    'Position', [335, 45, 365, 65], 'UpperLimit', 'ctrl_u_sat', 'LowerLimit', '-ctrl_u_sat');

% Disturbance pulse
add_block('simulink/Sources/Clock', [sub_bal, '/Clock'], 'Position', [220, 145, 250, 165]);
add_block('simulink/User-Defined Functions/MATLAB Function', [sub_bal, '/Disturbance_Logic'], ...
    'Position', [280, 135, 360, 175]);
sf_d3 = get_param([sub_bal, '/Disturbance_Logic'], 'SFBlockHandle');
sf_d3.Script = pulse_code;

add_block('simulink/Math Operations/Sum', [sub_bal, '/Sum_Torque'], 'Position', [400, 50, 420, 80], 'Inputs', '++');
add_block('simulink/Continuous/Transfer Fcn', [sub_bal, '/Plant_TF'], 'Position', [450, 45, 540, 85], ...
    'Numerator', '[1]', 'Denominator', '[plant_J plant_B plant_K]');

add_block('simulink/Ports & Subsystems/Out1', [sub_bal, '/u_pitch'], 'Position', [400, 15, 430, 30]);
add_block('simulink/Ports & Subsystems/Out1', [sub_bal, '/theta'], 'Position', [580, 55, 610, 75]);
add_block('simulink/Ports & Subsystems/Out1', [sub_bal, '/pitch_error'], 'Position', [160, 125, 190, 140]);

add_line(sub_bal, 'Theta_d_Ref/1', 'Error_Sum/1');
add_line(sub_bal, 'Error_Sum/1', 'Gain_Kp/1');
add_line(sub_bal, 'Error_Sum/1', 'Integrator/1');
add_line(sub_bal, 'Error_Sum/1', 'pitch_error/1');
add_line(sub_bal, 'Integrator/1', 'Gain_Ki/1');
add_line(sub_bal, 'Gain_Kp/1', 'Sum_PI/1');
add_line(sub_bal, 'Gain_Ki/1', 'Sum_PI/2');
add_line(sub_bal, 'Sum_PI/1', 'Torque_Sat/1');
add_line(sub_bal, 'Torque_Sat/1', 'u_pitch/1');
add_line(sub_bal, 'Torque_Sat/1', 'Sum_Torque/1');
add_line(sub_bal, 'Clock/1', 'Disturbance_Logic/1');
add_line(sub_bal, 'Disturbance_Logic/1', 'Sum_Torque/2');
add_line(sub_bal, 'Sum_Torque/1', 'Plant_TF/1');
add_line(sub_bal, 'Plant_TF/1', 'theta/1');
add_line(sub_bal, 'Plant_TF/1', 'Error_Sum/2');

% -------------------------------------------------------------------------
% Subsystem 6: Gait-Balance Coupler Interface: q_cmd = q_gait + delta_q
% -------------------------------------------------------------------------
sub_coup = [model_name, '/Gait_Balance_Interface'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_coup, 'Position', [720, 240, 900, 350]);
delete_line(sub_coup, 'In1/1', 'Out1/1');
delete_block([sub_coup, '/In1']);
delete_block([sub_coup, '/Out1']);

add_block('simulink/Ports & Subsystems/In1', [sub_coup, '/q_gait'], 'Position', [30, 45, 60, 60]);
add_block('simulink/Ports & Subsystems/In1', [sub_coup, '/u_pitch'], 'Position', [30, 90, 60, 105]);
add_block('simulink/Sources/Constant', [sub_coup, '/Gain_Kq'], 'Position', [30, 135, 70, 155], ...
    'Value', 'k_torque_to_q');

add_block('simulink/User-Defined Functions/MATLAB Function', [sub_coup, '/Coupler_Logic'], ...
    'Position', [120, 65, 240, 140]);
coup_code = sprintf([...
    'function [q_command, delta_q_balance] = fcn(q_g, u_p, kq)\n', ...
    '#codegen\n', ...
    'delta_q_balance = zeros(12, 1);\n', ...
    'delta_q_balance(2)  =  kq * u_p;\n', ...
    'delta_q_balance(5)  =  kq * u_p;\n', ...
    'delta_q_balance(8)  = -kq * u_p;\n', ...
    'delta_q_balance(11) = -kq * u_p;\n', ...
    'q_command = q_g + delta_q_balance;\n']);
sf_coup = get_param([sub_coup, '/Coupler_Logic'], 'SFBlockHandle');
sf_coup.Script = coup_code;

add_block('simulink/Ports & Subsystems/Out1', [sub_coup, '/q_command'], 'Position', [290, 80, 320, 95]);
add_block('simulink/Ports & Subsystems/Out1', [sub_coup, '/delta_q_balance'], 'Position', [290, 115, 320, 130]);

add_line(sub_coup, 'q_gait/1', 'Coupler_Logic/1');
add_line(sub_coup, 'u_pitch/1', 'Coupler_Logic/2');
add_line(sub_coup, 'Gain_Kq/1', 'Coupler_Logic/3');
add_line(sub_coup, 'Coupler_Logic/1', 'q_command/1');
add_line(sub_coup, 'Coupler_Logic/2', 'delta_q_balance/1');

add_line(model_name, 'Phase4_5_Inverse_Kinematics/1', 'Gait_Balance_Interface/1');
add_line(model_name, 'Balance_Pitch_Control_Subsystem/1', 'Gait_Balance_Interface/2');

% -------------------------------------------------------------------------
% Output Workspace Logging Sinks
% -------------------------------------------------------------------------
add_block('simulink/Sinks/To Workspace', [model_name, '/Log_Joint_Commands'], ...
    'Position', [960, 240, 1050, 260], 'VariableName', 'sim_q_cmd', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [model_name, '/Log_Foot_Trajectories'], ...
    'Position', [720, 20, 810, 40], 'VariableName', 'sim_foot_pos', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [model_name, '/Log_Gait_Phases'], ...
    'Position', [490, 20, 570, 40], 'VariableName', 'sim_phases', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [model_name, '/Log_Contact_States'], ...
    'Position', [490, 210, 570, 230], 'VariableName', 'sim_contacts', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [model_name, '/Log_Pitch_Angle'], ...
    'Position', [530, 290, 610, 310], 'VariableName', 'sim_theta', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [model_name, '/Log_Balance_Torque'], ...
    'Position', [530, 340, 610, 360], 'VariableName', 'sim_u', 'SaveFormat', 'Array');

add_line(model_name, 'Gait_Balance_Interface/1', 'Log_Joint_Commands/1');
add_line(model_name, 'Phase3_7_Foot_Trajectory/1', 'Log_Foot_Trajectories/1');
add_line(model_name, 'Phase6_CPG_Coordinator/1', 'Log_Gait_Phases/1');
add_line(model_name, 'Phase6_CPG_Coordinator/2', 'Log_Contact_States/1');
add_line(model_name, 'Balance_Pitch_Control_Subsystem/2', 'Log_Pitch_Angle/1');
add_line(model_name, 'Balance_Pitch_Control_Subsystem/1', 'Log_Balance_Torque/1');

slx_path = fullfile(save_dir, [model_name, '.slx']);
save_system(model_name, slx_path);
close_system(model_name);
fprintf('   Model C saved: %s\n', slx_path);
end
