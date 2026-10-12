function model_name = build_gait_model(save_dir)
% BUILD_GAIT_MODEL Programmatic builder for Quadruped_Gait_Unified.slx.
%
% Constructs the top-level Simulink architecture integrating:
%   1. Motion & Gait Configuration Subsystem
%   2. CPG Phase Generator Subsystem
%   3. Four-Leg Foot Trajectory Subsystem
%   4. Analytical Inverse Kinematics Subsystem
%   5. Gait-Balance Integration Subsystem
%   6. Robot Movement & Telemetry Subsystem
%   7. External Disturbance Injection Subsystem
%
% Architecture is completely self-contained and reproducible.

if nargin < 1 || isempty(save_dir)
    save_dir = fileparts(mfilename('fullpath'));
end

model_name = 'Quadruped_Gait_Unified';

fprintf('============================================================\n');
fprintf('  BUILDING UNIFIED SIMULINK GAIT MODEL: %s\n', model_name);
fprintf('============================================================\n');

% Initialize configuration in workspace first
gait_cfg = initialize_gait_model();

% Close existing model if open
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

% Create new system
new_system(model_name);
open_system(model_name);

% Configure Solver Settings
set_param(model_name, 'SolverType', 'Fixed-step');
set_param(model_name, 'Solver', 'FixedStepDiscrete');
set_param(model_name, 'FixedStep', num2str(gait_cfg.dt));
set_param(model_name, 'StopTime', num2str(gait_cfg.t_final));

fprintf('Constructing Subsystems and Signal Routing...\n');

% -------------------------------------------------------------
% Subsystem 1: CPG Gait Phase Generator
% -------------------------------------------------------------
sub_cpg = [model_name, '/CPG_Gait_Phase_Generator'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_cpg, 'Position', [100, 100, 300, 220]);

% Add ports inside CPG subsystem
delete_line(sub_cpg, 'In1/1', 'Out1/1');
delete_block([sub_cpg, '/In1']);
delete_block([sub_cpg, '/Out1']);

add_block('simulink/Sources/Clock', [sub_cpg, '/Clock'], 'Position', [30, 45, 60, 65]);
add_block('simulink/Sources/Constant', [sub_cpg, '/Frequency'], 'Position', [30, 95, 70, 115], ...
    'Value', 'gait_cfg.frequency');
add_block('simulink/Sources/Constant', [sub_cpg, '/Phase_Offsets'], 'Position', [30, 145, 70, 165], ...
    'Value', 'gait_cfg.phaseOffsets');

add_block('simulink/User-Defined Functions/MATLAB Function', [sub_cpg, '/CPG_Logic'], 'Position', [120, 60, 240, 150]);
cpg_code = sprintf([...
    'function [leg_phases, stance_states, swing_states] = fcn(t, freq, offsets)\n', ...
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

sf = get_param([sub_cpg, '/CPG_Logic'], 'SFBlockHandle');
sf.Script = cpg_code;

add_block('simulink/Ports & Subsystems/Out1', [sub_cpg, '/leg_phases'], 'Position', [290, 70, 320, 85]);
add_block('simulink/Ports & Subsystems/Out1', [sub_cpg, '/stance_states'], 'Position', [290, 100, 320, 115]);
add_block('simulink/Ports & Subsystems/Out1', [sub_cpg, '/swing_states'], 'Position', [290, 130, 320, 145]);

add_line(sub_cpg, 'Clock/1', 'CPG_Logic/1');
add_line(sub_cpg, 'Frequency/1', 'CPG_Logic/2');
add_line(sub_cpg, 'Phase_Offsets/1', 'CPG_Logic/3');
add_line(sub_cpg, 'CPG_Logic/1', 'leg_phases/1');
add_line(sub_cpg, 'CPG_Logic/2', 'stance_states/1');
add_line(sub_cpg, 'CPG_Logic/3', 'swing_states/1');

% -------------------------------------------------------------
% Subsystem 2: Four-Leg Foot Trajectory Generator
% -------------------------------------------------------------
sub_traj = [model_name, '/Foot_Trajectory_Generator'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_traj, 'Position', [360, 100, 560, 220]);

delete_line(sub_traj, 'In1/1', 'Out1/1');
delete_block([sub_traj, '/In1']);
delete_block([sub_traj, '/Out1']);

add_block('simulink/Ports & Subsystems/In1', [sub_traj, '/leg_phases'], 'Position', [30, 45, 60, 60]);
add_block('simulink/Sources/Constant', [sub_traj, '/Nominal_Foot_0'], 'Position', [30, 85, 80, 105], ...
    'Value', 'gait_cfg.nominalFootPosition');
add_block('simulink/Sources/Constant', [sub_traj, '/Step_Length'], 'Position', [30, 130, 70, 150], ...
    'Value', 'gait_cfg.stepLength');
add_block('simulink/Sources/Constant', [sub_traj, '/Step_Height'], 'Position', [30, 175, 70, 195], ...
    'Value', 'gait_cfg.stepHeight');

add_block('simulink/User-Defined Functions/MATLAB Function', [sub_traj, '/Traj_Logic'], 'Position', [130, 80, 250, 160]);
traj_code = sprintf([...
    'function foot_pos = fcn(phases, foot0, stepLen, stepHt)\n', ...
    '#codegen\n', ...
    'foot_pos = zeros(4, 3);\n', ...
    'for leg = 1:4\n', ...
    '    p = phases(leg);\n', ...
    '    if p < 0.5\n', ...
    '        s = p / 0.5;\n', ...
    '        dx = stepLen * (s - sin(2.0*pi*s)/(2.0*pi)) - (stepLen / 2.0);\n', ...
    '        dz = stepHt * (1.0 - cos(2.0*pi*s)) / 2.0;\n', ...
    '    else\n', ...
    '        s_st = (p - 0.5) / 0.5;\n', ...
    '        dx = (stepLen / 2.0) - stepLen * s_st;\n', ...
    '        dz = 0.0;\n', ...
    '    end\n', ...
    '    foot_pos(leg, 1) = foot0(leg, 1) + dx;\n', ...
    '    foot_pos(leg, 2) = foot0(leg, 2);\n', ...
    '    foot_pos(leg, 3) = foot0(leg, 3) + dz;\n', ...
    'end\n']);

sf_traj = get_param([sub_traj, '/Traj_Logic'], 'SFBlockHandle');
sf_traj.Script = traj_code;

add_block('simulink/Ports & Subsystems/Out1', [sub_traj, '/foot_positions'], 'Position', [300, 110, 330, 130]);

add_line(sub_traj, 'leg_phases/1', 'Traj_Logic/1');
add_line(sub_traj, 'Nominal_Foot_0/1', 'Traj_Logic/2');
add_line(sub_traj, 'Step_Length/1', 'Traj_Logic/3');
add_line(sub_traj, 'Step_Height/1', 'Traj_Logic/4');
add_line(sub_traj, 'Traj_Logic/1', 'foot_positions/1');

% Connect CPG to Trajectory
add_line(model_name, 'CPG_Gait_Phase_Generator/1', 'Foot_Trajectory_Generator/1');

% -------------------------------------------------------------
% Subsystem 3: 12-DOF Inverse Kinematics
% -------------------------------------------------------------
sub_ik = [model_name, '/Inverse_Kinematics'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_ik, 'Position', [620, 100, 820, 220]);

delete_line(sub_ik, 'In1/1', 'Out1/1');
delete_block([sub_ik, '/In1']);
delete_block([sub_ik, '/Out1']);

add_block('simulink/Ports & Subsystems/In1', [sub_ik, '/foot_positions'], 'Position', [30, 50, 60, 65]);
add_block('simulink/Sources/Constant', [sub_ik, '/L1'], 'Position', [30, 95, 60, 115], 'Value', 'gait_cfg.L1');
add_block('simulink/Sources/Constant', [sub_ik, '/L2'], 'Position', [30, 140, 60, 160], 'Value', 'gait_cfg.L2');

add_block('simulink/User-Defined Functions/MATLAB Function', [sub_ik, '/IK_Logic'], 'Position', [120, 70, 240, 150]);
ik_code = sprintf([...
    'function [q_gait, feasible] = fcn(foot_pos, L1, L2)\n', ...
    '#codegen\n', ...
    'q_gait = zeros(12, 1);\n', ...
    'feasible = true(4, 1);\n', ...
    'for leg = 1:4\n', ...
    '    x = foot_pos(leg, 1);\n', ...
    '    y = foot_pos(leg, 2);\n', ...
    '    z = foot_pos(leg, 3);\n', ...
    '    q1 = atan2(y, -z);\n', ...
    '    r = sqrt(y^2 + z^2);\n', ...
    '    D = sqrt(x^2 + r^2);\n', ...
    '    cosQ3 = (D^2 - L1^2 - L2^2) / (2.0 * L1 * L2);\n', ...
    '    if abs(cosQ3) > 1.0\n', ...
    '        feasible(leg) = false;\n', ...
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
    'end\n']);

sf_ik = get_param([sub_ik, '/IK_Logic'], 'SFBlockHandle');
sf_ik.Script = ik_code;

add_block('simulink/Ports & Subsystems/Out1', [sub_ik, '/q_gait'], 'Position', [290, 85, 320, 105]);
add_block('simulink/Ports & Subsystems/Out1', [sub_ik, '/feasible'], 'Position', [290, 120, 320, 140]);

add_line(sub_ik, 'foot_positions/1', 'IK_Logic/1');
add_line(sub_ik, 'L1/1', 'IK_Logic/2');
add_line(sub_ik, 'L2/1', 'IK_Logic/3');
add_line(sub_ik, 'IK_Logic/1', 'q_gait/1');
add_line(sub_ik, 'IK_Logic/2', 'feasible/1');

% Connect Foot Trajectory to IK
add_line(model_name, 'Foot_Trajectory_Generator/1', 'Inverse_Kinematics/1');

% -------------------------------------------------------------
% Subsystem 4: Gait-Balance Coupling & Pitch Interface
% -------------------------------------------------------------
sub_bal = [model_name, '/Gait_Balance_Interface'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_bal, 'Position', [620, 270, 820, 380]);

delete_line(sub_bal, 'In1/1', 'Out1/1');
delete_block([sub_bal, '/In1']);
delete_block([sub_bal, '/Out1']);

add_block('simulink/Ports & Subsystems/In1', [sub_bal, '/q_gait'], 'Position', [30, 50, 60, 65]);
add_block('simulink/Ports & Subsystems/In1', [sub_bal, '/u_pitch'], 'Position', [30, 95, 60, 110]);
add_block('simulink/Sources/Constant', [sub_bal, '/Gain_Kq'], 'Position', [30, 140, 70, 160], ...
    'Value', 'gait_cfg.balance.k_torque_to_q');

add_block('simulink/User-Defined Functions/MATLAB Function', [sub_bal, '/Coupler_Logic'], 'Position', [120, 70, 240, 150]);
coup_code = sprintf([...
    'function [q_command, delta_q] = fcn(q_g, u_p, kq)\n', ...
    '#codegen\n', ...
    'delta_q = zeros(12, 1);\n', ...
    'delta_q(2)  =  kq * u_p;\n', ...
    'delta_q(5)  =  kq * u_p;\n', ...
    'delta_q(8)  = -kq * u_p;\n', ...
    'delta_q(11) = -kq * u_p;\n', ...
    'q_command = q_g + delta_q;\n']);

sf_coup = get_param([sub_bal, '/Coupler_Logic'], 'SFBlockHandle');
sf_coup.Script = coup_code;

add_block('simulink/Ports & Subsystems/Out1', [sub_bal, '/q_command'], 'Position', [290, 85, 320, 105]);
add_block('simulink/Ports & Subsystems/Out1', [sub_bal, '/delta_q_balance'], 'Position', [290, 120, 320, 140]);

add_line(sub_bal, 'q_gait/1', 'Coupler_Logic/1');
add_line(sub_bal, 'u_pitch/1', 'Coupler_Logic/2');
add_line(sub_bal, 'Gain_Kq/1', 'Coupler_Logic/3');
add_line(sub_bal, 'Coupler_Logic/1', 'q_command/1');
add_line(sub_bal, 'Coupler_Logic/2', 'delta_q_balance/1');

% Connect IK q_gait to Balance Coupler
add_line(model_name, 'Inverse_Kinematics/1', 'Gait_Balance_Interface/1');

% -------------------------------------------------------------
% Subsystem 5: SISO Balance Control Subsystem (PI Controller)
% -------------------------------------------------------------
sub_pi = [model_name, '/SISO_Balance_Controller'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_pi, 'Position', [360, 270, 560, 380]);

delete_line(sub_pi, 'In1/1', 'Out1/1');
delete_block([sub_pi, '/In1']);
delete_block([sub_pi, '/Out1']);

add_block('simulink/Ports & Subsystems/In1', [sub_pi, '/theta_d'], 'Position', [30, 45, 60, 60]);
add_block('simulink/Ports & Subsystems/In1', [sub_pi, '/theta_actual'], 'Position', [30, 85, 60, 100]);

add_block('simulink/Math Operations/Sum', [sub_pi, '/Error_Sum'], 'Position', [100, 55, 120, 85], ...
    'Inputs', '+-');
add_block('simulink/Discrete/Discrete-Time Integrator', [sub_pi, '/Integrator'], 'Position', [160, 110, 200, 140], ...
    'SampleTime', 'gait_cfg.dt');
add_block('simulink/Math Operations/Gain', [sub_pi, '/Gain_Kp'], 'Position', [160, 55, 200, 85], ...
    'Gain', 'gait_cfg.balance.Kp');
add_block('simulink/Math Operations/Gain', [sub_pi, '/Gain_Ki'], 'Position', [230, 110, 270, 140], ...
    'Gain', 'gait_cfg.balance.Ki');
add_block('simulink/Math Operations/Sum', [sub_pi, '/Sum_PI'], 'Position', [300, 65, 320, 95], ...
    'Inputs', '++');
add_block('simulink/Discontinuities/Saturation', [sub_pi, '/Torque_Saturation'], 'Position', [340, 70, 370, 90], ...
    'UpperLimit', 'gait_cfg.balance.u_sat', 'LowerLimit', '-gait_cfg.balance.u_sat');
add_block('simulink/Ports & Subsystems/Out1', [sub_pi, '/u_pitch'], 'Position', [410, 70, 440, 90]);
add_block('simulink/Ports & Subsystems/Out1', [sub_pi, '/pitch_error'], 'Position', [410, 25, 440, 45]);

add_line(sub_pi, 'theta_d/1', 'Error_Sum/1');
add_line(sub_pi, 'theta_actual/1', 'Error_Sum/2');
add_line(sub_pi, 'Error_Sum/1', 'pitch_error/1');
add_line(sub_pi, 'Error_Sum/1', 'Gain_Kp/1');
add_line(sub_pi, 'Error_Sum/1', 'Integrator/1');
add_line(sub_pi, 'Integrator/1', 'Gain_Ki/1');
add_line(sub_pi, 'Gain_Kp/1', 'Sum_PI/1');
add_line(sub_pi, 'Gain_Ki/1', 'Sum_PI/2');
add_line(sub_pi, 'Sum_PI/1', 'Torque_Saturation/1');
add_line(sub_pi, 'Torque_Saturation/1', 'u_pitch/1');

% Connect PI Output to Gait-Balance Coupler
add_line(model_name, 'SISO_Balance_Controller/1', 'Gait_Balance_Interface/2');

% -------------------------------------------------------------
% Subsystem 6: SISO Pitch Plant Dynamics & Disturbance
% -------------------------------------------------------------
sub_plant = [model_name, '/Pitch_Plant_Dynamics'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_plant, 'Position', [100, 270, 300, 380]);

delete_line(sub_plant, 'In1/1', 'Out1/1');
delete_block([sub_plant, '/In1']);
delete_block([sub_plant, '/Out1']);

add_block('simulink/Ports & Subsystems/In1', [sub_plant, '/u_pitch'], 'Position', [30, 45, 60, 60]);
add_block('simulink/Sources/Clock', [sub_plant, '/Clock'], 'Position', [30, 85, 60, 105]);

% Disturbance injection block
add_block('simulink/User-Defined Functions/MATLAB Function', [sub_plant, '/Disturbance_Logic'], 'Position', [90, 80, 180, 130]);
dist_code = sprintf([...
    'function tau_dist = fcn(t)\n', ...
    '#codegen\n', ...
    'tau_dist = 0.0;\n', ...
    'if t >= 2.0 && t <= 2.5\n', ...
    '    tau_dist = 0.10;\n', ...
    'end\n']);
sf_dist = get_param([sub_plant, '/Disturbance_Logic'], 'SFBlockHandle');
sf_dist.Script = dist_code;

add_block('simulink/Math Operations/Sum', [sub_plant, '/Sum_Torque'], 'Position', [210, 50, 230, 80], 'Inputs', '++');
add_block('simulink/Continuous/Transfer Fcn', [sub_plant, '/Plant_TF'], 'Position', [260, 45, 360, 85], ...
    'Numerator', '[1]', 'Denominator', '[gait_cfg.balance.J, gait_cfg.balance.B, gait_cfg.balance.K]');

add_block('simulink/Ports & Subsystems/Out1', [sub_plant, '/theta'], 'Position', [400, 55, 430, 75]);
add_block('simulink/Ports & Subsystems/Out1', [sub_plant, '/tau_dist'], 'Position', [400, 105, 430, 125]);

add_line(sub_plant, 'Clock/1', 'Disturbance_Logic/1');
add_line(sub_plant, 'u_pitch/1', 'Sum_Torque/1');
add_line(sub_plant, 'Disturbance_Logic/1', 'Sum_Torque/2');
add_line(sub_plant, 'Sum_Torque/1', 'Plant_TF/1');
add_line(sub_plant, 'Plant_TF/1', 'theta/1');
add_line(sub_plant, 'Disturbance_Logic/1', 'tau_dist/1');

% Connect PI u_pitch into Pitch Plant
add_line(model_name, 'SISO_Balance_Controller/1', 'Pitch_Plant_Dynamics/1');

% Feed Plant Theta back to SISO PI
add_line(model_name, 'Pitch_Plant_Dynamics/1', 'SISO_Balance_Controller/2');

% -------------------------------------------------------------
% Reference Command Source (Theta_d = 0)
% -------------------------------------------------------------
add_block('simulink/Sources/Constant', [model_name, '/Theta_d_Ref'], 'Position', [360, 230, 400, 250], 'Value', '0.0');
add_line(model_name, 'Theta_d_Ref/1', 'SISO_Balance_Controller/1');

% -------------------------------------------------------------
% Output Sinks & Scopes
% -------------------------------------------------------------
add_block('simulink/Sinks/To Workspace', [model_name, '/To_WS_Gait_Phases'], 'Position', [330, 50, 420, 70], ...
    'VariableName', 'sim_phases', 'SaveFormat', 'Structure With Time');
add_block('simulink/Sinks/To Workspace', [model_name, '/To_WS_Foot_Positions'], 'Position', [600, 50, 700, 70], ...
    'VariableName', 'sim_feet', 'SaveFormat', 'Structure With Time');
add_block('simulink/Sinks/To Workspace', [model_name, '/To_WS_Joint_Commands'], 'Position', [860, 280, 960, 300], ...
    'VariableName', 'sim_joints', 'SaveFormat', 'Structure With Time');
add_block('simulink/Sinks/To Workspace', [model_name, '/To_WS_Balance_Torque'], 'Position', [600, 410, 700, 430], ...
    'VariableName', 'sim_torque', 'SaveFormat', 'Structure With Time');
add_block('simulink/Sinks/To Workspace', [model_name, '/To_WS_Pitch_Angle'], 'Position', [330, 410, 420, 430], ...
    'VariableName', 'sim_pitch', 'SaveFormat', 'Structure With Time');

add_line(model_name, 'CPG_Gait_Phase_Generator/1', 'To_WS_Gait_Phases/1');
add_line(model_name, 'Foot_Trajectory_Generator/1', 'To_WS_Foot_Positions/1');
add_line(model_name, 'Gait_Balance_Interface/1', 'To_WS_Joint_Commands/1');
add_line(model_name, 'SISO_Balance_Controller/1', 'To_WS_Balance_Torque/1');
add_line(model_name, 'Pitch_Plant_Dynamics/1', 'To_WS_Pitch_Angle/1');

% Save model
target_file = fullfile(save_dir, [model_name, '.slx']);
save_system(model_name, target_file);
close_system(model_name);

fprintf('\n>>> Unified Simulink Model built and saved successfully to: %s\n\n', target_file);
end
