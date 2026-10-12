% SIMULINK_INIT Pre-simulation parameter and workspace initialization script.
%
% Prepares the base workspace with physical quadruped parameters, state-space
% plant matrices, and controller gains for Simulink model callbacks or InitFcn.

% 1. Ensure project paths are active
if ~exist('project_paths', 'file')
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', '01_SETUP'));
end
project_setup(true);

% 2. Load physical parameters
robot_parameters;

% 3. Initialize plant dynamics
pitch_plant;

% 4. Load frozen controller gains if available
processed_path = fullfile(fileparts(mfilename('fullpath')), '..', '..', '07_DATA', 'processed', 'frozen_pi_controller.mat');
if exist(processed_path, 'file')
    load(processed_path);
    fprintf('[Simulink Init] Loaded frozen PI controller: Kp=%.4f, Ki=%.4f\n', Kp_final, Ki_final);
else
    % Default fallback to Phase 4 baseline if .mat has not been saved yet
    Kp_final = 0.50;
    Ki_final = 0.17;
    C_final = Kp_final + Ki_final/s;
    T_final = feedback(C_final * G, 1);
end

fprintf('[Simulink Init] Base workspace initialized successfully for Simulink execution.\n');
