function results = run_project(varargin)
% RUN_PROJECT Master interactive and programmatic execution hub for Quadruped Robot.
%
% Options:
%   run_project()                 % Interactive scenario selector
%   run_project('pid')            % Model A: Baseline Discrete PI/PID
%   run_project('learning')       % Model B: Supervised LSTM vs Deep RL TD3
%   run_project('full')           % Model C: Complete 8-Phase Movement Integration
%   run_project('3d')             % Real-time 3D Conveyor & Obstacle Simulation
%   run_project('tests')          % Master Automated Test Suite
%
% Usage:
%   results = run_project('pid')

proj_root = fileparts(mfilename('fullpath'));
build_project();

if nargin < 1
    fprintf('\n========================================================================\n');
    fprintf('  QUADRUPED ROBOT GAIT AND BALANCE CONTROL: MASTER RUNNER\n');
    fprintf('========================================================================\n');
    fprintf('  Select an execution target:\n');
    fprintf('    1. Model A : Baseline Discrete PI/PID Simulation\n');
    fprintf('    2. Model B : Learning Controllers (LSTM & TD3) Benchmark\n');
    fprintf('    3. Model C : Complete 8-Phase Movement Integration\n');
    fprintf('    4. 3D Sim  : Interactive 3D Conveyor Belt & Obstacle Course\n');
    fprintf('    5. Tests   : Execute Master Regression Test Suite\n');
    fprintf('========================================================================\n');
    choice = input('  Enter choice [1-5] (default=1): ');
    if isempty(choice), choice = 1; end
    
    switch choice
        case 1, target = 'pid';
        case 2, target = 'learning';
        case 3, target = 'full';
        case 4, target = '3d';
        case 5, target = 'tests';
        otherwise, target = 'pid';
    end
else
    target = lower(varargin{1});
end

switch target
    case {'pid', 'model_a'}
        addpath(fullfile(proj_root, 'Simulink'));
        results = run_pid_case();
        
    case {'learning', 'lstm', 'td3', 'model_b'}
        addpath(fullfile(proj_root, 'Simulink'));
        results = run_lstm_td3_case();
        
    case {'full', 'movement', 'model_c'}
        addpath(fullfile(proj_root, 'Simulink'));
        results = run_full_movement_case();
        
    case {'3d', 'sim3d', 'gui'}
        addpath(fullfile(proj_root, 'Simulation_3D'));
        addpath(fullfile(proj_root, 'Simulation_3D', 'controllers'));
        results = create_3d_simulation_app();
        
    case {'test', 'tests', 'suite'}
        results = run_tests();
        
    otherwise
        error('Unknown target: %s. Valid choices: pid, learning, full, 3d, tests.', target);
end
end
