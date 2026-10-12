function run_project(phase_choice)
% RUN_PROJECT Main interactive and programmatic entry launcher for the project.
%
% Responsibilities:
%   1. Locates project root dynamically.
%   2. Runs project_setup to ensure all directories are on MATLAB search path.
%   3. Displays available phases and options.
%   4. Executes ONLY the selected phase without forcing manual execution of prior phases.
%
% Usage:
%   run_project()         % Interactive menu prompt
%   run_project('1')      % Run Phase 1
%   run_project('2')      % Run Phase 2
%   run_project('3')      % Run Phase 3
%   run_project('4')      % Run Phase 4
%   run_project('4_5')    % Run Phase 4.5
%   run_project('all')    % Run Phases 1 through 4.5 sequentially

% 1. Determine project root and initialize paths
this_dir = fileparts(mfilename('fullpath'));
project_root = fileparts(this_dir);
setup_dir = fullfile(project_root, '01_SETUP');

if exist(fullfile(setup_dir, 'project_setup.m'), 'file')
    addpath(setup_dir);
    project_setup(false);
end

% 2. Check if phase choice was supplied
if nargin < 1 || isempty(phase_choice)
    fprintf('\n============================================================\n');
    fprintf('  QUADRUPED ROBOT BODY-PITCH CONTROL: EXECUTION MENU\n');
    fprintf('============================================================\n');
    fprintf('  [1] Phase 1   : Physical Model & Initial Response\n');
    fprintf('  [2] Phase 2   : SISO Plant & Open-Loop Analysis\n');
    fprintf('  [3] Phase 3   : Proportional (P) Feedback Controller\n');
    fprintf('  [4] Phase 4   : PI Controller Design & Pareto Tuning\n');
    fprintf('  [5] Phase 4.5 : Disturbance Rejection Validation\n');
    fprintf('  [6] Phase 5   : Hardware-Realistic SISO Simulation\n');
    fprintf('  [7] Phase 6   : One-Leg Contact-Loss / Fault Analysis\n');
    fprintf('  [8] Phase 7   : Robustness & Model Uncertainty Analysis\n');
    fprintf('  [9] Phase 8   : Learning Control Benchmark (PI vs LSTM vs TD3)\n');
    fprintf('  [10] Master 1-3: Run Master Script (Phases 1-3)\n');
    fprintf('  [11] Master 4-4.5: Run Master Script (Phases 4-4.5)\n');
    fprintf('  [A] All       : Run All Implemented Phases (1 to 8)\n');
    fprintf('  [Q] Quit\n');
    fprintf('============================================================\n');
    
    try
        choice = input('Select an option [1-11, A, Q]: ', 's');
    catch
        choice = '1';
    end
    if isempty(choice)
        choice = '1';
    end
else
    choice = char(string(phase_choice));
end

% 3. Dispatch to selected phase
switch lower(strtrim(choice))
    case {'1', 'phase1', 'phase_1'}
        run_phase_1();
        
    case {'2', 'phase2', 'phase_2'}
        run_phase_2();
        
    case {'3', 'phase3', 'phase_3'}
        run_phase_3();
        
    case {'4', 'phase4', 'phase_4'}
        run_phase_4();
        
    case {'4_5', '4.5', 'phase4_5', 'phase_4_5'}
        run_phase_4_5();
        
    case {'5', 'phase5', 'phase_5'}
        run_phase_5();
        
    case {'6', 'phase6', 'phase_6', '4_6', '4.6', 'phase4_6', 'phase_4_6'}
        run_phase_4_6();
        
    case {'7', 'phase7', 'phase_7', '4_7', '4.7', 'phase4_7', 'phase_4_7'}
        run_phase_4_7();
        
    case {'8', 'phase8', 'phase_8'}
        run_phase_8();
        
    case {'9', 'master1'}
        fprintf('\n>>> Executing Master Script (Phases 1-3)...\n');
        main_phase1_to_3;
        
    case {'10', 'master4'}
        fprintf('\n>>> Executing Master Script (Phases 4-4.5)...\n');
        main_phase4_to_4_5;
        
    case {'a', 'all'}
        fprintf('\n>>> Executing all implemented phases in sequence (Phases 1 to 8)...\n');
        run_phase_1();
        run_phase_2();
        run_phase_3();
        run_phase_4();
        run_phase_4_5();
        run_phase_5();
        run_phase_4_6();
        run_phase_4_7();
        run_phase_8();
        fprintf('\n>>> All phases completed successfully.\n');
        
    case {'q', 'quit', 'exit'}
        fprintf('Exited launcher without running any phase.\n');
        
    otherwise
        warning('run_project:InvalidChoice', 'Invalid choice "%s". Please select 1-10 or A.', choice);
end

end
