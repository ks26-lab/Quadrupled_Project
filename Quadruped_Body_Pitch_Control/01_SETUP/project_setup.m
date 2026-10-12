function setup_info = project_setup(silent)
% PROJECT_SETUP Initializes environment paths and verifies folder structure.
%
% Responsibilities:
%   1. Determines the project root dynamically.
%   2. Adds all required project directories to MATLAB's path.
%   3. Verifies key folders exist.
%   4. Displays a concise setup confirmation.
%
% Restrictions:
%   - Does NOT modify any control logic.
%   - Does NOT modify model parameters.
%   - Does NOT run the project automatically.

if nargin < 1
    silent = false;
end

% 1 & 2. Add directories dynamically
paths = project_paths();
cfg = project_config();

% 3 & 4. Verify essential folders exist
required_folders = { ...
    paths.main, ...
    paths.setup, ...
    paths.model, ...
    paths.phases, ...
    paths.common, ...
    paths.simulink, ...
    paths.results, ...
    paths.data, ...
    paths.docs, ...
    paths.archive ...
};

missing_folders = {};
for i = 1:numel(required_folders)
    f = required_folders{i};
    if ~exist(f, 'dir')
        missing_folders{end+1} = f; %#ok<AGROW>
    end
end

if ~isempty(missing_folders)
    warning('project_setup:MissingFolders', ...
        'The following expected project folders were not found:\n%s', ...
        strjoin(missing_folders, '\n'));
end

setup_info.paths = paths;
setup_info.config = cfg;
setup_info.is_ready = isempty(missing_folders);

% 5. Display concise confirmation
if ~silent
    fprintf('\n============================================================\n');
    fprintf('  QUADRUPED ROBOT BODY-PITCH CONTROL PROJECT\n');
    fprintf('  Setup Initialized Successfully\n');
    fprintf('============================================================\n');
    fprintf('  Project Root : %s\n', paths.root);
    fprintf('  Version      : %s\n', cfg.version);
    fprintf('  Status       : All paths loaded into MATLAB environment.\n');
    fprintf('============================================================\n\n');
end

end
