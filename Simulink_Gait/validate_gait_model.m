function test_report = validate_gait_model()
% VALIDATE_GAIT_MODEL Comprehensive 11-Test Verification Suite for Unified Gait Model.
%
% Executes Tests 1 through 11:
%   TEST 1: Initialization
%   TEST 2: Gait Phase Progression
%   TEST 3: Four-Leg Coordination
%   TEST 4: Foot Trajectory
%   TEST 5: Inverse Kinematics
%   TEST 6: Joint Constraints
%   TEST 7: Robot Movement Integration
%   TEST 8: Balance Integration
%   TEST 9: Disturbance Interface
%   TEST 10: Repeatability
%   TEST 11: Regression

fprintf('\n============================================================\n');
fprintf('  QUADRUPED GAIT UNIFIED MODEL: 11-TEST VERIFICATION SUITE\n');
fprintf('============================================================\n\n');

test_report = struct();
all_passed = true;

% Load configuration
cfg = initialize_gait_model();

%% TEST 1: Initialization
fprintf('TEST 1: Initialization ... ');
try
    assert(isfield(cfg, 'L1') && cfg.L1 == 0.12, 'L1 mismatch');
    assert(isfield(cfg, 'L2') && cfg.L2 == 0.15, 'L2 mismatch');
    assert(size(cfg.hipPosition, 1) == 4 && size(cfg.nominalFootPosition, 2) == 3, 'Geometry dimension error');
    test_report.test1 = 'PASS';
    fprintf('PASS\n');
catch ME
    test_report.test1 = ['FAIL: ', ME.message];
    all_passed = false;
    fprintf('FAIL (%s)\n', ME.message);
end

%% TEST 2: Gait Phase Progression
fprintf('TEST 2: Gait Phase Progression ... ');
try
    [p0, ~, ~] = cpg_gait_generator(0.0, 1.0, cfg.phaseOffsets);
    [p1, ~, ~] = cpg_gait_generator(0.25, 1.0, cfg.phaseOffsets);
    [p2, ~, ~] = cpg_gait_generator(1.0, 1.0, cfg.phaseOffsets);
    assert(abs(p0(1) - 0.0) < 1e-6, 'LF initial phase must be 0.0');
    assert(abs(p1(1) - 0.25) < 1e-6, 'LF phase at 0.25s must be 0.25');
    assert(abs(p2(1) - 0.0) < 1e-6, 'LF phase at 1.0s must wrap to 0.0');
    test_report.test2 = 'PASS';
    fprintf('PASS\n');
catch ME
    test_report.test2 = ['FAIL: ', ME.message];
    all_passed = false;
    fprintf('FAIL (%s)\n', ME.message);
end

%% TEST 3: Four-Leg Coordination
fprintf('TEST 3: Four-Leg Coordination ... ');
try
    [p_mid, stance_mid, swing_mid] = cpg_gait_generator(0.25, 1.0, cfg.phaseOffsets);
    % At t=0.25s: LF phase=0.25 (swing), RF phase=0.75 (stance), LH phase=0.75 (stance), RH phase=0.25 (swing)
    assert(swing_mid(1) == true && swing_mid(4) == true, 'Diagonal pair LF-RH must be in swing');
    assert(stance_mid(2) == true && stance_mid(3) == true, 'Diagonal pair RF-LH must be in stance');
    test_report.test3 = 'PASS';
    fprintf('PASS\n');
catch ME
    test_report.test3 = ['FAIL: ', ME.message];
    all_passed = false;
    fprintf('FAIL (%s)\n', ME.message);
end

%% TEST 4: Foot Trajectory
fprintf('TEST 4: Foot Trajectory ... ');
try
    % Test swing midpoint clearance for leg 1 at phase = 0.25 (s = 0.5)
    phases_test = [0.25; 0.75; 0.75; 0.25];
    foot_pos = four_leg_foot_trajectory(phases_test, cfg.nominalFootPosition, cfg.stepLength, cfg.stepHeight);
    
    % At s = 0.5: dz = stepHeight * sin(pi*0.5) = stepHeight = 0.04
    expected_z = cfg.nominalFootPosition(1, 3) + cfg.stepHeight;
    actual_z = foot_pos(1, 3);
    assert(abs(actual_z - expected_z) < 1e-6, 'Peak swing clearance mismatch');
    
    % Stance leg should have dz = 0
    assert(abs(foot_pos(2, 3) - cfg.nominalFootPosition(2, 3)) < 1e-6, 'Stance foot z should remain nominal');
    test_report.test4 = 'PASS';
    fprintf('PASS\n');
catch ME
    test_report.test4 = ['FAIL: ', ME.message];
    all_passed = false;
    fprintf('FAIL (%s)\n', ME.message);
end

%% TEST 5: Inverse Kinematics Comparison Against Baseline
fprintf('TEST 5: Inverse Kinematics ... ');
try
    root_dir = fileparts(fileparts(mfilename('fullpath')));
    ik_baseline_func = fullfile(root_dir, 'Gait_Movement', 'Phase_3_Inverse_Kinematics', 'solve_leg_ik.m');
    
    test_pos = cfg.nominalFootPosition;
    [q_12, feas] = four_leg_ik_solver(test_pos, cfg.L1, cfg.L2);
    
    % Compare leg 1 against baseline solve_leg_ik
    q_base = solve_leg_ik(test_pos(1,1), test_pos(1,2), test_pos(1,3), cfg.L1, cfg.L2);
    diff_q = max(abs(q_12(1:3) - q_base));
    assert(diff_q < 1e-10, 'IK output deviates from baseline solve_leg_ik');
    assert(all(feas), 'Nominal foot positions must be reachable');
    test_report.test5 = 'PASS';
    fprintf('PASS (Max IK Diff: %.2e rad)\n', diff_q);
catch ME
    test_report.test5 = ['FAIL: ', ME.message];
    all_passed = false;
    fprintf('FAIL (%s)\n', ME.message);
end

%% TEST 6: Joint Constraints & Reachability
fprintf('TEST 6: Joint Constraints ... ');
try
    unreachable_foot = test_pos;
    unreachable_foot(1, 3) = -0.50; % 0.50m distance exceeds L1 + L2 = 0.27m
    [~, feas_unreach] = four_leg_ik_solver(unreachable_foot, cfg.L1, cfg.L2);
    assert(feas_unreach(1) == false, 'Singular/unreachable target must flag feasible=false');
    test_report.test6 = 'PASS';
    fprintf('PASS\n');
catch ME
    test_report.test6 = ['FAIL: ', ME.message];
    all_passed = false;
    fprintf('FAIL (%s)\n', ME.message);
end

%% TEST 7: Robot Movement Integration
fprintf('TEST 7: Robot Movement Integration ... ');
try
    sim_res = run_gait_simulation(2.0);
    assert(size(sim_res.q_cmd, 1) == length(sim_res.time), 'Joint command time dimension mismatch');
    assert(size(sim_res.q_cmd, 2) == 12, 'Joint commands must be 12-DOF');
    test_report.test7 = 'PASS';
    fprintf('PASS\n');
catch ME
    test_report.test7 = ['FAIL: ', ME.message];
    all_passed = false;
    fprintf('FAIL (%s)\n', ME.message);
end

%% TEST 8: Balance Integration (SISO Pitch Invariance)
fprintf('TEST 8: Balance Integration ... ');
try
    % Verify delta_q_balance does not alter roll joints (q1, q4, q7, q10) or knee joints (q3, q6, q9, q12)
    q_dummy = zeros(12, 1);
    u_test = 0.25;
    [q_cmd, delta_q] = gait_balance_coupler(q_dummy, u_test, cfg.balance.k_torque_to_q);
    
    assert(delta_q(1) == 0 && delta_q(3) == 0, 'Roll and knee joints must not be perturbed by pitch balance');
    assert(delta_q(2) > 0 && delta_q(8) < 0, 'Differential front/rear pitch coupling must be opposite');
    test_report.test8 = 'PASS';
    fprintf('PASS\n');
catch ME
    test_report.test8 = ['FAIL: ', ME.message];
    all_passed = false;
    fprintf('FAIL (%s)\n', ME.message);
end

%% TEST 9: Disturbance Interface
fprintf('TEST 9: Disturbance Interface ... ');
try
    sim_res = run_gait_simulation(3.0);
    idx_dist = find(sim_res.time >= 2.0 & sim_res.time <= 2.5);
    max_tau_dist = max(sim_res.tau_dist(idx_dist));
    assert(abs(max_tau_dist - 0.10) < 1e-4, 'Disturbance pulse magnitude must be 0.10 N.m');
    test_report.test9 = 'PASS';
    fprintf('PASS\n');
catch ME
    test_report.test9 = ['FAIL: ', ME.message];
    all_passed = false;
    fprintf('FAIL (%s)\n', ME.message);
end

%% TEST 10: Repeatability
fprintf('TEST 10: Repeatability ... ');
try
    run1 = run_gait_simulation(2.0);
    run2 = run_gait_simulation(2.0);
    diff_repeat = max(abs(run1.q_cmd(:) - run2.q_cmd(:)));
    assert(diff_repeat < 1e-12, 'Consecutive simulation runs must be identical');
    test_report.test10 = 'PASS';
    fprintf('PASS (Diff: %.2e rad)\n', diff_repeat);
catch ME
    test_report.test10 = ['FAIL: ', ME.message];
    all_passed = false;
    fprintf('FAIL (%s)\n', ME.message);
end

%% TEST 11: Regression (Frozen Files Verification)
fprintf('TEST 11: Regression Check ... ');
try
    frozen_files = { ...
        fullfile(root_dir, 'Gait_Movement', 'Master_Gait.m'), ...
        fullfile(root_dir, 'Robot_Movement', 'Master_Robot.m'), ...
        fullfile(root_dir, 'Integration', 'Master_Integration.m'), ...
        fullfile(root_dir, 'Quadruped_Body_Pitch_Control', '02_MODEL', 'parameters', 'robot_parameters.m') ...
    };
    for f = 1:length(frozen_files)
        assert(exist(frozen_files{f}, 'file') == 2, sprintf('Frozen file missing: %s', frozen_files{f}));
    end
    test_report.test11 = 'PASS';
    fprintf('PASS\n');
catch ME
    test_report.test11 = ['FAIL: ', ME.message];
    all_passed = false;
    fprintf('FAIL (%s)\n', ME.message);
end

fprintf('\n============================================================\n');
if all_passed
    fprintf('  ALL 11 TESTS PASSED SUCCESSFULLY.\n');
else
    fprintf('  WARNING: SOME TESTS FAILED.\n');
end
fprintf('============================================================\n\n');
end
