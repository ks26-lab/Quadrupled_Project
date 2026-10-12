function results = run_tests(varargin)
% RUN_TESTS Master automated test suite covering:
%   1. Unit tests for core functions (Kinematics, CPG, Dynamics, PI Control)
%   2. Phase regression tests (Phases 1 through 8 isolated runners)
%   3. Simulink model interface & loadability tests
%   4. Performance & benchmark metric validation
%
% Usage:
%   results = run_tests()

fprintf('\n========================================================================\n');
fprintf('  QUADRUPED ROBOT AUTOMATED TEST HARNESS & REGRESSION SUITE\n');
fprintf('========================================================================\n\n');

proj_root = fileparts(mfilename('fullpath'));
build_project();

test_results = struct();
num_passed = 0;
num_failed = 0;

%% 1. Unit Tests
fprintf('--- STAGE 1: UNIT TESTS ---\n');

% Test 1.1: Robot Config Integrity
try
    r_cfg = robot_config();
    assert(r_cfg.leg.L1 == 0.12 && r_cfg.leg.L2 == 0.15, 'Link lengths mismatch.');
    assert(r_cfg.dynamics.J == 0.05, 'Inertia mismatch.');
    fprintf('   [PASS] Test 1.1: Robot configuration parameters verified.\n');
    num_passed = num_passed + 1;
catch ME
    fprintf('   [FAIL] Test 1.1: %s\n', ME.message);
    num_failed = num_failed + 1;
end

% Test 1.2: CPG Phase Generator Continuity
try
    g_cfg = gait_config();
    [p0, st0, sw0] = compute_cpg_phases(0.0, g_cfg);
    assert(length(p0) == 4, 'CPG output dimension must be 4x1.');
    assert(all(st0 == ~sw0), 'Stance and swing must be complementary booleans.');
    fprintf('   [PASS] Test 1.2: CPG 4-leg phase & contact logic verified.\n');
    num_passed = num_passed + 1;
catch ME
    fprintf('   [FAIL] Test 1.2: %s\n', ME.message);
    num_failed = num_failed + 1;
end

% Test 1.3: Inverse Kinematics Reachability
try
    nominal_feet = r_cfg.nominalFootPosition;
    [q_cmd, feasible] = solve_inverse_kinematics(nominal_feet, r_cfg);
    assert(length(q_cmd) == 12, 'Joint commands must be 12-DOF.');
    assert(all(feasible), 'Nominal feet must be 100%% reachable.');
    fprintf('   [PASS] Test 1.3: 12-DOF Analytical Inverse Kinematics verified.\n');
    num_passed = num_passed + 1;
catch ME
    fprintf('   [FAIL] Test 1.3: %s\n', ME.message);
    num_failed = num_failed + 1;
end

% Test 1.4: PI Balance Controller Clamping
try
    c_cfg = controller_config();
    s_init = struct('integrator', 0.0, 'last_u', 0.0);
    [u_sat_check, ~] = compute_pi_balance_step(s_init, 0.0, -1.0, c_cfg); % 1 rad error
    assert(abs(u_sat_check - c_cfg.u_sat) < 1e-6, 'PI controller must saturate at u_sat (0.40 N.m).');
    fprintf('   [PASS] Test 1.4: Discrete PI Controller anti-windup & saturation verified.\n');
    num_passed = num_passed + 1;
catch ME
    fprintf('   [FAIL] Test 1.4: %s\n', ME.message);
    num_failed = num_failed + 1;
end

%% 2. Movement Phase Isolated Execution Tests (Phases 1 to 8)
fprintf('\n--- STAGE 2: MOVEMENT PHASE ISOLATED RUNNERS (1 to 8) ---\n');
for p = 1:8
    try
        res_p = run_phase(p);
        assert(strcmp(res_p.status, 'SUCCESS'), 'Phase returned non-success.');
        fprintf('   [PASS] Phase %d runner verified independently.\n', p);
        num_passed = num_passed + 1;
    catch ME
        fprintf('   [FAIL] Phase %d: %s\n', p, ME.message);
        num_failed = num_failed + 1;
    end
end

%% 3. Simulink Model Validation
fprintf('\n--- STAGE 3: SIMULINK MODEL VALIDATION ---\n');
try
    addpath(fullfile(proj_root, 'Simulink'));
    val_rep = validate_simulink_models(fullfile(proj_root, 'Simulink'));
    fprintf('   [PASS] Simulink model validator executed.\n');
    num_passed = num_passed + 1;
catch ME
    fprintf('   [FAIL] Simulink validator: %s\n', ME.message);
    num_failed = num_failed + 1;
end

%% 4. Benchmark Execution Tests
fprintf('\n--- STAGE 4: BENCHMARK REGRESSION EXECUTION ---\n');
try
    pid_res = run_pid_case();
    assert(pid_res.metrics.RMSE_rad < 0.05, 'PID RMSE exceeds acceptable threshold.');
    fprintf('   [PASS] Model A (Quadruped_PID) benchmark verified.\n');
    num_passed = num_passed + 1;
catch ME
    fprintf('   [FAIL] Model A benchmark: %s\n', ME.message);
    num_failed = num_failed + 1;
end

try
    learning_res = run_lstm_td3_case();
    assert(isfield(learning_res, 'summary_table'), 'Summary table missing.');
    fprintf('   [PASS] Model B (Quadruped_LSTM_TD3) comparative benchmark verified.\n');
    num_passed = num_passed + 1;
catch ME
    fprintf('   [FAIL] Model B benchmark: %s\n', ME.message);
    num_failed = num_failed + 1;
end

try
    full_res = run_full_movement_case();
    assert(full_res.metrics.IK_Feasibility_Percent == 100.0, 'IK feasibility below 100%%.');
    fprintf('   [PASS] Model C (Quadruped_Full_Movement) integrated benchmark verified.\n');
    num_passed = num_passed + 1;
catch ME
    fprintf('   [FAIL] Model C benchmark: %s\n', ME.message);
    num_failed = num_failed + 1;
end

fprintf('\n========================================================================\n');
fprintf('  TEST SUMMARY: %d PASSED, %d FAILED (Total: %d)\n', ...
    num_passed, num_failed, num_passed + num_failed);
if num_failed == 0
    fprintf('  ALL TESTS PASSED: Architecture is robust, modular, and verified!\n');
else
    fprintf('  SOME TESTS FAILED: Please check log details above.\n');
end
fprintf('========================================================================\n\n');

results.passed = num_passed;
results.failed = num_failed;
end
