function metrics = compute_benchmark_metrics(sim_out, test_id)
% COMPUTE_BENCHMARK_METRICS Computes standard control-system performance metrics.
%
% Metrics:
%   - RMSE               : Root-Mean-Square Error [rad]
%   - IAE                : Integrated Absolute Error [rad.s]
%   - Peak_Deviation     : Maximum tracking absolute error [rad]
%   - Overshoot_pct      : Peak overshoot percentage [%] (for step transitions)
%   - Settling_Time_s    : 2% settling time [s]
%   - Control_Energy     : Integrated squared control torque [N^2.m^2.s]
%   - Peak_Torque        : Maximum absolute control effort [N.m]
%   - RMS_Torque         : RMS control effort [N.m]
%   - Recovery_Time_s    : Time to return within 0.005 rad (0.29 deg) after disturbance
%   - Mean_Inference_us  : Mean execution time per control step [microseconds]

if nargin < 2, test_id = 'generic'; end

t = sim_out.time;
dt = t(2) - t(1);
e = sim_out.error;
u = sim_out.u;
th = sim_out.theta;
th_d = sim_out.theta_d;

metrics.RMSE           = sqrt(mean(e.^2));
metrics.IAE            = sum(abs(e)) * dt;
metrics.Peak_Deviation = max(abs(e));
metrics.Control_Energy = sum(u.^2) * dt;
metrics.Peak_Torque    = max(abs(u));
metrics.RMS_Torque     = sqrt(mean(u.^2));
metrics.Mean_Inference_us = sim_out.mean_step_us;

% Test-specific metrics (Step Settling and Overshoot for Test 1)
switch lower(test_id)
    case 'test1'
        % Test 1 step occurs at t = 1.0 s to th_d = 0.087 rad (5 deg) until t = 3.0 s
        idx_step = find(t >= 1.0 & t < 3.0);
        t_sub = t(idx_step) - 1.0;
        th_sub = th(idx_step);
        target = 0.087266; % 5 degrees
        
        peak_val = max(th_sub);
        if peak_val > target
            metrics.Overshoot_pct = max(0, ((peak_val - target) / target) * 100);
        else
            metrics.Overshoot_pct = 0.0;
        end
        
        % Settling time: within 2% band of target (band: [0.98, 1.02]*target)
        band = 0.02 * target;
        outside = find(abs(th_sub - target) > band);
        if isempty(outside)
            metrics.Settling_Time_s = 0.0;
        elseif outside(end) == length(idx_step)
            metrics.Settling_Time_s = NaN; % Did not settle before next transition
        else
            metrics.Settling_Time_s = t_sub(outside(end) + 1);
        end
        metrics.Recovery_Time_s = NaN;
        
    case {'test2', 'test3'}
        % Disturbance tests: find recovery time after disturbance onset
        % Active disturbance interval
        dist_indices = find(abs(sim_out.tau_dist) > 0.01);
        metrics.Overshoot_pct = NaN;
        metrics.Settling_Time_s = NaN;
        
        if ~isempty(dist_indices)
            t_dist_start = t(dist_indices(1));
            t_dist_end   = t(dist_indices(end));
            
            % Tolerance recovery band: |theta| < 0.005 rad (~0.29 deg)
            rec_band = 0.005;
            post_dist_idx = find(t >= t_dist_end);
            unrecovered = find(abs(th(post_dist_idx)) > rec_band);
            if isempty(unrecovered)
                metrics.Recovery_Time_s = 0.0;
            elseif unrecovered(end) == length(post_dist_idx)
                metrics.Recovery_Time_s = NaN; % Did not recover before end of simulation
            else
                metrics.Recovery_Time_s = t(post_dist_idx(unrecovered(end) + 1)) - t_dist_start;
            end
        else
            metrics.Recovery_Time_s = 0.0;
        end
        
    otherwise
        metrics.Overshoot_pct = NaN;
        metrics.Settling_Time_s = NaN;
        metrics.Recovery_Time_s = NaN;
end
end
