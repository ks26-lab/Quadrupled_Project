function app = create_3d_simulation_app()
% CREATE_3D_SIMULATION_APP Real-time 3D Quadruped Simulation & Interactive Dashboard.
%
% Features:
%   - 3D rendering of Quadruped Robot (Trunk, Hips, Thighs, Shanks, Feet)
%   - Physical moving conveyor belt with dynamic chevron striping
%   - Interactive obstacle course (step platform, ramp, block, gap)
%   - Live telemetry charts (Body pitch, balance torque, foot clearance)
%   - Interactive control dashboard (Conveyor, Gait, Obstacles, Scenarios, Disturbances)
%
% Usage:
%   app = create_3d_simulation_app();

sim_dir = fileparts(mfilename('fullpath'));
proj_root = fileparts(sim_dir);
addpath(fullfile(sim_dir, 'controllers'));

% Initialize Engine
simEngine = init_3d_simulation();

% Create UI Figure
fig = figure('Name', 'Quadruped Robot Real-Time 3D Gait & Conveyor Simulation', ...
    'Color', [0.12, 0.12, 0.14], 'Position', [50, 50, 1400, 850], ...
    'MenuBar', 'none', 'NumberTitle', 'off', 'CloseRequestFcn', @onClose);

% 1. Main 3D Simulation Viewport
ax3d = axes('Parent', fig, 'Position', [0.03, 0.32, 0.65, 0.64], ...
    'Color', [0.15, 0.15, 0.18], 'XColor', [0.7, 0.7, 0.7], ...
    'YColor', [0.7, 0.7, 0.7], 'ZColor', [0.7, 0.7, 0.7]);
grid(ax3d, 'on');
view(ax3d, 40, 25);
axis(ax3d, 'equal');
xlim(ax3d, [-0.5, 3.5]);
ylim(ax3d, [-0.8, 0.8]);
zlim(ax3d, [0.0, 0.75]);
xlabel(ax3d, 'X - Stride / Conveyor (m)');
ylabel(ax3d, 'Y - Lateral (m)');
zlabel(ax3d, 'Z - Vertical (m)');
title(ax3d, 'Quadruped Robot on Interactive Conveyor Belt & Obstacle Course', ...
    'Color', [0.9, 0.9, 0.95], 'FontSize', 12, 'FontWeight', 'bold');
hold(ax3d, 'on');

% 2. Telemetry Plots
% Pitch Response Plot
ax_pitch = axes('Parent', fig, 'Position', [0.72, 0.67, 0.25, 0.28], ...
    'Color', [0.15, 0.15, 0.18], 'XColor', [0.7, 0.7, 0.7], 'YColor', [0.7, 0.7, 0.7]);
grid(ax_pitch, 'on');
hold(ax_pitch, 'on');
line_pitch = plot(ax_pitch, 0, 0, 'c-', 'LineWidth', 1.8);
line_ref   = yline(ax_pitch, 0, 'r--', 'LineWidth', 1.2);
ylabel(ax_pitch, 'Pitch \theta (deg)', 'Color', [0.8, 0.8, 0.8]);
title(ax_pitch, 'IMU Pitch Telemetry vs Reference', 'Color', [0.9, 0.9, 0.9]);

% Balance Torque Plot
ax_torque = axes('Parent', fig, 'Position', [0.72, 0.35, 0.25, 0.25], ...
    'Color', [0.15, 0.15, 0.18], 'XColor', [0.7, 0.7, 0.7], 'YColor', [0.7, 0.7, 0.7]);
grid(ax_torque, 'on');
hold(ax_torque, 'on');
line_u = plot(ax_torque, 0, 0, 'm-', 'LineWidth', 1.6);
yline(ax_torque, 0.40, 'k:'); yline(ax_torque, -0.40, 'k:');
xlabel(ax_torque, 'Time (s)', 'Color', [0.8, 0.8, 0.8]);
ylabel(ax_torque, 'Torque u (N\cdotm)', 'Color', [0.8, 0.8, 0.8]);
title(ax_torque, 'SISO Balance Effort (Sat \pm 0.40)', 'Color', [0.9, 0.9, 0.9]);

% 3. Interactive Control Dashboard Panel
pnl_ctrl = uipanel('Parent', fig, 'Position', [0.03, 0.02, 0.94, 0.28], ...
    'BackgroundColor', [0.18, 0.18, 0.22], 'ForegroundColor', [0.8, 0.8, 0.9], ...
    'Title', 'INTERACTIVE CONTROL DASHBOARD & SCENARIO SELECTOR', 'FontSize', 10, 'FontWeight', 'bold');

% Button Controls
btn_play = uicontrol('Parent', pnl_ctrl, 'Style', 'pushbutton', 'String', 'PAUSE', ...
    'Position', [15, 120, 80, 32], 'BackgroundColor', [0.2, 0.6, 0.4], ...
    'ForegroundColor', 'white', 'FontWeight', 'bold', 'Callback', @onTogglePlay);

btn_reset = uicontrol('Parent', pnl_ctrl, 'Style', 'pushbutton', 'String', 'RESET', ...
    'Position', [105, 120, 80, 32], 'BackgroundColor', [0.7, 0.3, 0.2], ...
    'ForegroundColor', 'white', 'FontWeight', 'bold', 'Callback', @onReset);

% Conveyor Controls
uicontrol('Parent', pnl_ctrl, 'Style', 'text', 'String', 'Conveyor Speed (m/s):', ...
    'Position', [200, 130, 130, 20], 'BackgroundColor', [0.18, 0.18, 0.22], 'ForegroundColor', 'white');
sld_speed = uicontrol('Parent', pnl_ctrl, 'Style', 'slider', 'Min', -0.30, 'Max', 0.30, ...
    'Value', 0.10, 'Position', [340, 130, 120, 20], 'Callback', @onConveyorSpeed);
lbl_speed = uicontrol('Parent', pnl_ctrl, 'Style', 'text', 'String', '0.10 m/s', ...
    'Position', [470, 130, 60, 20], 'BackgroundColor', [0.18, 0.18, 0.22], 'ForegroundColor', 'yellow');

% Gait Speed Control
uicontrol('Parent', pnl_ctrl, 'Style', 'text', 'String', 'Gait Cadence (Hz):', ...
    'Position', [200, 95, 130, 20], 'BackgroundColor', [0.18, 0.18, 0.22], 'ForegroundColor', 'white');
sld_freq = uicontrol('Parent', pnl_ctrl, 'Style', 'slider', 'Min', 0.5, 'Max', 2.0, ...
    'Value', 1.0, 'Position', [340, 95, 120, 20], 'Callback', @onGaitFreq);
lbl_freq = uicontrol('Parent', pnl_ctrl, 'Style', 'text', 'String', '1.0 Hz', ...
    'Position', [470, 95, 60, 20], 'BackgroundColor', [0.18, 0.18, 0.22], 'ForegroundColor', 'yellow');

% Step Height Control
uicontrol('Parent', pnl_ctrl, 'Style', 'text', 'String', 'Step Height (m):', ...
    'Position', [200, 60, 130, 20], 'BackgroundColor', [0.18, 0.18, 0.22], 'ForegroundColor', 'white');
sld_ht = uicontrol('Parent', pnl_ctrl, 'Style', 'slider', 'Min', 0.02, 'Max', 0.08, ...
    'Value', 0.04, 'Position', [340, 60, 120, 20], 'Callback', @onStepHeight);
lbl_ht = uicontrol('Parent', pnl_ctrl, 'Style', 'text', 'String', '0.04 m', ...
    'Position', [470, 60, 60, 20], 'BackgroundColor', [0.18, 0.18, 0.22], 'ForegroundColor', 'yellow');

% Scenario Selector
uicontrol('Parent', pnl_ctrl, 'Style', 'text', 'String', 'Select Scenario:', ...
    'Position', [550, 130, 100, 20], 'BackgroundColor', [0.18, 0.18, 0.22], 'ForegroundColor', 'cyan', 'FontWeight', 'bold');
pop_scen = uicontrol('Parent', pnl_ctrl, 'Style', 'popupmenu', ...
    'String', {'Scenario 1: Nominal Gait', ...
               'Scenario 2: Moving Conveyor (Forward/Reverse)', ...
               'Scenario 3: Obstacle Traversal', ...
               'Scenario 4: Uneven Terrain & Ramp', ...
               'Scenario 5: Disturbance Rejection Pulse', ...
               'Scenario 6: Right-Front Contact Loss Fault'}, ...
    'Position', [660, 130, 280, 22], 'Callback', @onSelectScenario);

% Disturbance & Contact-Loss Pushbuttons
btn_dist = uicontrol('Parent', pnl_ctrl, 'Style', 'pushbutton', 'String', 'INJECT 0.10 N.m DISTURBANCE', ...
    'Position', [550, 75, 200, 32], 'BackgroundColor', [0.7, 0.5, 0.1], ...
    'ForegroundColor', 'white', 'FontWeight', 'bold', 'Callback', @onInjectDisturbance);

btn_fault = uicontrol('Parent', pnl_ctrl, 'Style', 'pushbutton', 'String', 'TOGGLE RF CONTACT LOSS', ...
    'Position', [765, 75, 175, 32], 'BackgroundColor', [0.7, 0.2, 0.3], ...
    'ForegroundColor', 'white', 'FontWeight', 'bold', 'Callback', @onToggleContactLoss);

% Live Status Telemetry
lbl_status = uicontrol('Parent', pnl_ctrl, 'Style', 'text', ...
    'String', 'STATUS: System Nominal | Running 200 Hz Locomotion Engine | Conveyor Active', ...
    'Position', [20, 15, 920, 25], 'BackgroundColor', [0.15, 0.15, 0.18], ...
    'ForegroundColor', [0.3, 0.9, 0.5], 'FontSize', 10, 'FontWeight', 'bold');

% 4. Render 3D World Geometry Objects
% Conveyor Frame
patch(ax3d, [-0.5, 3.5, 3.5, -0.5], [-0.5, -0.5, 0.5, 0.5], [0.0, 0.0, 0.0, 0.0], ...
    [0.2, 0.2, 0.25], 'FaceAlpha', 0.9, 'EdgeColor', [0.3, 0.3, 0.4]);
% Conveyor Moving Surface
h_belt = patch(ax3d, [-0.4, 3.4, 3.4, -0.4], [-0.45, -0.45, 0.45, 0.45], ...
    [0.20, 0.20, 0.20, 0.20], [0.12, 0.12, 0.15], 'EdgeColor', [0.4, 0.4, 0.5]);

% Render Conveyor Stripes (Chevron markers to visualize motion)
num_stripes = 10;
h_stripes = gobjects(num_stripes, 1);
for s = 1:num_stripes
    xs = -0.3 + (s - 1) * 0.40;
    h_stripes(s) = patch(ax3d, [xs, xs+0.05, xs+0.05, xs], [-0.4, -0.4, 0.4, 0.4], ...
        [0.201, 0.201, 0.201, 0.201], [0.8, 0.7, 0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.7);
end

% Render Obstacles
num_obs = length(simEngine.obstacles);
h_obs = gobjects(num_obs, 1);
for ob = 1:num_obs
    o = simEngine.obstacles(ob);
    ox = o.pos(1); oy = o.pos(2); oz = o.pos(3);
    dx = o.size(1); dy = o.size(2); dz = o.size(3);
    
    if strcmp(o.type, 'ramp')
        % Triangular/Wedge prism
        X = [ox-dx/2, ox+dx/2, ox+dx/2, ox-dx/2; ox-dx/2, ox+dx/2, ox+dx/2, ox-dx/2];
        Y = [-dy/2, -dy/2, dy/2, dy/2; -dy/2, -dy/2, dy/2, dy/2];
        Z = [0.20, 0.20+dz, 0.20+dz, 0.20; 0.20, 0.20, 0.20, 0.20];
        h_obs(ob) = surf(ax3d, X, Y, Z, 'FaceColor', o.color, 'EdgeColor', 'none', 'FaceAlpha', 0.85);
    else
        % Box
        h_obs(ob) = patch(ax3d, [ox-dx/2, ox+dx/2, ox+dx/2, ox-dx/2], ...
                                [oy-dy/2, oy-dy/2, oy+dy/2, oy+dy/2], ...
                                [oz+dz/2, oz+dz/2, oz+dz/2, oz+dz/2], o.color, 'EdgeColor', 'k', 'FaceAlpha', 0.9);
    end
end

% Render Quadruped Robot Body
h_body = patch(ax3d, zeros(1, 4), zeros(1, 4), zeros(1, 4), [0.25, 0.45, 0.85], ...
    'EdgeColor', [0.1, 0.8, 0.9], 'LineWidth', 1.5);
h_imu = plot3(ax3d, 0, 0, 0, 'yo', 'MarkerSize', 8, 'MarkerFaceColor', 'y');

% Render Legs (LF, RF, LH, RH): Hip -> Thigh -> Knee -> Shank -> Foot
leg_colors = {[0.2, 0.8, 0.9], [0.9, 0.4, 0.2], [0.2, 0.8, 0.5], [0.9, 0.8, 0.2]};
h_legs = gobjects(4, 1);
h_feet = gobjects(4, 1);
for leg = 1:4
    h_legs(leg) = plot3(ax3d, [0, 0, 0], [0, 0, 0], [0, 0, 0], 'Color', leg_colors{leg}, 'LineWidth', 3.5);
    h_feet(leg) = plot3(ax3d, 0, 0, 0, 'ro', 'MarkerSize', 7, 'MarkerFaceColor', 'r');
end

% 5. Timer for Real-Time Execution Loop
sim_timer = timer('ExecutionMode', 'fixedRate', 'Period', 0.025, ... % 40 Hz visual frame rate
    'TimerFcn', @onTimerTick);

start(sim_timer);

% Store Handle Structure
app.fig = fig;
app.simEngine = simEngine;
app.sim_timer = sim_timer;

%% ==================== CALLBACKS ==================== %%

    function onTimerTick(~, ~)
        if ~isvalid(fig), return; end
        
        % Advance physics sub-steps (5 sub-steps of 5ms = 25ms per visual frame)
        for sub = 1:5
            simEngine = step_3d_simulation(simEngine);
        end
        
        % Update Robot Visual Representation
        pos = simEngine.state.pos;
        pitch = simEngine.state.rpy(2);
        L = simEngine.robot.body_length / 2;
        W = simEngine.robot.body_width / 2;
        H = simEngine.robot.body_height / 2;
        
        R = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
        
        % Body Box Top vertices
        v_top = [ L,  W, H;
                 -L,  W, H;
                 -L, -W, H;
                  L, -W, H]';
        v_top_world = pos + R * v_top;
        set(h_body, 'XData', v_top_world(1, :), 'YData', v_top_world(2, :), 'ZData', v_top_world(3, :));
        
        % IMU Marker
        imu_world = pos + R * [0; 0; H + 0.01];
        set(h_imu, 'XData', imu_world(1), 'YData', imu_world(2), 'ZData', imu_world(3));
        
        % Update 4 Legs
        for l = 1:4
            hip_rel = simEngine.robot.hip_positions(l, :)';
            hip_world = pos + R * hip_rel;
            
            % Foot coordinate
            foot_w = simEngine.state.foot_pos_world(l, :)';
            
            % Intermediate Knee position (approximated midpoint with knee pitch bending)
            knee_world = 0.5 * (hip_world + foot_w) + R * [0.03; 0; 0.02];
            
            set(h_legs(l), 'XData', [hip_world(1), knee_world(1), foot_w(1)], ...
                           'YData', [hip_world(2), knee_world(2), foot_w(2)], ...
                           'ZData', [hip_world(3), knee_world(3), foot_w(3)]);
            
            % Foot contact color indicator (Green = Stance/Contact, Red = Swing)
            if simEngine.state.contact_states(l)
                set(h_feet(l), 'XData', foot_w(1), 'YData', foot_w(2), 'ZData', foot_w(3), ...
                    'MarkerFaceColor', [0.2, 0.9, 0.3], 'MarkerEdgeColor', 'k');
            else
                set(h_feet(l), 'XData', foot_w(1), 'YData', foot_w(2), 'ZData', foot_w(3), ...
                    'MarkerFaceColor', [0.9, 0.2, 0.2], 'MarkerEdgeColor', 'k');
            end
        end
        
        % Update Conveyor Chevron Striping
        offset = simEngine.conveyor.texture_offset;
        for s = 1:num_stripes
            xs = mod(-0.3 + (s - 1) * 0.40 - offset + 0.5, 4.0) - 0.5;
            set(h_stripes(s), 'XData', [xs, xs+0.05, xs+0.05, xs]);
        end
        
        % Update Telemetry Charts (Last 200 points)
        if ~isempty(simEngine.history.time)
            idx_view = max(1, length(simEngine.history.time) - 200):length(simEngine.history.time);
            t_buf = simEngine.history.time(idx_view);
            p_buf = rad2deg(simEngine.history.theta(idx_view));
            u_buf = simEngine.history.u(idx_view);
            
            set(line_pitch, 'XData', t_buf, 'YData', p_buf);
            set(line_u, 'XData', t_buf, 'YData', u_buf);
            xlim(ax_pitch, [min(t_buf), max(min(t_buf)+1.0, max(t_buf))]);
            xlim(ax_torque, [min(t_buf), max(min(t_buf)+1.0, max(t_buf))]);
        end
        
        drawnow limitrate;
    end

    function onTogglePlay(~, ~)
        simEngine.time.is_paused = ~simEngine.time.is_paused;
        if simEngine.time.is_paused
            set(btn_play, 'String', 'RESUME', 'BackgroundColor', [0.8, 0.6, 0.2]);
            set(lbl_status, 'String', 'STATUS: Simulation Paused.');
        else
            set(btn_play, 'String', 'PAUSE', 'BackgroundColor', [0.2, 0.6, 0.4]);
            set(lbl_status, 'String', 'STATUS: Running 200 Hz Locomotion Engine.');
        end
    end

    function onReset(~, ~)
        simEngine = init_3d_simulation();
        set(lbl_status, 'String', 'STATUS: Environment Reset to Nominal Configuration.');
    end

    function onConveyorSpeed(src, ~)
        simEngine.conveyor.speed = abs(src.Value);
        simEngine.conveyor.direction = sign(src.Value);
        set(lbl_speed, 'String', sprintf('%.2f m/s', src.Value));
        set(lbl_status, 'String', sprintf('STATUS: Conveyor Speed set to %.2f m/s', src.Value));
    end

    function onGaitFreq(src, ~)
        simEngine.gait.frequency = src.Value;
        set(lbl_freq, 'String', sprintf('%.1f Hz', src.Value));
    end

    function onStepHeight(src, ~)
        simEngine.gait.step_height = src.Value;
        set(lbl_ht, 'String', sprintf('%.2f m', src.Value));
    end

    function onInjectDisturbance(~, ~)
        simEngine.state.external_force(1) = 2.0; % 2.0 N external force
        set(lbl_status, 'String', 'STATUS: Injected External Disturbance Force on Trunk!');
    end

    function onToggleContactLoss(~, ~)
        if simEngine.state.contact_loss_leg == 2
            simEngine.state.contact_loss_leg = 0;
            set(lbl_status, 'String', 'STATUS: RF Contact Loss Cleared. Nominal 4-Leg Support Restored.');
            set(btn_fault, 'String', 'TOGGLE RF CONTACT LOSS', 'BackgroundColor', [0.7, 0.2, 0.3]);
        else
            simEngine.state.contact_loss_leg = 2; % Leg 2 = RF
            set(lbl_status, 'String', 'STATUS: RF FAULT INJECTED! Leg disabled, testing 3-leg fault recovery.');
            set(btn_fault, 'String', 'RESTORE CONTACT', 'BackgroundColor', [0.3, 0.7, 0.4]);
        end
    end

    function onSelectScenario(src, ~)
        val = src.Value;
        simEngine = init_3d_simulation();
        switch val
            case 1 % Nominal
                simEngine.conveyor.speed = 0.0;
                set(lbl_status, 'String', 'SCENARIO 1: Nominal Gait on Stationary Surface.');
            case 2 % Moving conveyor
                simEngine.conveyor.speed = 0.12;
                simEngine.conveyor.direction = 1.0;
                set(lbl_status, 'String', 'SCENARIO 2: Forward Moving Conveyor Belt.');
            case 3 % Obstacles
                simEngine.conveyor.speed = 0.08;
                set(lbl_status, 'String', 'SCENARIO 3: Obstacle Traversal (Step platform clearance).');
            case 4 % Uneven terrain
                simEngine.conveyor.speed = 0.06;
                set(lbl_status, 'String', 'SCENARIO 4: Uneven Terrain & Ramp Ascending.');
            case 5 % Disturbance rejection
                simEngine.conveyor.speed = 0.10;
                simEngine.state.external_force(1) = 2.5;
                set(lbl_status, 'String', 'SCENARIO 5: 0.10 N.m Disturbance Torque Pulse Rejection.');
            case 6 % Contact loss
                simEngine.state.contact_loss_leg = 2;
                set(lbl_status, 'String', 'SCENARIO 6: Single-Leg Contact-Loss Fault Tolerance.');
        end
    end

    function onClose(~, ~)
        stop(sim_timer);
        delete(sim_timer);
        delete(fig);
    end
end
