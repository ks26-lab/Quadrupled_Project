function simEngine = step_3d_simulation(simEngine)
% STEP_3D_SIMULATION Advances the 3D quadruped simulation by one physics time step (dt).
%
% Implements:
%   1. Conveyor surface motion kinematics & moving ground effect
%   2. Phase 1-8 Gait coordination, foot trajectory & inverse kinematics
%   3. Closed-loop IMU body pitch sensing & frozen PI balance correction
%   4. Ground obstacle surface detection & contact collision reactions
%   5. Conveyor friction force transmission & slip dynamics
%   6. Telemetry logging for real-time visualizer and dashboards
%
% Usage:
%   simEngine = step_3d_simulation(simEngine);

if simEngine.time.is_paused
    return;
end

dt = simEngine.time.dt;
t  = simEngine.time.current;

robot    = simEngine.robot;
conveyor = simEngine.conveyor;
gait     = simEngine.gait;
state    = simEngine.state;
obs      = simEngine.obstacles;

% 1. Conveyor Belt Movement
if conveyor.running
    v_belt = conveyor.direction * conveyor.speed;
else
    v_belt = 0.0;
end
conveyor.texture_offset = mod(conveyor.texture_offset + v_belt * dt, 0.40);

% 2. Phase 6: Four-Leg Trot CPG Generator
T_gait = 1.0 / max(0.1, gait.frequency);
global_phase = mod(t / T_gait, 1.0);
leg_phases = zeros(4, 1);
stance_states = false(4, 1);
for leg = 1:4
    p = mod(global_phase + gait.phase_offsets(leg), 1.0);
    leg_phases(leg) = p;
    stance_states(leg) = (p >= 0.50);
end

% 3. Balance Control & Disturbance Integration
theta_m = state.rpy(2); % Body pitch from IMU
theta_d = 0.0;
pitch_err = theta_d - theta_m;

% Adaptive stride parameters (Phase 4.6 / 4.7 thresholds)
if abs(pitch_err) > 0.15
    curr_ht  = gait.step_height * 1.25;
    curr_len = gait.step_length * 0.70;
elseif abs(pitch_err) > 0.05
    curr_ht  = gait.step_height * 1.10;
    curr_len = gait.step_length * 0.85;
else
    curr_ht  = gait.step_height;
    curr_len = gait.step_length;
end

% Frozen PI Controller computation
u_p = robot.Kp * pitch_err;
int_cand = state.integrator + pitch_err * dt;
u_i = robot.Ki * int_cand;
u_unsat = u_p + u_i;
u_pitch = max(-robot.u_sat, min(robot.u_sat, u_unsat));

if ~((u_unsat > robot.u_sat && pitch_err > 0) || (u_unsat < -robot.u_sat && pitch_err < 0))
    state.integrator = int_cand;
end

% 4. Phase 3 & 7: Continuous 3D Foot Trajectory in Hip Frame
foot_hip = zeros(4, 3);
for leg = 1:4
    p = leg_phases(leg);
    x0 = robot.nominal_feet(leg, 1);
    y0 = robot.nominal_feet(leg, 2);
    z0 = robot.nominal_feet(leg, 3);
    
    if p < 0.50
        % Swing Phase (cycloidal lift)
        s = p / 0.50;
        dx = curr_len * (s - sin(2*pi*s)/(2*pi)) - (curr_len / 2.0);
        dz = curr_ht * (1.0 - cos(2*pi*s)) / 2.0;
    else
        % Stance Phase (linear retraction against ground)
        s_st = (p - 0.50) / 0.50;
        dx = (curr_len / 2.0) - curr_len * s_st;
        dz = 0.0;
    end
    foot_hip(leg, :) = [x0 + dx, y0, z0 + dz];
end

% 5. Phase 4 & 5: 12-DOF Analytical Inverse Kinematics
q_gait = zeros(12, 1);
feasible = true(4, 1);
for leg = 1:4
    xf = foot_hip(leg, 1);
    yf = foot_hip(leg, 2);
    zf = foot_hip(leg, 3);
    
    q1 = atan2(yf, -zf);
    r = sqrt(yf^2 + zf^2);
    D = sqrt(xf^2 + r^2);
    cosQ3 = (D^2 - robot.L1^2 - robot.L2^2) / (2.0 * robot.L1 * robot.L2);
    
    if abs(cosQ3) > 1.0
        feasible(leg) = false;
        q_leg = zeros(3, 1);
    else
        q3 = acos(cosQ3);
        alpha = atan2(xf, r);
        beta = atan2(robot.L2 * sin(q3), robot.L1 + robot.L2 * cos(q3));
        q2 = alpha - beta;
        q_leg = [q1; q2; q3];
    end
    base_idx = (leg - 1) * 3;
    q_gait(base_idx + 1) = q_leg(1);
    q_gait(base_idx + 2) = q_leg(2);
    q_gait(base_idx + 3) = q_leg(3);
end

% 6. Coupling: q_cmd = q_gait + delta_q_balance
delta_q = zeros(12, 1);
delta_q(2)  =  robot.k_torque_to_q * u_pitch;
delta_q(5)  =  robot.k_torque_to_q * u_pitch;
delta_q(8)  = -robot.k_torque_to_q * u_pitch;
delta_q(11) = -robot.k_torque_to_q * u_pitch;
q_command = q_gait + delta_q;

% 7. Forward Kinematics to calculate world foot coordinates
foot_world = zeros(4, 3);
pitch_angle = state.rpy(2);
R_pitch = [cos(pitch_angle) 0 sin(pitch_angle); 0 1 0; -sin(pitch_angle) 0 cos(pitch_angle)];

for leg = 1:4
    hip_rel = robot.hip_positions(leg, :)';
    foot_rel = foot_hip(leg, :)';
    pos_body = hip_rel + foot_rel;
    pos_world = state.pos + R_pitch * pos_body;
    foot_world(leg, :) = pos_world';
end

% 8. Physical Contact & Obstacle Collision Detection
% Conveyor surface level is z = conveyor.height (0.20 m)
terrain_z = conveyor.height * ones(4, 1);

for leg = 1:4
    fx = foot_world(leg, 1);
    fy = foot_world(leg, 2);
    
    % Query each obstacle
    for ob_idx = 1:length(obs)
        if ~obs(ob_idx).active, continue; end
        ox = obs(ob_idx).pos(1);
        oy = obs(ob_idx).pos(2);
        dx = obs(ob_idx).size(1);
        dy = obs(ob_idx).size(2);
        dz = obs(ob_idx).size(3);
        
        if (fx >= ox - dx/2 && fx <= ox + dx/2) && (fy >= oy - dy/2 && fy <= oy + dy/2)
            if strcmp(obs(ob_idx).type, 'ramp')
                % Linear elevation along ramp
                ramp_prog = (fx - (ox - dx/2)) / dx;
                terrain_z(leg) = max(terrain_z(leg), conveyor.height + dz * ramp_prog);
            else
                terrain_z(leg) = max(terrain_z(leg), conveyor.height + dz);
            end
        end
    end
end

% Check contact penetration and contact loss fault injection
contact_force = zeros(4, 1);
for leg = 1:4
    if state.contact_loss_leg == leg
        % Contact loss experiment: disable support
        contact_force(leg) = 0.0;
        stance_states(leg) = false;
        continue;
    end
    
    penetration = terrain_z(leg) - foot_world(leg, 3);
    if stance_states(leg) || penetration > 0
        contact_force(leg) = max(0.0, 1500.0 * max(0.0, penetration) + 50.0);
        stance_states(leg) = true;
    end
end

% 9. Body Dynamic Movement Integration
% Net forward velocity on conveyor: v_net = v_gait - v_conveyor
% If robot walks at +0.10 m/s and belt runs at +0.10 m/s, robot stays stationary in world frame!
v_gait = gait.vx_cmd;
v_robot_world = (v_gait - v_belt) * (mean(stance_states) > 0.25);

% Update Body Pitch Dynamics: J*theta_ddot + B*theta_dot = u_pitch + tau_terrain + tau_ext
% Terrain moment from differential front/rear ground heights
h_front = mean(terrain_z(1:2));
h_rear  = mean(terrain_z(3:4));
tau_terrain = -15.0 * (h_front - h_rear);

% External disturbance torque
tau_ext = state.external_force(1) * 0.10;

tau_net = u_pitch + tau_terrain + tau_ext;
theta_ddot = (tau_net - robot.B_pitch * state.omega(2)) / robot.J_pitch;

% Integrate pitch state
state.omega(2) = state.omega(2) + theta_ddot * dt;
state.rpy(2)   = state.rpy(2) + state.omega(2) * dt;

% Update world translation
state.pos(1) = state.pos(1) + v_robot_world * dt;
state.pos(3) = conveyor.height + robot.standing_height + (mean(terrain_z) - conveyor.height);

% Save states
state.q_cmd = q_command;
state.foot_pos_world = foot_world;
state.contact_states = stance_states;

simEngine.time.current = t + dt;
simEngine.conveyor = conveyor;
simEngine.state = state;

% Telemetry Logging
simEngine.history.time(end+1)  = simEngine.time.current;
simEngine.history.theta(end+1) = state.rpy(2);
simEngine.history.u(end+1)     = u_pitch;
simEngine.history.conveyor_x(end+1) = conveyor.texture_offset;
end
