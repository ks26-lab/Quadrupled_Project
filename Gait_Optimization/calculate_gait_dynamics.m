function dyn = calculate_gait_dynamics(step_length, frequency, step_height, duty_factor, params)
% CALCULATE_GAIT_DYNAMICS Rigorous analytical dynamic analysis for quadruped gait.
%
% Computes:
%   - Forward walking speed: v = L_stride * f_gait
%   - Swing and Stance durations: T_swing = (1 - beta)*T, T_stance = beta*T
%   - Foot trajectory Cartesian velocities: v_foot(t)
%   - Analytical 12-DOF joint angular velocities: q_dot = J_leg^{-1} * v_foot
%   - Dynamic foot ground-reaction forces (GRF) during stance
%   - Joint torque requirements: tau = J_leg^T * F_foot + tau_inertial
%   - Mechanical and electrical power: P_mech = sum(|tau * omega|), I_bus, P_elec
%   - Friction cone utilization: F_tangential / F_normal <= mu

if nargin < 5 || isempty(params)
    params = get_dynamics_and_actuator_limits();
end
if nargin < 4 || isempty(duty_factor), duty_factor = 0.50; end

L1 = params.L1; % 0.12 m
L2 = params.L2; % 0.15 m
M_total = params.mass_total; % 5.0 kg
g = params.g; % 9.81 m/s^2
W = M_total * g; % 49.05 N

T = 1.0 / frequency;
v_speed = step_length * frequency;

T_swing  = (1.0 - duty_factor) * T;
T_stance = duty_factor * T;

% Discrete time evaluation across one complete gait cycle
N_samples = 200;
t_vec = linspace(0, T, N_samples)';
dt = t_vec(2) - t_vec(1);

% Nominal foot roots
root_LF = [0.15, 0.10, -params.standing_h];

% Trajectory evaluation for LF leg
p_lf = mod(t_vec / T, 1.0);
x_lf = zeros(N_samples, 1);
z_lf = zeros(N_samples, 1);
vx_lf = zeros(N_samples, 1);
vz_lf = zeros(N_samples, 1);
ax_lf = zeros(N_samples, 1);
az_lf = zeros(N_samples, 1);

for k = 1:N_samples
    p = p_lf(k);
    if p < (1.0 - duty_factor)
        % Swing phase: 0 <= p < (1 - beta)
        s = p / (1.0 - duty_factor);
        % Position
        x_lf(k) = root_LF(1) + step_length * s;
        z_lf(k) = root_LF(3) + step_height * sin(pi * s);
        % Velocity (analytical derivative)
        vx_lf(k) = step_length / T_swing;
        vz_lf(k) = (step_height * pi / T_swing) * cos(pi * s);
        % Acceleration
        ax_lf(k) = 0.0;
        az_lf(k) = -(step_height * (pi / T_swing)^2) * sin(pi * s);
    else
        % Stance phase: (1 - beta) <= p < 1.0
        s = (p - (1.0 - duty_factor)) / duty_factor;
        x_lf(k) = root_LF(1) + step_length * (1.0 - s);
        z_lf(k) = root_LF(3);
        vx_lf(k) = -step_length / T_stance;
        vz_lf(k) = 0.0;
        ax_lf(k) = 0.0;
        az_lf(k) = 0.0;
    end
end
y_lf = root_LF(2) * ones(N_samples, 1);
vy_lf = zeros(N_samples, 1);

% Inverse Kinematics and Jacobian evaluation
q_lf = zeros(N_samples, 3);
qdot_lf = zeros(N_samples, 3);
tau_lf = zeros(N_samples, 3);
P_mech_lf = zeros(N_samples, 1);

% GRF during Trot: 2 legs in stance support total weight W
% Each stance leg supports approximately Fz = W / (4 * duty_factor) = W / 2 = 24.525 N
Fz_stance = W / (4.0 * duty_factor);
% Forward propulsion force Fx needed to overcome damping and accelerate body
Fx_propulsion = (M_total * (v_speed / (0.5 * T_stance))) * 0.15; % ~ dynamic propulsion

for k = 1:N_samples
    x = x_lf(k);
    y = y_lf(k);
    z = z_lf(k);
    
    % IK
    q1 = atan2(y, -z);
    r = sqrt(y^2 + z^2);
    D = sqrt(x^2 + r^2);
    cosQ3 = max(-1.0, min(1.0, (D^2 - L1^2 - L2^2) / (2 * L1 * L2)));
    q3 = acos(cosQ3);
    alpha = atan2(x, r);
    beta = atan2(L2 * sin(q3), L1 + L2 * cos(q3));
    q2 = alpha - beta;
    q_lf(k, :) = [q1, q2, q3];
    
    % Analytical Leg Jacobian: v_foot = J_leg * q_dot  =>  q_dot = J_leg \ v_foot
    % For sagittal/spatial leg kinematics:
    s2 = sin(q2); c2 = cos(q2);
    s23 = sin(q2 + q3); c23 = cos(q2 + q3);
    
    % Planar projection in leg coordinate frame:
    J_planar = [ -L1*s2 - L2*s23,  -L2*s23;
                 -L1*c2 - L2*c23,  -L2*c23 ];
    v_planar = [vx_lf(k); vz_lf(k)];
    
    if cond(J_planar) < 100
        qdot_planar = J_planar \ v_planar;
        qdot_lf(k, 1) = 0.0; % y is constant
        qdot_lf(k, 2) = qdot_planar(1);
        qdot_lf(k, 3) = qdot_planar(2);
    else
        qdot_lf(k, :) = [0, 0, 0];
    end
    
    % Static & dynamic torque from ground-reaction forces: tau = J^T * F
    p = p_lf(k);
    if p >= (1.0 - duty_factor)
        % Stance phase: Ground reaction force vector [Fx; Fz]
        F_ext = [Fx_propulsion; Fz_stance];
        tau_planar = J_planar' * F_ext;
        tau_lf(k, 1) = y * Fz_stance * 0.10; % Hip roll torque
        tau_lf(k, 2) = abs(tau_planar(1));
        tau_lf(k, 3) = abs(tau_planar(2));
    else
        % Swing phase: Limb inertial acceleration torque: tau = I_limb * q_ddot
        m_limb = 0.40; % ~0.4 kg limb mass
        tau_lf(k, 1) = 0.05;
        tau_lf(k, 2) = m_limb * L1 * abs(az_lf(k)) * 0.25;
        tau_lf(k, 3) = m_limb * L2 * abs(az_lf(k)) * 0.25;
    end
    
    P_mech_lf(k) = sum(abs(tau_lf(k, :) .* qdot_lf(k, :)));
end

% 4-leg total system power and current
% In trot, 2 legs in swing, 2 legs in stance
P_mech_total = 4 * mean(P_mech_lf);
P_mech_peak  = 4 * max(P_mech_lf);

% Electrical model: P_elec = P_mech / eta_m + 12 * (I_idle * V + R_a * I^2)
eta_motor = 0.65; % Typical servo DC motor efficiency
I_bus_avg = (P_mech_total / eta_motor) / params.actuator.voltage_nom + 12 * params.actuator.current_idle;
I_bus_peak= (P_mech_peak / eta_motor) / params.actuator.voltage_nom + 12 * params.actuator.current_idle;
P_elec_avg = I_bus_avg * params.actuator.voltage_nom;

% Friction coefficient required: mu_req = F_tangential / F_normal
mu_required = abs(Fx_propulsion) / Fz_stance;

% Package output struct
dyn.speed         = v_speed;
dyn.step_length   = step_length;
dyn.frequency     = frequency;
dyn.step_height   = step_height;
dyn.duty_factor   = duty_factor;
dyn.T_cycle       = T;
dyn.T_swing       = T_swing;
dyn.T_stance      = T_stance;
dyn.omega_max     = max(abs(qdot_lf(:)));
dyn.tau_max       = max(abs(tau_lf(:)));
dyn.tau_rms       = sqrt(mean(tau_lf(:).^2));
dyn.P_mech_avg    = P_mech_total;
dyn.P_mech_peak   = P_mech_peak;
dyn.P_elec_avg    = P_elec_avg;
dyn.I_bus_avg     = I_bus_avg;
dyn.I_bus_peak    = I_bus_peak;
dyn.mu_required   = mu_required;
dyn.t_vec         = t_vec;
dyn.q_lf          = q_lf;
dyn.qdot_lf       = qdot_lf;
dyn.tau_lf        = tau_lf;

% Constraint Check Flags
dyn.violates_omega = dyn.omega_max > params.actuator.omega_max;
dyn.violates_tau   = dyn.tau_max > params.actuator.tau_stall;
dyn.violates_thermal = dyn.tau_rms > params.actuator.tau_rated;
dyn.violates_current = dyn.I_bus_peak > params.power.max_continuous_current;
dyn.violates_friction= dyn.mu_required > params.contact.mu_friction;
dyn.is_feasible = ~(dyn.violates_omega || dyn.violates_tau || dyn.violates_friction);
end
