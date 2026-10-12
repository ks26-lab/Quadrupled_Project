function [u, ctrl] = lstm_step(ctrl, theta_d, theta_m)
% LSTM_STEP Single-step recurrent inference for LSTM Pitch Controller.
%
% Implements standard LSTM equations:
%   i_t = sigmoid(W_i * x_t + U_i * h_{t-1} + b_i)
%   f_t = sigmoid(W_f * x_t + U_f * h_{t-1} + b_f)
%   g_t = tanh(W_g * x_t + U_g * h_{t-1} + b_g)
%   o_t = sigmoid(W_o * x_t + U_o * h_{t-1} + b_o)
%   c_t = f_t .* c_{t-1} + i_t .* g_t
%   h_t = o_t .* tanh(c_t)
%   u   = sat(W_out * h_t + b_out)
%
% SISO Interface:
%   Input  : theta_d [rad], theta_m [rad]
%   Output : u [N.m]

e = theta_d - theta_m;

if ~ctrl.has_prev
    e_dot = 0.0;
    ctrl.has_prev = true;
else
    e_dot = (e - ctrl.prev_e) / ctrl.dt;
end
ctrl.prev_e = e;

% Input feature vector: [e; e_dot; theta_m]
x_raw = [e; e_dot; theta_m];

% Normalize if scalers present
if isfield(ctrl.net, 'mu_x') && isfield(ctrl.net, 'sigma_x')
    x = (x_raw - ctrl.net.mu_x) ./ (ctrl.net.sigma_x + 1e-8);
else
    x = x_raw;
end

net = ctrl.net;
h_prev = ctrl.h;
c_prev = ctrl.c;

% Gate activations
i_gate = 1.0 ./ (1.0 + exp(-(net.Wi * x + net.Ui * h_prev + net.bi)));
f_gate = 1.0 ./ (1.0 + exp(-(net.Wf * x + net.Uf * h_prev + net.bf)));
g_gate = tanh(net.Wg * x + net.Ug * h_prev + net.bg);
o_gate = 1.0 ./ (1.0 + exp(-(net.Wo * x + net.Uo * h_prev + net.bo)));

% State updates
c = f_gate .* c_prev + i_gate .* g_gate;
h = o_gate .* tanh(c);

% Output projection
u_norm = net.W_out * h + net.b_out;

if isfield(ctrl.net, 'mu_u') && isfield(ctrl.net, 'sigma_u')
    u_unsat = u_norm * ctrl.net.sigma_u + ctrl.net.mu_u;
else
    u_unsat = u_norm;
end

% Actuator saturation limit enforcement
u = max(-ctrl.u_sat, min(ctrl.u_sat, u_unsat));

% Update recurrent internal states
ctrl.h = h;
ctrl.c = c;
ctrl.last_u = u;
end
