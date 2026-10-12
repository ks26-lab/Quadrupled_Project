function ctrl = lstm_pitch_controller(weights_or_mat, cfg)
% LSTM_PITCH_CONTROLLER Recurrent LSTM Supervised Controller Constructor.
%
% Implements a standard Recurrent LSTM cell for SISO Body-Pitch stabilization.
% Preserves identical interface: [u, ctrl] = lstm_step(ctrl, theta_d, theta_m).
%
% Inputs:
%   weights_or_mat : filepath to .mat file OR struct of trained weights.
%   cfg            : optional configuration struct (u_sat, dt, history_len).

if nargin < 2, cfg = struct(); end

if isfield(cfg, 'dt'), ctrl.dt = cfg.dt; else, ctrl.dt = 0.005; end
if isfield(cfg, 'u_sat'), ctrl.u_sat = cfg.u_sat; else, ctrl.u_sat = 0.40; end

if ischar(weights_or_mat) || isstring(weights_or_mat)
    if exist(weights_or_mat, 'file')
        loaded = load(weights_or_mat);
        if isfield(loaded, 'net'), ctrl.net = loaded.net;
        elseif isfield(loaded, 'weights'), ctrl.net = loaded.weights;
        else, ctrl.net = loaded;
        end
    else
        error('LSTM model file not found: %s', weights_or_mat);
    end
elseif isstruct(weights_or_mat)
    ctrl.net = weights_or_mat;
else
    error('weights_or_mat must be a filepath or struct.');
end

hidden_dim = ctrl.net.hidden_dim;

% Internal recurrent states: cell state c and hidden state h
ctrl.h = zeros(hidden_dim, 1);
ctrl.c = zeros(hidden_dim, 1);

% History tracking for numerical error derivative
ctrl.prev_e = 0.0;
ctrl.has_prev = false;
ctrl.last_u = 0.0;
ctrl.type = 'LSTM';
end
