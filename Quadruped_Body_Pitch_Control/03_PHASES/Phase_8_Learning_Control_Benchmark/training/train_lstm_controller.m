function net = train_lstm_controller(dataset, hidden_dim, max_epochs)
% TRAIN_LSTM_CONTROLLER Trains recurrent LSTM weights using sequence gradient optimization.
%
% Implements forward and truncated Backpropagation Through Time (BPTT) with Adam.
% Requires ZERO external toolboxes (runs seamlessly on base MATLAB).
%
% Architecture:
%   Input dim  : 3 ([e; e_dot; theta_m])
%   Hidden dim : 16 (default)
%   Output dim : 1 (control torque u)

if nargin < 2 || isempty(hidden_dim), hidden_dim = 16; end
if nargin < 3 || isempty(max_epochs), max_epochs = 40; end

in_dim = 3;
out_dim = 1;

rng(123); % Fixed weight initialization seed

% Xavier/Glorot weight initialization
scale_in  = sqrt(2.0 / (in_dim + hidden_dim));
scale_rec = sqrt(2.0 / (2 * hidden_dim));
scale_out = sqrt(2.0 / (hidden_dim + out_dim));

net.Wi = (rand(hidden_dim, in_dim) * 2 - 1) * scale_in;
net.Ui = (rand(hidden_dim, hidden_dim) * 2 - 1) * scale_rec;
net.bi = zeros(hidden_dim, 1);

net.Wf = (rand(hidden_dim, in_dim) * 2 - 1) * scale_in;
net.Uf = (rand(hidden_dim, hidden_dim) * 2 - 1) * scale_rec;
net.bf = ones(hidden_dim, 1); % Forget gate positive bias for long-term memory

net.Wg = (rand(hidden_dim, in_dim) * 2 - 1) * scale_in;
net.Ug = (rand(hidden_dim, hidden_dim) * 2 - 1) * scale_rec;
net.bg = zeros(hidden_dim, 1);

net.Wo = (rand(hidden_dim, in_dim) * 2 - 1) * scale_in;
net.Uo = (rand(hidden_dim, hidden_dim) * 2 - 1) * scale_rec;
net.bo = zeros(hidden_dim, 1);

net.W_out = (rand(out_dim, hidden_dim) * 2 - 1) * scale_out;
net.b_out = zeros(out_dim, 1);

net.hidden_dim = hidden_dim;
net.in_dim     = in_dim;
net.mu_x       = dataset.mu_x;
net.sigma_x    = dataset.sigma_x;
net.mu_u       = dataset.mu_u;
net.sigma_u    = dataset.sigma_u;

% Adam optimizer hyperparameters
lr = 0.005;
beta1 = 0.9;
beta2 = 0.999;
eps_adam = 1e-8;

param_names = {'Wi', 'Ui', 'bi', 'Wf', 'Uf', 'bf', 'Wg', 'Ug', 'bg', 'Wo', 'Uo', 'bo', 'W_out', 'b_out'};
for p = 1:length(param_names)
    m_adam.(param_names{p}) = zeros(size(net.(param_names{p})));
    v_adam.(param_names{p}) = zeros(size(net.(param_names{p})));
end

fprintf('Starting LSTM training (Epochs = %d, Hidden = %d, Sequences = %d)...\n', ...
    max_epochs, hidden_dim, dataset.N_trajectories);

num_seqs = dataset.N_trajectories;
t_step = 0;

for epoch = 1:max_epochs
    total_loss = 0.0;
    seq_order = randperm(num_seqs);
    
    for s_idx = 1:num_seqs
        seq = seq_order(s_idx);
        X_seq = dataset.X{seq};
        Y_seq = dataset.Y{seq};
        T_len = size(X_seq, 1);
        
        % Normalize inputs
        X_norm = (X_seq' - net.mu_x) ./ (net.sigma_x + 1e-8);
        Y_norm = (Y_seq' - net.mu_u) ./ (net.sigma_u + 1e-8);
        
        % Preallocate forward states
        h_cache = zeros(hidden_dim, T_len + 1);
        c_cache = zeros(hidden_dim, T_len + 1);
        i_cache = zeros(hidden_dim, T_len);
        f_cache = zeros(hidden_dim, T_len);
        g_cache = zeros(hidden_dim, T_len);
        o_cache = zeros(hidden_dim, T_len);
        y_pred  = zeros(out_dim, T_len);
        
        % Forward pass through time
        for t = 1:T_len
            x_t = X_norm(:, t);
            h_prev = h_cache(:, t);
            c_prev = c_cache(:, t);
            
            i_t = 1.0 ./ (1.0 + exp(-(net.Wi * x_t + net.Ui * h_prev + net.bi)));
            f_t = 1.0 ./ (1.0 + exp(-(net.Wf * x_t + net.Uf * h_prev + net.bf)));
            g_t = tanh(net.Wg * x_t + net.Ug * h_prev + net.bg);
            o_t = 1.0 ./ (1.0 + exp(-(net.Wo * x_t + net.Uo * h_prev + net.bo)));
            
            c_t = f_t .* c_prev + i_t .* g_t;
            h_t = o_t .* tanh(c_t);
            
            i_cache(:, t) = i_t;
            f_cache(:, t) = f_t;
            g_cache(:, t) = g_t;
            o_cache(:, t) = o_t;
            c_cache(:, t+1) = c_t;
            h_cache(:, t+1) = h_t;
            
            y_pred(:, t) = net.W_out * h_t + net.b_out;
        end
        
        % MSE Loss
        diff = y_pred - Y_norm;
        loss = mean(sum(diff.^2, 1));
        total_loss = total_loss + loss;
        
        % Backpropagation Through Time (BPTT)
        for p = 1:length(param_names)
            grad.(param_names{p}) = zeros(size(net.(param_names{p})));
        end
        
        dh_next = zeros(hidden_dim, 1);
        dc_next = zeros(hidden_dim, 1);
        
        % Truncated stride backprop to balance speed and accuracy
        stride = 2;
        for t = T_len:-stride:1
            dy = (2.0 / T_len) * diff(:, t);
            grad.W_out = grad.W_out + dy * (h_cache(:, t+1))';
            grad.b_out = grad.b_out + dy;
            
            dh = net.W_out' * dy + dh_next;
            c_t = c_cache(:, t+1);
            c_prev = c_cache(:, t);
            h_prev = h_cache(:, t);
            x_t = X_norm(:, t);
            
            tanh_c = tanh(c_t);
            do_t = dh .* tanh_c .* o_cache(:, t) .* (1.0 - o_cache(:, t));
            dc = dh .* o_cache(:, t) .* (1.0 - tanh_c.^2) + dc_next;
            
            df_t = dc .* c_prev .* f_cache(:, t) .* (1.0 - f_cache(:, t));
            di_t = dc .* g_cache(:, t) .* i_cache(:, t) .* (1.0 - i_cache(:, t));
            dg_t = dc .* i_cache(:, t) .* (1.0 - g_cache(:, t).^2);
            
            grad.Wi = grad.Wi + di_t * x_t';
            grad.Ui = grad.Ui + di_t * h_prev';
            grad.bi = grad.bi + di_t;
            
            grad.Wf = grad.Wf + df_t * x_t';
            grad.Uf = grad.Uf + df_t * h_prev';
            grad.bf = grad.bf + df_t;
            
            grad.Wg = grad.Wg + dg_t * x_t';
            grad.Ug = grad.Ug + dg_t * h_prev';
            grad.bg = grad.bg + dg_t;
            
            grad.Wo = grad.Wo + do_t * x_t';
            grad.Uo = grad.Uo + do_t * h_prev';
            grad.bo = grad.bo + do_t;
            
            dh_next = net.Ui' * di_t + net.Uf' * df_t + net.Ug' * dg_t + net.Uo' * do_t;
            dc_next = dc .* f_cache(:, t);
        end
        
        % Adam Parameter Update with Gradient Clipping
        t_step = t_step + 1;
        for p = 1:length(param_names)
            g_clip = max(-5.0, min(5.0, grad.(param_names{p})));
            m_adam.(param_names{p}) = beta1 * m_adam.(param_names{p}) + (1 - beta1) * g_clip;
            v_adam.(param_names{p}) = beta2 * v_adam.(param_names{p}) + (1 - beta2) * (g_clip.^2);
            
            m_hat = m_adam.(param_names{p}) / (1 - beta1^t_step);
            v_hat = v_adam.(param_names{p}) / (1 - beta2^t_step);
            
            net.(param_names{p}) = net.(param_names{p}) - (lr / (sqrt(v_hat) + eps_adam)) .* m_hat;
        end
    end
    
    avg_loss = total_loss / num_seqs;
    if mod(epoch, 10) == 0 || epoch == 1 || epoch == max_epochs
        fprintf('   Epoch [%2d/%2d] - Mean Squared Sequence Loss: %.6f\n', epoch, max_epochs, avg_loss);
    end
end

fprintf('LSTM Controller training completed successfully.\n');
end
