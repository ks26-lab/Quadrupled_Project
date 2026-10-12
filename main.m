clc;
clear;
close all;

%% QUADRUPED BODY-PITCH CONTROL
% Initial SISO simulation

%% Physical parameters

J = 0.05;      % Body pitch moment of inertia
B = 0.10;      % Damping coefficient
K = 0;         % Initial restoring stiffness

%% Plant model

s = tf('s');

G = 1/(J*s^2 + B*s + K);

disp('Quadruped pitch plant:');
G

%% Open-loop step response

%figure;

step(G);

grid on;

title('Open-Loop Body Pitch Response');

xlabel('Time (s)');
ylabel('Pitch Angle \theta (rad)');

%% Test 3 - Initial Condition Response

% Convert initial pitch from degrees to radians
theta0 = deg2rad(5);

% Initial angular velocity
omega0 = 0;

% State-space model
A = [0 1;
    0 -B/J];

B_matrix = [0;
    1/J];

C_matrix = [1 0];

D_matrix = 0;

sys_ss = ss(A,B_matrix,C_matrix,D_matrix);

% Time vector
t = 0:0.01:10;

% Initial state vector
x0 = [theta0;
    omega0];

% Simulate response from initial condition
theta_initial = initial(sys_ss,x0,t);

% Plot response
%figure;

plot(t,rad2deg(theta_initial),'LineWidth',1.5);

grid on;

title('Test 3 - Initial Pitch Response');

xlabel('Time (s)');

ylabel('Pitch Angle \theta (deg)');

%% PHASE 2 - SISO PLANT & OPEN-LOOP ANALYSIS

%% Pole-zero analysis

disp('Poles of the plant:');
pole(G)

disp('Zeros of the plant:');
zero(G)


%% Pole-zero map

%figure
pzmap(G);

grid on;

title('Pole-Zero Map of Open-Loop Plant');


%% Open-loop step response

%figure

step(G);

grid on;

title('Open-Loop Step Response');

xlabel('Time (s)');

ylabel('Pitch Angle \theta (rad)');


%% Open-loop impulse response

%figure
impulse(G);

grid on;

title('Open-Loop Impulse Response');

xlabel('Time (s)');

ylabel('Pitch Angle \theta (rad)');

%% PHASE 2 - ANALYTICAL PLANT VALIDATION

% Calculate poles and zeros
plant_poles = pole(G);
plant_zeros = zero(G);

disp(' ');
disp('===== PLANT ANALYSIS =====');

disp('Poles:');
disp(plant_poles);

disp('Zeros:');
disp(plant_zeros);


%% Analytical parameters

disp('System type:');

if any(abs(plant_poles) < 1e-10)
    disp('Type 1 system - one pole at the origin');
else
    disp('No pole at the origin');
end


%% Analytical impulse response final value

impulse_final_value = 20/2;

fprintf('Expected final impulse response = %.2f rad\n', ...
    impulse_final_value);


%% Pole locations

fprintf('Pole 1 = %.2f\n', plant_poles(1));
fprintf('Pole 2 = %.2f\n', plant_poles(2));

%% PHASE 3 - FEEDBACK CONTROLLER DESIGN

%% Proportional Controller

Kp = 1;

C = Kp;

%% Closed-loop system

T = feedback(C*G,1);

disp(' ');
disp('===== CLOSED-LOOP SYSTEM =====');

T


%% Closed-loop step response

%figure;

step(T);

grid on;

title('Closed-Loop Response with Proportional Control');

xlabel('Time (s)');

ylabel('Pitch Angle \theta (rad)');

%% Closed-loop performance analysis

info = stepinfo(T);

disp(' ');
disp('===== CLOSED-LOOP PERFORMANCE =====');

disp(['Rise Time = ', num2str(info.RiseTime), ' s']);

disp(['Settling Time = ', num2str(info.SettlingTime), ' s']);

disp(['Overshoot = ', num2str(info.Overshoot), ' %']);

disp(['Peak = ', num2str(info.Peak), ' rad']);

disp(['Peak Time = ', num2str(info.PeakTime), ' s']);


%% Closed-loop poles

disp(' ');
disp('Closed-loop poles:');

pole(T)

%% PHASE 3 - SYSTEMATIC PROPORTIONAL GAIN STUDY

% Range of proportional gains
Kp_values = 0.05:0.05:5;

% Preallocate result arrays
rise_time = zeros(size(Kp_values));
settling_time = zeros(size(Kp_values));
overshoot = zeros(size(Kp_values));
peak_value = zeros(size(Kp_values));

% Store closed-loop poles
pole_real = zeros(size(Kp_values));
pole_imag = zeros(size(Kp_values));

%% Run parameter sweep

for i = 1:length(Kp_values)

    Kp = Kp_values(i);

    % Proportional controller
    C = Kp;

    % Closed-loop system
    T = feedback(C*G,1);

    % Step-response information
    info = stepinfo(T);

    % Store performance metrics
    rise_time(i) = info.RiseTime;
    settling_time(i) = info.SettlingTime;
    overshoot(i) = info.Overshoot;
    peak_value(i) = info.Peak;

    % Store poles
    p = pole(T);

    pole_real(i) = real(p(1));
    pole_imag(i) = abs(imag(p(1)));

end


%% Display selected results

results = table(Kp_values', ...
                rise_time', ...
                settling_time', ...
                overshoot', ...
                peak_value', ...
                'VariableNames', ...
                {'Kp','RiseTime','SettlingTime','Overshoot','Peak'});

disp(' ');
disp('===== SYSTEMATIC Kp STUDY =====');

disp(results);


%% Plot 1 - Rise time versus Kp

figure;

plot(Kp_values,rise_time,'LineWidth',1.5);

grid on;

title('Rise Time versus Proportional Gain');

xlabel('K_p');

ylabel('Rise Time (s)');


%% Plot 2 - Settling time versus Kp

figure;

plot(Kp_values,settling_time,'LineWidth',1.5);

grid on;

title('Settling Time versus Proportional Gain');

xlabel('K_p');

ylabel('Settling Time (s)');


%% Plot 3 - Overshoot versus Kp

figure;

plot(Kp_values,overshoot,'LineWidth',1.5);

grid on;

title('Overshoot versus Proportional Gain');

xlabel('K_p');

ylabel('Overshoot (%)');


%% Plot 4 - Peak value versus Kp

figure;

plot(Kp_values,peak_value,'LineWidth',1.5);

grid on;

title('Peak Pitch versus Proportional Gain');

xlabel('K_p');

ylabel('Peak Pitch (rad)');


%% Plot 5 - Closed-loop pole location

figure;

plot(pole_real,pole_imag,'LineWidth',1.5);

grid on;

title('Closed-Loop Pole Movement with K_p');

xlabel('Real Part');

ylabel('Imaginary Part');

%% PHASE 3 - FEASIBLE Kp REGION

% IMPORTANT:
% These are preliminary illustrative design limits.
% Final limits must be justified from robot requirements/literature.

max_overshoot = 25;       % percent
max_settling_time = 4;    % seconds
max_rise_time = 1;        % seconds


%% Find gains satisfying all constraints

feasible = (overshoot <= max_overshoot) & ...
    (settling_time <= max_settling_time) & ...
    (rise_time <= max_rise_time);


feasible_Kp = Kp_values(feasible);


%% Display feasible region

disp(' ');
disp('===== FEASIBLE Kp REGION =====');

if isempty(feasible_Kp)

    disp('No Kp satisfies all constraints.');

else

    fprintf('Minimum feasible Kp = %.2f\n', ...
        min(feasible_Kp));

    fprintf('Maximum feasible Kp = %.2f\n', ...
        max(feasible_Kp));

    fprintf('Number of feasible gains = %d\n', ...
        length(feasible_Kp));

end


%% Display feasible gain table

feasible_results = results(feasible,:);

disp(' ');
disp('===== FEASIBLE CONTROLLER CONFIGURATIONS =====');

disp(feasible_results);


%% PHASE 3 - BASELINE P CONTROLLER COMPARISON

baseline_Kp = [0.25 0.30];

baseline_results = zeros(length(baseline_Kp),6);

for i = 1:length(baseline_Kp)

    Kp = baseline_Kp(i);

    C = Kp;

    T = feedback(C*G,1);

    info = stepinfo(T);

    % Final value of the response
    final_value = dcgain(T);

    % Store results
    baseline_results(i,1) = Kp;
    baseline_results(i,2) = info.RiseTime;
    baseline_results(i,3) = info.SettlingTime;
    baseline_results(i,4) = info.Overshoot;
    baseline_results(i,5) = info.Peak;
    baseline_results(i,6) = final_value;

end


%% Display baseline comparison

disp(' ');
disp('===== BASELINE P CONTROLLER COMPARISON =====');

disp(' Kp      RiseTime    SettlingTime    Overshoot    Peak    FinalValue');

disp(baseline_results);


%% Compare the two responses

figure;

hold on;

for i = 1:length(baseline_Kp)

    Kp = baseline_Kp(i);

    C = Kp;

    T = feedback(C*G,1);

    step(T);

end

grid on;

title('Comparison of Feasible P Controllers');

xlabel('Time (s)');

ylabel('Pitch Angle \theta (rad)');

legend('K_p = 0.25','K_p = 0.30');

hold off;

%% PHASE 3 - DISTURBANCE REJECTION

baseline_Kp = [0.25 0.30];

%% Disturbance definition

t = 0:0.01:10;

% Transient disturbance:
% 0.1 N.m torque applied from 2 s to 2.5 s

Td_pulse = zeros(size(t));

Td_pulse(t >= 2 & t < 2.5) = 0.1;


% Sustained disturbance:
% 0.1 N.m torque applied from 2 s onward

Td_step = zeros(size(t));

Td_step(t >= 2) = 0.1;


%% Test both baseline controllers

for i = 1:length(baseline_Kp)

    Kp = baseline_Kp(i);

    C = Kp;

    % Closed-loop disturbance transfer function
    Gd = G/(1 + C*G);

    % Transient disturbance response
    theta_pulse = lsim(Gd,Td_pulse,t);

    % Sustained disturbance response
    theta_step = lsim(Gd,Td_step,t);


    %% Plot transient disturbance

    figure;

    plot(t,rad2deg(theta_pulse),'LineWidth',1.5);

    grid on;

    title(['Transient Disturbance Response, K_p = ',num2str(Kp)]);

    xlabel('Time (s)');

    ylabel('Pitch Angle \theta (deg)');


    %% Plot sustained disturbance

    figure;

    plot(t,rad2deg(theta_step),'LineWidth',1.5);

    grid on;

    title(['Sustained Disturbance Response, K_p = ',num2str(Kp)]);

    xlabel('Time (s)');

    ylabel('Pitch Angle \theta (deg)');


end

%% PHASE 3 - DISTURBANCE PERFORMANCE METRICS

baseline_Kp = [0.25 0.30];

disturbance_results = zeros(length(baseline_Kp),5);

for i = 1:length(baseline_Kp)

    Kp = baseline_Kp(i);

    C = Kp;

    % Disturbance transfer function
    Gd = G/(1 + C*G);

    % Simulate transient disturbance
    theta_pulse = lsim(Gd,Td_pulse,t);

    % Simulate sustained disturbance
    theta_step = lsim(Gd,Td_step,t);


    %% Transient disturbance metrics

    max_deviation = max(abs(theta_pulse));

    final_transient = theta_pulse(end);


    %% Sustained disturbance metrics

    final_sustained = theta_step(end);

    steady_state_error = abs(final_sustained);


    %% Store results

    disturbance_results(i,1) = Kp;

    disturbance_results(i,2) = rad2deg(max_deviation);

    disturbance_results(i,3) = rad2deg(final_transient);

    disturbance_results(i,4) = rad2deg(final_sustained);

    disturbance_results(i,5) = rad2deg(steady_state_error);

end


%% Display results

disp(' ');
disp('===== DISTURBANCE PERFORMANCE =====');

disp(' Kp    Max Transient Deviation    Final Transient    Final Sustained    Sustained Error');

disp(disturbance_results);

%% THEORETICAL vs SIMULATED DISTURBANCE ERROR

Td0 = 0.1;

theoretical_error = Td0 ./ baseline_Kp;

simulated_error = deg2rad( ...
    disturbance_results(:,5));

comparison = table( ...
    baseline_Kp', ...
    theoretical_error', ...
    simulated_error, ...
    'VariableNames', ...
    {'Kp','TheoreticalError_rad','SimulatedError_rad'});

disp(' ');
disp('===== THEORETICAL vs SIMULATED =====');

disp(comparison);


%% Percentage difference

percentage_difference = ...
    abs(simulated_error - theoretical_error') ...
    ./ theoretical_error' * 100;

disp('Percentage difference (%):');

disp(percentage_difference);