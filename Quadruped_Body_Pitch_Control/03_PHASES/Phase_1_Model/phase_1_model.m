%% QUADRUPED BODY-PITCH CONTROL - PHASE 1
% Initial SISO simulation & Physical Model

%% Physical parameters

if ~exist('J', 'var') || ~exist('B', 'var') || ~exist('K', 'var')
    robot_parameters;
end

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
