%% PHASE 2 - SISO PLANT & OPEN-LOOP ANALYSIS

% Ensure plant model G exists
if ~exist('G', 'var')
    pitch_plant;
end

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
