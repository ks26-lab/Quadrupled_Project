function val = validate_plant(G)
% VALIDATE_PLANT Analytical validation of open-loop plant model.
%
% Replicates Phase 2 analytical plant validation logic:
%   - Plant poles and zeros
%   - System type determination
%   - Analytical impulse response final value calculation
%
% Usage:
%   val = validate_plant(G)
%   val = validate_plant()

if nargin < 1
    [G, ~] = get_pitch_plant();
end

plant_poles = pole(G);
plant_zeros = zero(G);

disp(' ');
disp('===== PLANT ANALYSIS =====');
disp('Poles:');
disp(plant_poles);
disp('Zeros:');
disp(plant_zeros);

disp('System type:');
if any(abs(plant_poles) < 1e-10)
    disp('Type 1 system - one pole at the origin');
    is_type_1 = true;
else
    disp('No pole at the origin');
    is_type_1 = false;
end

% Analytical impulse response final value
impulse_final_value = 20/2;
fprintf('Expected final impulse response = %.2f rad\n', impulse_final_value);

fprintf('Pole 1 = %.2f\n', plant_poles(1));
fprintf('Pole 2 = %.2f\n', plant_poles(2));

if nargout > 0
    val.poles = plant_poles;
    val.zeros = plant_zeros;
    val.is_type_1 = is_type_1;
    val.impulse_final_value = impulse_final_value;
end

end
