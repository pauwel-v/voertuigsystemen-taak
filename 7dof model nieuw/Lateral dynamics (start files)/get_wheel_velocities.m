function [v_long, v_lat, beta] = get_wheel_velocities(Vx, Vy, dphi, delta, x, y)
% Input
% Vx, Vy: vehicle velocities in x'y' frame
% dphi: rotational velocity vehicle
% delta: total heading angle of wheel
% x, y: tyre coordinates in x'y' frame
%
% Output
% v_long: longitudinal tyre velocity in wheel frame
% v_lat: lateral tyre velocity wheel frame
% beta: slip agle wheel

% Calculation wheel velocities in x'y' frame
% v_long_accent = Vx;
% v_lat_accent = Vy + cross([0;0;dphi], [x,y,0]);
v_accent = [Vx; Vy; 0] + cross([0;0;dphi], [x;y;0]);

% Calculation wheel rotation matrix
R = [cos(delta) sin(delta); -sin(delta) cos(delta)];

v_wheel = R*[v_accent(1); v_accent(2)];

v_long = v_wheel(1);
v_lat = v_wheel(2);

beta = atan2(v_lat, v_long);
end