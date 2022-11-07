function vehicle_param = set_vehicle_param()
%
% This function is for defining the vehicle parameters in a structure
% 'vehicle_param'. 
%

% Set number of degrees-of-freedom for the model:
vehicle_param.n_dofs = 7;

% exam assignment a = 1.751

% Define additional system parameters:
vehicle_param.M = 1500; % vehicle mass [kg]
vehicle_param.Izz = 2058; % vehicle moment of inertia [kgm²]
vehicle_param.L = 3.18; % Wheel base [m]
vehicle_param.h = 0.5;  % cog heigth wrt wheel axle [m]
vehicle_param.a = 1.751;  % Distance between front axle and cog [m]
vehicle_param.b = vehicle_param.L-vehicle_param.a;

% Aerodynamic data
vehicle_param.h_aero = 0.4; % height of aerodynamic center [m]
vehicle_param.Cv = 0.3; % Aerodynamics drag coefficient [Ns²/(m.kg)]
vehicle_param.A_front = 1.9; % frontal surface area [m²]
vehicle_param.rho_air = 1.3; % density of air [kg/m³]

% Tyre data
vehicle_param.r_w = 0.3; % wheel radius [m]
vehicle_param.I_w = 25; % Moment of inertia for wheel [kgm²]
vehicle_param.Cx_f = 1.3e5; % longitudinal tyre-stiffness front wheels
vehicle_param.Cy_f = 3e4; % lateral tyre-stiffness front wheels
vehicle_param.Cx_r = 1.3e5; % longitudinal tyre-stiffness rear wheels
vehicle_param.Cy_r = 3e4; % lateral tyre-stiffness rear wheels
vehicle_param.mu = 1; % friction constant between wheel-road [/]

% Vehicle data
vehicle_param.tf = 1.8; % Track width of the front axle [m]
vehicle_param.tr = 1.8; % Track width of the rear axle [m]

% Transient tyre
vehicle_param.sigma_x = 0.5;
vehicle_param.sigma_y = 0.5;




