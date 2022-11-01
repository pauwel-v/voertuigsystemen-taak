function vehicle_param = set_vehicle_param()
%
% This function is for defining the vehicle parameters in a structure
% 'vehicle_param'. 
%

% Set number of degrees-of-freedom for the model:
vehicle_param.n_dofs = 2;

% Define additional system parameters:
vehicle_param.M1 = 290;
vehicle_param.M2 = 59;
vehicle_param.K1 = 16182;
vehicle_param.K2 = 190000;

vehicle_param.C = 1000;