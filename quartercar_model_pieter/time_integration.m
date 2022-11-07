function [X, dX, data] = time_integration(vehicle_param, inputs)
%
% [X, data] = time_integration(vehicle_param, inputs)
%
%   vehicle_param = parameters of simulated vehicle system
%   inputs = inputs for simulation
%
%   X = state variables (positions and velocities)
%   data = structure with interesting data for post-processing (eg.
%   slip-angles, forces)
%   
% This function performs the time integration of the vehicle system for the
% given inputs with a FORWARD EULER INTEGRATION. 

% Initialize state-space vector:
X = zeros(vehicle_param.n_dofs*2, length(inputs.time));
dX = zeros(vehicle_param.n_dofs*2, length(inputs.time));

% v_initial = 100; % km/h
% X(6) = v_initial * 1000/60;


% Create waitbar to track simulation progress:
wbar = waitbar(0,'Simulation running. Please wait...');

% Loop over time for integration:
dX_estim = zeros(vehicle_param.n_dofs*2,1);
for k = 1:length(inputs.time)-1
    
    % Calculate change of variables from equations of motion:
    [dX(:,k), data(:,k)] = equations_of_motion(vehicle_param, inputs, X(:,k), dX_estim, k);
    
    % Euler integration step:
    X(:,k+1) = X(:,k) + dX(:,k)*inputs.dt;
    dX_estim = dX(:,k);
    
    % Update wait-bar:
    waitbar(k/length(inputs.time), wbar)
    
end
% Close waitbar:
close(wbar)