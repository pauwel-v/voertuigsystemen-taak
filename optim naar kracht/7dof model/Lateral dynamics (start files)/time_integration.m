function [X, data] = time_integration(vehicle_param, inputs)
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
X = zeros(vehicle_param.n_dofs*2 + 8, length(inputs.time));
dX = zeros(vehicle_param.n_dofs*2+ 8, length(inputs.time));

X(vehicle_param.n_dofs+1,1) = inputs.v_init; % xdot
X(vehicle_param.n_dofs+4,1) = inputs.v_init/vehicle_param.r_w;
X(vehicle_param.n_dofs+5,1) = inputs.v_init/vehicle_param.r_w;
X(vehicle_param.n_dofs+6,1) = inputs.v_init/vehicle_param.r_w;
X(vehicle_param.n_dofs+7,1) = inputs.v_init/vehicle_param.r_w;
try
    % Create waitbar to track simulation progress:
    wbar = waitbar(0,'Simulation running. Please wait...');
    Nc = zeros(2, length(inputs.time));
    % Loop over time for integration:
    for k = 1:length(inputs.time)-1
% 
%         % Calculate change of variables from equations of motion:
%         if k == 1
%             [dX(:,k+1), data(:,k)] = equations_of_motion(vehicle_param, inputs, X(:,k), dX(:,k), k, 0);
%         else
%             [dX(:,k+1), data(:,k)] = equations_of_motion(vehicle_param, inputs, X(:,k), dX(:,k), k, data(:,k-1));
%         end

        %Nc = zeros(2, length(inputs.time));
        [dX(:,k+1), data(:,k), Nc(:,k+1)] = equations_of_motion(vehicle_param, inputs, X(:,k), dX(:,k), k, Nc(:,k));


        % Euler integration step:
        X(:,k+1) = X(:,k) + dX(:,k+1)*inputs.dt;

        % Update wait-bar:
        waitbar(k/length(inputs.time), wbar)

    end
catch ERROR
    delete(wbar)
    rethrow(ERROR)
end
    
% Close waitbar:
close(wbar)