function    [dX, data] = equations_of_motion(vehicle_param, inputs, X, dX_old, k)

% Initialize dX:
dX = zeros(vehicle_param.n_dofs*2,1);


%% Velocity in dX equals the velocity from the previous time-step:
dX(1:vehicle_param.n_dofs) = X(vehicle_param.n_dofs+1:end);


%% params
K1 = vehicle_param.K1;
K2 = vehicle_param.K2;
M1 = vehicle_param.M1;
M2 = vehicle_param.M2;
C  = vehicle_param.C;


%% controller
K = inputs.K;
U = -K*X;

%% road-input

w = smooth_input(0.5,0.75, inputs);

dX(vehicle_param.n_dofs+1) = (-C*(dX(1)-dX(2))-K1*(X(1)-X(2)) - U)*1/M1;
dX(vehicle_param.n_dofs+2) = (C*(dX(1)-dX(2)) + K1*(X(1)-X(2)) + K2*(w(k)-X(2)) + U)*1/M2;

% Additional interesting data can be stored in data:
data = [0;0];


