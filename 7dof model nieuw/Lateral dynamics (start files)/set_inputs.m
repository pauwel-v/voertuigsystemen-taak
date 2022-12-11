function inputs = set_inputs(vehicle_param)
%
% This function is for defining the inputs for the time-simulation in a structure
% 'inputs'. This always has to include the time-vector and the time-step.  
%

% Obligatory time information:
% smaller dt makes animation much slower but forces are much more precise
inputs.dt = 0.001;
inputs.sim_time = 7;
inputs.time = (0:inputs.dt:inputs.sim_time);

inputs.animation = 0; % 1 = animate
inputs.save_video = 0;

% Other inputs are dependent on the model used. (eg. steering angle, engine
% torque, ...)

% Wheel Torque:
% inputs.T = [ones(1, floor(length(inputs.time)/2))*50 -50*ones(1,floor(length(inputs.time)/2))];
inputs.T = ones(1,length(inputs.time))*200;
%inputs.T = ones(1,length(inputs.time))*200;
inputs.torque_vectoring = 0; % boolean to activate torque vectoring

inputs.control_on = 0;

% use relaxation length
inputs.transient = 0;

% initial velocity:
start_vel = 20; % m/s
inputs.v_init = start_vel;


% Inclination of road:
inputs.inclin_angle = 0*pi/180; % [rad]

% Steering angle:
% steering_angle = 0; % degrees
% inputs.delta = ones(size(inputs.time))*pi/180*steering_angle;

% steer angle assignment
[delta, t] = GetInputSignal(inputs.dt, inputs.time);
inputs.delta = delta;