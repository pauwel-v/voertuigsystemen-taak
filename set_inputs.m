function inputs = set_inputs(vehicle_param,control)
%
% This function is for defining the inputs for the time-simulation in a structure
% 'inputs'. This always has to include the time-vector and the time-step.  
%

% Obligatory time information:
inputs.dt = 0.001;
inputs.time = (0:inputs.dt:10);

%% LQR controller
if control == 1
    K1 = vehicle_param.K1;
    K2 = vehicle_param.K2;
    M1 = vehicle_param.M1;
    M2 = vehicle_param.M2;
    C  = vehicle_param.C;
    
    A = [0 0 1 0;
        0 0 0 1;
        -K1/M1 K1/M1 -C/M1 C/M1;
        K1/M2 -(K1+K2)/M2 C/M2 -C/M2];
    B = [0; 0; -1/M1; 1/M2];
    
    Q = 3*10^3 *diag([1 1 1 1]);
    R = 0.001;
    [K,S,P] = lqr(A,B,Q,R);
    inputs.K = K;
else
    inputs.K = [0 0 0 0];
end



% Other inputs are dependent on the model used. (eg. steering angle, engine
% torque, ...)
% inputs.F = zeros(vehicle_param.n_dofs, length(inputs.time));
% 
% i1 = find(inputs.time==1);
% i2 = find(inputs.time==2);
% 
% inputs.F(i1:i2) = 1;