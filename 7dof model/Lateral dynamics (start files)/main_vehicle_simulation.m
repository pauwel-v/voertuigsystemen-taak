
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main Vehicle Simulation %
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% This is the main file for the simulation of a vehicle system. 
% In this file all parameters and inputs are defined, the integration
% algorithm is called and the results are post-processed. 

clc
close all
clear all

%% Define vehicle parameters:
vehicle_param = set_vehicle_param;


%% Define simulation inputs:
inputs = set_inputs(vehicle_param);


%% Perform time-integration of vehicle system over inputs:
[X, data, Nc] = time_integration(vehicle_param, inputs);
% inputs.torque_vectoring = 1;
% [X_tvec_static_s, data_tvec_static_s] = time_integration(vehicle_param, inputs);
% inputs.transient = 1;
% [X_tvec_transient_s, data_tvec_transient_s] = time_integration(vehicle_param, inputs);


%% Post-process results: 
% post_processing([X; X_tvec_static_s; X_tvec_transient_s], [data; data_tvec_static_s; data_tvec_transient_s], vehicle_param, inputs);
data = [zeros(size(data,1),1) data];
%% write results of Torque-Vectoring to EXCEL file
% writematrix([inputs.time; [0, data_tvec_static_s(22,:)]]', 'torqueL.xlsx');
% writematrix([inputs.time; [0, data_tvec_static_s(23,:)]]', 'torqueR.xlsx');


figure
plot(inputs.time, Nc);

figure
yyaxis left
plot(inputs.time, data(24,:));
hold on
plot(inputs.time, X(vehicle_param.n_dofs + 3,:));
% plot(inputs.time, inputs.delta);
yyaxis right
plot(inputs.time, data(25,:));
ylabel('V_x [m/s]')
xlabel('t [s]')
legend('Ackerman yaw rate [rad/s]', 'Effective yaw rate [rad/s]')%, 'steering angle delta [rad]')
sgtitle('Comparison Ackerman yaw rate and effective yaw rate')