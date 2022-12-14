
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
[X, data] = time_integration(vehicle_param, inputs);
data = [zeros(size(data,1),1) data];

inputs.control_on = 1;
%inputs.v_init = 10;
%inputs.T = ones(1,length(inputs.time))*200;
[X_control, data_control] = time_integration(vehicle_param, inputs);
data_control = [zeros(size(data_control,1),1) data_control];
% inputs.torque_vectoring = 1;
% [X_tvec_static_s, data_tvec_static_s] = time_integration(vehicle_param, inputs);
% inputs.transient = 1;
% [X_tvec_transient_s, data_tvec_transient_s] = time_integration(vehicle_param, inputs);


%% Post-process results:
close all;
post_processing([X; X_control], [data; data_control], vehicle_param, inputs);
% post_processing([X_control], [data_control], vehicle_param, inputs);