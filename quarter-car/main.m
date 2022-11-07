clc
clear all
close all

vehicle_param = set_vehicle_param;
inputs = set_inputs(vehicle_param,0);
[X1, data] = time_integration(vehicle_param, inputs);
%post_processing(X1, data,vehicle_param,inputs);

vehicle_param = set_vehicle_param;
inputs = set_inputs(vehicle_param,1);
% inputsnoC = set_inputs(vehicle_param, 0);

[X2, data] = time_integration(vehicle_param, inputs);
% [XnoC, datanoC] = time_integration(vehicle_param, inputsnoC);
w = smooth_input(0.5,0.75, inputs);
%post_processing(X2, data,vehicle_param,inputs);
figure
plot(inputs.time, X1(1,:) - X1(2,:))
hold on
plot(inputs.time, X2(1,:) - X2(2,:))
%plot(inputs.time, w)
legend('Passive','Active',' Input')