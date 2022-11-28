clear all;
close all;
clc

dt = 0.001;
t = 10;
t = (0:dt:t)';

tsinus = (0:dt:1.4)';
sine = sin(2*pi*1/1.4*tsinus);

index = find()

starttime = 0.2;
dwelltime = 0.5; % s

sine_dwell = [sine(1:index) ones(dwelltime/dt, 1) sine(index:end)];

plot(sine_dwell)