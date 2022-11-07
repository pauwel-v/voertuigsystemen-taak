function w = smooth_input(t0, t1, inputs)
%
% amp = amplitude of input signal
% t0 = start of input signal
% t1 = point when inputs should be fully built
% time = time vector
%
% This function creates an input which rises with a sinusoidal shape from
% t0 to t1 and then remains constant with an amplitude amp.

a = 0.05;

dt = inputs.dt;
t= inputs.time;
w = zeros(1,length(t));

t1 = 0.5/dt + 1;
t2 = 0.75/dt + 1;

w(t1:t2) = a*(1-cos(8*pi*[0.5:dt:0.75]));
