function w = smooth_input(t0, t1, inputs)
%
% amp = amplitude of input signal
% t0 = start of input signal
% t1 = point when inputs should be fully built
% time = time vector
%
% This function creates an input which rises with a sinusoidal shape from
% t0 to t1 and then remains constant with an amplitude amp.

%% if highly dynamic input (cos(8pi)) for a long period --> active suspension fails and performs less good compared to passive

dt = inputs.dt;
t= inputs.time;
w = zeros(1,length(t));

t1 = 0.5/dt + 1;
t2 = 0.75/dt + 1;

w(t1:t2) = 0.11*(1-cos(8*pi*[0.5:dt:0.75]))/2;


t3 = 3/dt + 1;
t4 = 3.25/dt + 1;

w(t3:t4) = 0.05*(1-cos(8*pi*[3:dt:3.25]))/2;

t5 = 7/dt + 1;
t6 = 8/dt + 1;

w(t5:t6) = 0.15*(1-cos(2*pi*[7:dt:8]))/2;




