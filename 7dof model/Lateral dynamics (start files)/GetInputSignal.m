function [delta, t] = GetInputSignal(dt,t)
%
% This function generates the steering angle (delta in radians) for the assignment. It
% also generates the time-vector (t in seconds), and the user can select the
% time-sampling-rate (dt in seconds) which is used.

% t = (0:dt:t);

delta = zeros(1,length(t)); 

t1 = 0.5;
[~,i1] = min(abs(t-t1));
t2 = 2.5;
[~,i2] = min(abs(t-t2));
delta(i1:i2) = 10*pi/180;


t1 = 3.5;
[~,i1] = min(abs(t-t1));
t2 = 4.5;
[~,i2] = min(abs(t-t2));
delta(i1:i2) = -10*pi/180;


