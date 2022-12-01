function [delta, t] = GetInputSignal(dt,t)
%
% This function generates the steering angle (delta in radians) for the assignment. It
% also generates the time-vector (t in seconds), and the user can select the
% time-sampling-rate (dt in seconds) which is used.

% % t = (0:dt:t);
% 
% delta = zeros(1,length(t)); 
% % % 
% t1 = 1;
% [~,i1] = min(abs(t-t1));
% t2 = 1.5;
% [~,i2] = min(abs(t-t2));
% delta(i1:i2) = 10*pi/180;
% 
% 
% t3 = 3.5;
% [~,i3] = min(abs(t-t3));
% t4 = 4.5;
% [~,i4] = min(abs(t-t4));
% delta(i3:i4) = -10*pi/180;

% t1 = 3.5;
% delta = 10*pi/180.*ones(1,length(t));
% [~,i1] = min(abs(t-t1));
% delta(1:i1) = 0*pi/180;%.*ones(1,length(t));


t2 = 0:0.001: 3/4 * 1.4;
t1 = 3/4*1.4:0.001:1.4;
% delta = [sin(2*pi/1.4 * t2), -1*ones(1,length(0:0.001:1/4*1.4)-1), sin(2*pi/1.4 * t1)];
delta = [sin(2*pi/1.4 * t2), -1*ones(1,length(0:0.001:1.4)-1), sin(2*pi/1.4 * t1)];
delta = 10*pi/180.*[delta, zeros(1,length(t)- length(delta))];



