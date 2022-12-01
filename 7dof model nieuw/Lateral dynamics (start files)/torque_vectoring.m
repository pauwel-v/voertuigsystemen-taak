function [T_l, T_r] = torque_vectoring(ackermanyawrate, delta, dphi, available_torque)
% available_torque is all torque available on axle (not including possible
% braking torque)
factor = 0.7;
braking_T = -50;
tolerance = 0.01;

if (ackermanyawrate == dphi) % good turning
    T_l = available_torque/2;
    T_r = available_torque - T_l;
elseif (abs(ackermanyawrate) < abs(dphi)) % oversteer
%     disp('OVERSTEER')
    if (abs(ackermanyawrate - dphi) < tolerance)
        T_l = available_torque/2;
        T_r = available_torque - T_l;
    elseif (delta < 0 || sign(dphi) < 0) % vehicle turns too much right
       T_r = factor*available_torque;
       T_l = braking_T;
   elseif (delta > 0 || sign(dphi) > 0) % vehicle turns too much left
       T_l = factor*available_torque;
       T_r = braking_T;
   end
elseif (abs(ackermanyawrate) > abs(dphi)) % understeer
%     disp('UNDERSTEER')
    if (abs(ackermanyawrate - dphi) < tolerance)
        T_l = available_torque/2;
        T_r = available_torque - T_l;
    elseif (delta < 0 || sign(dphi) < 0) % vehicle turns not enough to the right
       T_l = factor*available_torque;
       T_r = braking_T;
   elseif (delta > 0 || sign(dphi) > 0) % vehicle turns not enough to the left
       T_r = factor*available_torque;
       T_l = braking_T;
   end
end