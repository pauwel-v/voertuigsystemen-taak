function [F_fr_n, F_fl_n, F_rr_n, F_rl_n] = get_tyre_forces(M, ax_old, ay_old, alpha, b, L, tf, tr, h, h_aero, F_aero, T)
% distribution of forces due gravity, longitudinal acceleration and
% aerodynamic load are spread evenly between left and right
g = 9.81;
G = M*g;

F_nfront = (-M*ax_old*h - h*G*sin(alpha) + b*G*cos(alpha) - h_aero*F_aero)/L;
F_nrear = G*cos(alpha) - F_nfront;

F_fl_n = F_nfront/2;
F_fr_n = F_nfront/2;

F_rl_n = F_nrear/2;
F_rr_n = F_nrear/2;


F_fl_n = F_fl_n - 0.5*M*ay_old*h*tf/2;
F_fr_n = F_fr_n + 0.5*M*ay_old*h*tf/2;

F_rl_n = F_rl_n - 0.5*M*ay_old*h*tr/2;
F_rr_n = F_rr_n + 0.5*M*ay_old*h*tr/2;

F = [F_fl_n, F_fr_n, F_rr_n, F_rl_n];

% for f = 1:length(F)
%    if F(f) < 0
%        F(f) = 100;
%    end
% end
% 
% F_fl_n = F(1);
% F_fr_n = F(2);
% F_rr_n = F(3);
% F_rl_n = F(4);

end