function    [dX, data, Nc] = equations_of_motion(vehicle_param, inputs, X, dX_old, k, Nc_old)
%
% This file solves the equations of motion with respect to the acceleration
% in order to determine dX. Other interesting variables can be stored in
% 'data'. 
%
%   dX = [velocity;
%         acceleration]
%          
% X = [x, y, phi, phi_fr, phi_fl, phi_rr, phi_rl, x_dot, y_dot, phi_dot, omega_fr, omega_fl, omega_rr, omega_rl]

% for transient slip
% X = [x, y, phi, phi_fr, phi_fl, phi_rr, phi_rl, x_dot, y_dot, phi_dot, omega_fr, omega_fl, omega_rr, omega_rl, kappa_fr, alpha_fr, kappa_fl, alpha_fl, kappa_rr, alpha_rr, kappa_rl, alpha_rl]

dX = zeros(vehicle_param.n_dofs*2,1);

% Velocity equals the velocity from the previous time-step:
dX(1:vehicle_param.n_dofs) = X(vehicle_param.n_dofs+1:vehicle_param.n_dofs*2);

%% Perform projections on velocities:
phi = X(3);
dphidt = X(vehicle_param.n_dofs+3);
steer_angle = inputs.delta(k);
tf = vehicle_param.tf;
tr = vehicle_param.tr;

r_w = vehicle_param.r_w;
alpha = 0;

M = vehicle_param.M;
a = vehicle_param.a;
b = vehicle_param.b;
L = vehicle_param.L;
h = vehicle_param.h;
h_aero = vehicle_param.h_aero;
Cx_f = vehicle_param.Cx_f;
Cy_f = vehicle_param.Cy_f;
Cx_r = vehicle_param.Cx_r;
Cy_r = vehicle_param.Cy_r;
mu = vehicle_param.mu;
T = inputs.T;
I_w = vehicle_param.I_w;

R1 = @(phi) [cos(phi) sin(phi); -sin(phi) cos(phi)];

% velocity in vehicle frame
v_accent = R1(phi)*[dX(1); dX(2)];

F_aero = v_accent(1)^2*vehicle_param.rho_air*vehicle_param.Cv*vehicle_param.A_front/2;

%% Calculate wheel velocities:
[v_long_fl, v_lat_fl, beta_fl] = get_wheel_velocities(v_accent(1), v_accent(2), dphidt, steer_angle, a,0.5*tf);
[v_long_rl, v_lat_rl, beta_rl] = get_wheel_velocities(v_accent(1), v_accent(2), dphidt, 0, -b, 0.5*tr);

[v_long_fr, v_lat_fr, beta_fr] = get_wheel_velocities(v_accent(1), v_accent(2), dphidt, steer_angle, a,-0.5*tf);
[v_long_rr, v_lat_rr, beta_rr] = get_wheel_velocities(v_accent(1), v_accent(2), dphidt, 0, -b, -0.5*tr);

%% Calculate longitudinal slip:
if inputs.transient == 0
    [s_fr, sign_Fx_fr] = calc_slip(v_long_fr, dX(4)*r_w);
    [s_fl, sign_Fx_fl] = calc_slip(v_long_fl, dX(5)*r_w);
    [s_rr, sign_Fx_rr] = calc_slip(v_long_rr, dX(6)*r_w);
    [s_rl, sign_Fx_rl] = calc_slip(v_long_rl, dX(7)*r_w);
else
    s_fr = X(15);
    beta_fr = X(16);
    s_fl = X(17);
    beta_fl = X(18);
    s_rr = X(19);
    beta_rr = X(20);
    s_rl = X(21);
    beta_rl = X(22);
end
%% Calculate normal forces:

% accx_old = dX_old(8);
% accy_old = dX_old(9);
acc_old = R1(phi)*[dX_old(8); dX_old(9)];

ackermanyawrate = (v_accent(1)*steer_angle)/L;


[F_fr_n, F_fl_n, F_rr_n, F_rl_n] = get_tyre_forces(M, acc_old(1), acc_old(2), alpha, b, L, tf, tr, h, h_aero, F_aero, T(k));

% start of control bs
diffCW = Nc_old;

% F_fl_n = F_fl_n - diffCW/2*M*9.81;
% F_fr_n = F_fr_n + diffCW/2*M*9.81;
% F_rl_n = F_rl_n + diffCW/2*M*9.81;
% F_rr_n = F_rr_n - diffCW/2*M*9.81;
factor_frontrear = 0.5;

if inputs.control_on
    F_fl_n = F_fl_n - diffCW*factor_frontrear*M*9.81;
    F_fr_n = F_fr_n + diffCW*factor_frontrear*M*9.81;
    F_rl_n = F_rl_n + diffCW*(1-factor_frontrear)*M*9.81;
    F_rr_n = F_rr_n - diffCW*(1-factor_frontrear)*M*9.81;
end

% end of control bs


%% Calculate tyre forces:
if inputs.transient == 0
    F_fl = tyre_model_Dugoff(F_fl_n, beta_fl, s_fl, mu, Cx_f, Cy_f, sign_Fx_fl);
    F_fr = tyre_model_Dugoff(F_fr_n, beta_fr, s_fr, mu, Cx_f, Cy_f, sign_Fx_fr);
    F_rl = tyre_model_Dugoff(F_rl_n, beta_rl, s_rl, mu, Cx_r, Cy_r, sign_Fx_rl);
    F_rr = tyre_model_Dugoff(F_rr_n, beta_rr, s_rr, mu, Cx_r, Cy_r, sign_Fx_rr);
else
    F_fl = tyre_model_Dugoff(F_fl_n, beta_fl, abs(s_fl), mu, Cx_f, Cy_f, 0);
    F_fr = tyre_model_Dugoff(F_fr_n, beta_fr, abs(s_fr), mu, Cx_f, Cy_f, 0);
    F_rl = tyre_model_Dugoff(F_rl_n, beta_rl, abs(s_rl), mu, Cx_r, Cy_r, 0);
    F_rr = tyre_model_Dugoff(F_rr_n, beta_rr, abs(s_rr), mu, Cx_r, Cy_r, 0);
    
    F_fr(1) = -1*sign(s_fr)*F_fr(1);
    F_fl(1) = -1*sign(s_fl)*F_fl(1); % different from example code on toledo because sign different in Dugoff line 29
    F_rr(1) = -1*sign(s_rr)*F_rr(1);
    F_rl(1) = -1*sign(s_rl)*F_rl(1);
end


%% Determine accelerations:
% acceleration in vehicle frame
acc_x_accent = 1/M*(F_rl(1) + F_rr(1) + (F_fl(1)+F_fr(1))*cos(steer_angle) - F_aero - (F_fl(2) + F_fr(2))*sin(steer_angle));
acc_y_accent = 1/M*(F_rl(2) + F_rr(2) +(F_fl(2) + F_fr(2))*cos(steer_angle) + (F_fl(1) + F_fr(2))*sin(steer_angle));

acc = R1(-phi)*[acc_x_accent;acc_y_accent];

% total moment around z axis
m_tot = cross([a tf/2 0],  [F_fl(1)*cos(steer_angle)-F_fl(2)*sin(steer_angle) F_fl(1)*sin(steer_angle)+F_fl(2)*cos(steer_angle) 0]) +...
        cross([a -tf/2 0], [F_fr(1)*cos(steer_angle)-F_fr(2)*sin(steer_angle) F_fr(1)*sin(steer_angle)+F_fr(2)*cos(steer_angle) 0]) +...
        cross([-b tr/2 0], [F_rl(1) F_rl(2) 0]) + ...
        cross([-b -tr/2 0],[F_rr(1) F_rr(2) 0]);

I_zz = vehicle_param.Izz;


if inputs.torque_vectoring == 1
    [T_l, T_r] = torque_vectoring(ackermanyawrate, steer_angle, dphidt, T(k));
else
    T_l = T(k)/2;
    T_r = T_l;
end

%% determine necessary counterweight
[Nc, B] = suboptimalnormalforceaugmentation(tf, tr, a, b, F_fl, F_fr, F_rl, F_rr, ackermanyawrate, dphidt, I_zz, Nc_old,M);

%% assign accelerations
dX(8) = acc(1);
dX(9) = acc(2);
dX(10) = m_tot(3)/I_zz;
dX(11) = (0*T_r - r_w*F_fr(1))/I_w;
dX(12) = (0*T_l - r_w*F_fl(1))/I_w;
dX(13) = (1*T_r - r_w*F_rr(1))/I_w;
dX(14) = (1*T_l - r_w*F_rl(1))/I_w;

% transient slip(s)
sigma_x = vehicle_param.sigma_x;
sigma_y = vehicle_param.sigma_y;

% front right
dX(15) = (-v_long_fr*X(15) + (dX(4)*r_w - v_long_fr))/sigma_x;
dX(16) = (-v_long_fr*X(16) + (v_lat_fr))/sigma_y;

% front left
dX(17) = (-v_long_fl*X(17) + (dX(5)*r_w - v_long_fl))/sigma_x;
dX(18) = (-v_long_fl*X(18) + (v_lat_fl))/sigma_y;

% rear right
dX(19) = (-v_long_rr*X(19) + (dX(6)*r_w - v_long_rr))/sigma_x;
dX(20) = (-v_long_rr*X(20) + (v_lat_rr))/sigma_y;

% rear left
dX(21) = (-v_long_rl*X(21) + (dX(7)*r_w - v_long_rl))/sigma_x;
dX(22) = (-v_long_rl*X(22) + (v_lat_rl))/sigma_y;


% % Acceleration is determined from the force-equilibrium of the system:
% % Additional interesting data can be stored in data:
data = [F_fl(1); F_fl(2); F_fl(3);... % 1-3
        F_fr(1); F_fr(2); F_fr(3);... % 4-6
        F_rl(1); F_rl(2); F_rl(3);... % 7-9
        F_rr(1); F_rr(2); F_rr(3);... % 10-12
        s_fl; s_fr; s_rl; s_rr; ...   % 13-16
%         sign_Fx_fl; sign_Fx_fr; sign_Fx_rl; sign_Fx_rr;...
        F_aero; ... % 17
        beta_fl; beta_fr; beta_rl; beta_rr;... % 18-21
        T_l; T_r; ackermanyawrate; v_accent(1); B]; % 22-26
% data = Nc;
    


