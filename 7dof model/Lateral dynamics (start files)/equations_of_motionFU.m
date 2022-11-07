function    [dX, data] = equations_of_motion(vehicle_param, inputs, X, dX_old, k)
%
% This file solves the equations of motion with respect to the acceleration
% in order to determine dX. Other interesting variables can be stored in
% 'data'. 
%
%   dX = [velocity;
%         acceleration]
%          
% X = [x, y, phi, phi_fr, phi_fl, phi_rr, phi_rl, x_dot, y_dot, phi_dot, omega_fr, omega_fl, omega_rr, omega_rl, s_fr_long, s_fr_lat, s_fl_long, s_fl_lat, s_rr_long, s_rr_lat, s_rl_long, s_rl_lat]

% dX = [x_dot, y_dot, phi_dot, omega_fr, omega_fl, omega_rr, omega_rl,
%      s_fr_long_dot, s_fr_lat_dot, s_fl_long_dot, s_fl_lat_dot, s_rr_long_dot, s_rr_lat_dot, s_rl_long_dot, s_rl_lat_dot, x_dot_dot , ...]


dX = zeros(vehicle_param.n_dofs*2+8,1);

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

ndofs = vehicle_param.n_dofs;

R1 = @(alpha) [cos(alpha) sin(alpha); -sin(alpha) cos(alpha)];

% velocity in vehicle frame
v_accent = R1(phi)*[dX(1); dX(2)];

F_aero = v_accent(1)^2*vehicle_param.rho_air*vehicle_param.Cv*vehicle_param.A_front/2;

%% Calculate wheel velocities:
[v_long_fl, v_lat_fl, beta_fl] = get_wheel_velocities(v_accent(1), v_accent(2), dphidt, steer_angle, a,0.5*tf);
[v_long_rl, v_lat_rl, beta_rl] = get_wheel_velocities(v_accent(1), v_accent(2), dphidt, 0, -b, 0.5*tr);

[v_long_fr, v_lat_fr, beta_fr] = get_wheel_velocities(v_accent(1), v_accent(2), dphidt, steer_angle, a,-0.5*tf);
[v_long_rr, v_lat_rr, beta_rr] = get_wheel_velocities(v_accent(1), v_accent(2), dphidt, 0, -b, -0.5*tr);
%%
Vsx_fr = vehicle_param.r_w*X(vehicle_param.n_dofs+4) - v_long_fr;
Vsx_fl = vehicle_param.r_w*X(vehicle_param.n_dofs+5) - v_long_fl;
Vsx_rr = vehicle_param.r_w*X(vehicle_param.n_dofs+6) - v_long_rr;
Vsx_rl = vehicle_param.r_w*X(vehicle_param.n_dofs+7) - v_long_rl;

Vsy_fr = v_lat_fr + vehicle_param.a*phi; 
Vsy_fl = v_lat_fl + vehicle_param.a*phi; 
Vsy_rr = v_lat_rr - vehicle_param.b*phi;
Vsy_rl = v_lat_rl - vehicle_param.b*phi;

%% Calculate slip:
if inputs.transient == 0
    [s_fl_long, sign_Fx_fl] = calc_slip(v_long_fl, dX(5)*r_w);
    [s_fr_long, sign_Fx_fr] = calc_slip(v_long_fr, dX(4)*r_w);
    [s_rl_long, sign_Fx_rl] = calc_slip(v_long_rl, dX(7)*r_w);
    [s_rr_long, sign_Fx_rr] = calc_slip(v_long_rr, dX(6)*r_w);
else
    kappa_fr = X(2*vehicle_param.n_dofs+1);
    alpha_fr = X(2*vehicle_param.n_dofs+2);
    kappa_fl = X(2*vehicle_param.n_dofs+3);
    alpha_fl = X(2*vehicle_param.n_dofs+4);
    
    kappa_rr = X(2*vehicle_param.n_dofs+5);
    alpha_rr = X(2*vehicle_param.n_dofs+6);
    kappa_rl = X(2*vehicle_param.n_dofs+7);
    alpha_rl = X(2*vehicle_param.n_dofs+8);    
    
    sign_Fx_fr = 0;
    sign_Fx_fl = 0;
    sign_Fx_rr = 0;
    sign_Fx_rl = 0;
    
    beta_fl = alpha_fl;
    beta_rl = alpha_rl;
    beta_fr = alpha_fr;
    beta_rr = alpha_rr;
    
    s_fl_long = kappa_fl;
    s_fr_long = kappa_fr;
    s_rr_long = kappa_rr;
    s_rl_long = kappa_rl;
end
%% Calculate normal forces:

% accx_old = dX_old(8);
% accy_old = dX_old(9);
acc_old = R1(phi)*[dX_old(ndofs + 1); dX_old(ndofs+2)];

[F_fr_n, F_fl_n, F_rr_n, F_rl_n] = get_tyre_forces(M, acc_old(1), acc_old(2), alpha, b, L, tf, tr, h, h_aero, F_aero, T(k));
%% Calculate tyre forces:
F_fl = tyre_model_Dugoff(F_fl_n, beta_fl, s_fl_long, mu, Cx_f, Cy_f, sign_Fx_fl);
F_fr = tyre_model_Dugoff(F_fr_n, beta_fr, s_fr_long, mu, Cx_f, Cy_f, sign_Fx_fr);
F_rl = tyre_model_Dugoff(F_rl_n, beta_rl, s_rl_long, mu, Cx_r, Cy_r, sign_Fx_rl);
F_rr = tyre_model_Dugoff(F_rr_n, beta_rr, s_rr_long, mu, Cx_r, Cy_r, sign_Fx_rr);


if inputs.transient == 1
   F_fr(1) = F_fr(1) * (-1)* sign(kappa_fr);
   F_fl(1) = F_fl(1) *(-1)* sign(kappa_fl);
   F_rr(1) = F_rr(1) *(-1)* sign(kappa_rr);
   F_rl(1) = F_rl(1) *(-1)* sign(kappa_rl);    
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


ackermanyawrate = (v_accent(1)*steer_angle)/L;
if inputs.torque_vectoring == 1
    [T_l, T_r] = torque_vectoring(ackermanyawrate, steer_angle, dphidt, T(k));
else
    T_l = T(k)/2;
    T_r = T_l;
end
dX(ndofs+1) = acc(1);
dX(ndofs+2) = acc(2);
dX(ndofs+3) = m_tot(3)/I_zz;
dX(ndofs+4) = (0*T_r - r_w*F_fr(1))/I_w;
dX(ndofs+5) = (0*T_l - r_w*F_fl(1))/I_w;
dX(ndofs+6) = (1*T_r - r_w*F_rr(1))/I_w;
dX(ndofs+7) = (1*T_l - r_w*F_rl(1))/I_w;
% Acceleration is determined from the force-equilibrium of the system:
% Additional interesting data can be stored in data:
data = [F_fl(1); F_fl(2); F_fl(3);...
        F_fr(1); F_fr(2); F_fr(3);...
        F_rl(1); F_rl(2); F_rl(3);...
        F_rr(1); F_rr(2); F_rr(3);...
        s_fl_long; s_fr_long; s_rl_long; s_rr_long; ...
%         sign_Fx_fl; sign_Fx_fr; sign_Fx_rl; sign_Fx_rr;...
        F_aero; ...
        beta_fl; beta_fr; beta_rl; beta_rr;...
        T_l; T_r; ackermanyawrate];

% if inputs.transient == 0
%     dX(ndofs+8:end) = 0;
% else
%         
% end
if inputs.transient == 1
    dX(vehicle_param.n_dofs+8)  = (-v_long_fr*X(vehicle_param.n_dofs+8)  + Vsx_fr)/vehicle_param.sigma_x; % Tyre relaxation longitudinal - front right
    dX(vehicle_param.n_dofs+9)  = (-v_long_fr*X(vehicle_param.n_dofs+9)  + Vsy_fr)/vehicle_param.sigma_y; % Tyre relaxation lateral - front right
    dX(vehicle_param.n_dofs+10) = (-v_long_fl*X(vehicle_param.n_dofs+10) + Vsx_fl)/vehicle_param.sigma_x; % Tyre relaxation longitudinal - front left
    dX(vehicle_param.n_dofs+11) = (-v_long_fl*X(vehicle_param.n_dofs+11) + Vsy_fl)/vehicle_param.sigma_y; % Tyre relaxation lateral - front left

    dX(vehicle_param.n_dofs+12) = (-v_long_rr*X(vehicle_param.n_dofs+12) + Vsx_rr)/vehicle_param.sigma_x; % Tyre relaxation longitudinal - rear right
    dX(vehicle_param.n_dofs+13) = (-v_long_rr*X(vehicle_param.n_dofs+13) + Vsy_rr)/vehicle_param.sigma_y; % Tyre relaxation lateral - rear right
    dX(vehicle_param.n_dofs+14) = (-v_long_rl*X(vehicle_param.n_dofs+14) + Vsx_rl)/vehicle_param.sigma_x; % Tyre relaxation longitudinal - rear left
    dX(vehicle_param.n_dofs+15) = (-v_long_rl*X(vehicle_param.n_dofs+15) + Vsy_rl)/vehicle_param.sigma_y; % Tyre relaxation lateral - rear left
end
