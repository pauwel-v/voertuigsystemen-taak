function [Nc, B] = suboptimalnormalforceaugmentation(tf, tr, a, b, F_fl, F_fr, F_rl, F_rr, ackermanyawrate, dphi, I_zz, last_Nc,M)
% tuning parameters
lambda = 0.0001;
zeta = 0.5;

N_xfl = F_fl(1)/F_fl(3);
N_yfl = F_fl(2)/F_fl(3);
N_xfr = F_fr(1)/F_fr(3);
N_yfr = F_fr(2)/F_fr(3);
N_xrl = F_rl(1)/F_rl(3);
N_yrl = F_rl(2)/F_rl(3);
N_xrr = F_rr(1)/F_rr(3);
N_yrr = F_rr(2)/F_rr(3);

M_y = (ackermanyawrate)*I_zz;


% B = (tf/2*N_xfl - a*N_yfl) + (tf/2*N_xfr + a*N_yfr) ...
%     + (-tr/2*N_xrl - b*N_yrl) + (tr/2*N_xrr + b*N_yrr);
% 
% f = @(Nc) abs(B*Nc - M_y) + lambda*abs(Nc) + zeta*abs(Nc - last_Nc);
% options = optimset('MaxFunEvals', 2e10);
% [Nc, ~] = fminsearch(f, 0.1, options);


B = [(tf/2*N_xfl - a*N_yfl) + (tf/2*N_xfr + a*N_yfr) (-tr/2*N_xrl - b*N_yrl) + (-tr/2*N_xrr + b*N_yrr)];

f = @(Nc) norm(B*[Nc(1); Nc(2)] - M_y) + lambda*norm([Nc(1); Nc(2)]) + zeta*norm([Nc(1); Nc(2)] - last_Nc);

options = optimset('MaxFunEvals', 2e10);
[Nc, ~] = fminsearch(f, [0; 0], options);

B = norm(B);
Nc = (Nc(1)+Nc(2));


limitval = 0.1;

Nc = min(max(Nc,-limitval), limitval);
% Nc = M_y/B;
end