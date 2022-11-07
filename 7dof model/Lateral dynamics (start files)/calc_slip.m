% function    [s, sign_Fx] = calc_slip(Vx, omega_R)
% %
% %   This function calculates the tyre slip which is necessary for
% %   determining the tyre-forces. This function has to be adjusted to the
% %   tyre-model used. 
% % 
% %   Vx = longitudinal vehicle velocity
% %   omega_R = omega*r_w
% %
% %   s = tyre slip (between 0 and 1)
% %   sign_Fx = sign for resulting force due to tyre slip (-1 or 1)
% 
% if abs(Vx) >= abs(omega_R) && Vx ~=0 % 
%     
%     s = min([abs((Vx-omega_R)/Vx),1]);
% 
%     sign_Fx = -1;
%     
% elseif abs(omega_R) > abs(Vx)
%     
%     s = min([abs((Vx-omega_R)/Vx),1]);
%     
%     sign_Fx = 1;
%     
% else % Vx = omega_R = 0
%     
%     s = 0;
%     
%     sign_Fx = 1;
%     
% end
function    [s ,sign_Fx] = calc_slip(Vx, omega_R)
%
%   This function calculates the tyre slip which is necessary for
%   determining the tyre-forces. This function has to be adjusted to the
%   tyre-model used. 
% 
%   Vx = longitudinal vehicle velocity
%   omega_R = omega*r_w
%
%   s = tyre slip (between 0 and 1)
%   sign_Fx = sign for resulting force due to tyre slip (-1 or 1)
if abs(Vx) >= abs(omega_R) && Vx ~=0 % 
    s = (abs(Vx)-abs(omega_R))/(abs(Vx));
    if Vx >= omega_R
        sign_Fx = -1;
    else 
        sign_Fx = 1; 
    end
elseif abs(Vx) < abs(omega_R)
    
    s = (abs(omega_R)-abs(Vx))/(abs(omega_R));
    
    if omega_R > Vx
        sign_Fx = 1;
    else 
        sign_Fx = -1; 
    end
else
    s = 0; 
    
    sign_Fx = 1;
end
end