% function u = calculate_odometry(e_R, e_L, E_T, B, R_R, R_L, delta_t, mu)
% This function should calculate the odometry information
% Inputs:
%           e_L(t):         1X1
%           e_R(t):         1X1
%           E_T:            1X1
%           B:              1X1
%           R_L:            1X1
%           R_R:            1X1
%           delta_t:        1X1
%           mu(t-1):        3X1
% Outputs:
%           u(t):           3X1
function u = calculate_odometry(e_R, e_L, E_T, B, R_R, R_L, delta_t, mu)

if ~delta_t
    u = [0;0;0];
    return;
end

w_R = (2*pi*e_R) / (E_T*delta_t);   % Angular velocity of the right wheel
w_L = (2*pi*e_L) / (E_T*delta_t);   % Angular velocity of the left wheel

w = (w_R*R_R - w_L*R_L) / B;    % Angular velocity of the robot
v = (w_R*R_R + w_L*R_L) / 2;    % Translational velocity of the robot

u = [
        v*delta_t*cos( mu(3) ); % mu(3) is the estimated orientation
        v*delta_t*sin( mu(3) ); 
        w*delta_t
    ];

end
