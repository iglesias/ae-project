% function H = jacobian_observation_model(mu_bar, M, j, z, i)
% This function is the implementation of the H function
%
% Inputs:
%           mu_bar(t)   3X1
%           M           2XN
%           j           1X1
%           z           2Xn
%           i           1X1
%
% Outputs:  
%           H           2X3
%
function H = jacobian_observation_model(mu_bar, M, j, z, i)

H = [
        ( mu_bar(1)-M(1,j) ) / z(1,i)   ( mu_bar(2)-M(2,j) ) / z(1,i)    0;
       -( mu_bar(2)-M(2,j) ) / z(1,i)^2 ( mu_bar(1)-M(1,j) ) / z(1,i)^2 -1
    ];

end