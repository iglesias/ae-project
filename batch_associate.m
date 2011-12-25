% function [c,outlier, nu_bar, H_bar] = batch_associate(robot, z, M, ...
%                                                       lambda_m, Q)
% This function performs the maximum likelihood association and outlier 
% detection.
% Note that the bearing error lies in the interval [-pi,pi)
%
% Inputs:
%           robot               robot structure
%           R                   3x3
%           Q                   2x2
%           z(t)                2xn
%           M                   2xN
%           lambda_m            1x1
%
% Outputs: 
%           c(t)                1xn
%           outlier             1xn
%           nu_bar(t)           2nx1
%           H_bar(t)            2nx3
%
function [c,outlier, nu_bar, H_bar] = batch_associate(robot, z, M, ...
                                                      lambda_m, Q)

% Useful constants and memory pre-allocation
n       = size(z, 2);     % Number of observations
N       = size(M, 2);     % Number of landmarks
H_k     = zeros(2*N, 3);
nu_k    = zeros(2*N, 1);
D_k     = zeros(N, 1);
lhood_k = zeros(N, 1);
c       = zeros(1, n);
outlier = zeros(1, n);
nu_bar  = zeros(2*n, 1);
H_bar   = zeros(2*n, 3);

for i = 1:n
    
    for k = 1:N
        zhat_k = observation_model(robot.mu_bar, M, k);
        H_k(2*k-1:2*k, :) = ...
            jacobian_observation_model(robot.mu_bar, M, k, zhat_k, 1);
        S_k = H_k(2*k-1:2*k, :)*sigma_bar*H_k(2*k-1:2*k, :)' + Q;
        nu_k(2*k-1:2*k) = z(:, i) - zhat_k;
        
        % IMPORTANT: normalize the bearing error
        nu_k(2*k) = wrapToPi( nu_k(2*k) );
        
        D_k(k) = nu_k(2*k-1:2*k)'/(S_k)*nu_k(2*k-1:2*k);
        lhood_k(k) = det(2*pi*S_k)^(-1/2) * exp( -1/2*D_k(k) );
    end
    
    [~, c(i)]   = max( lhood_k );
    outlier(i)  = D_k( c(i) ) >= lambda_m;
    
    nu_bar(2*i-1:2*i)   = nu_k(2*c(i)-1:2*c(i));
    H_bar(2*i-1:2*i, :) =  H_k(2*c(i)-1:2*c(i), :);
    
end

end