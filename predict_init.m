% function [mu_bar, sigma_bar] = predict_init(mu, sigma, u, R)
% This function performs the prediction step when there is no previous shared
% measurements between the robots.
% Inputs:
%           mu(t-1)           3X1   
%           sigma(t-1)        3X3
%           u(t)              3X1
%           R                 3X3
% Outputs:   
%           mu_bar(t)         3X1
%           sigma_bar(t)      3X3
function [mu_bar, sigma_bar] = predict_init(mu, sigma, u, R)

g = mu+u;   % Linearization
G = [
        1   0   -u(2);
        0   1    u(1);
        0   0       1
    ];      % Jacobian of g

% Prediction step
mu_bar = g;
sigma_bar = G*sigma*G' + R;

end
