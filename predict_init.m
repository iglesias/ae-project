% function [mu_bar, sigma_bar] = predict_init(mu, sigma, u, R)
% This function performs the prediction step when there are no previous shared
% measurements between the robots.
% Inputs:
%           robot(t):         robot structure   
%           Q:                3x3
% Outputs:   
%           robot(t):         robot structure
function robot = predict_init(robot, Q)

% Useful constants
i = robot.index;

g = robot.mu + robot.u;   % Linearization
G = [
        1   0   -robot.u(2);
        0   1    robot.u(1);
        0   0       1
    ];                    % Jacobian of g

% Prediction step
robot.mu_bar              = g;
sigma                     = robot.sigma(:, :, i);   % Alias
sigma_bar                 = G*sigma*G' + Q;         % Alias
robot.sigma_bar(:, :, i)  = sigma_bar;

end
