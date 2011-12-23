% function [mu_bar, sigma_bar] = predict_init(mu, sigma, u, R)
% This function performs the prediction step when there are no previous shared
% measurements between the robots.
% Inputs:
%           robot(t):         robot structure   
%           Q:                3X3
% Outputs:   
%           robot(t):         robot structure
function robot = predict_init(robot, Q)

g = robot.mu + robot.u;   % Linearization
G = [
        1   0   -robot.u(2);
        0   1    robot.u(1);
        0   0       1
    ];                    % Jacobian of g

% Prediction step
robot.mu_bar    = g;
robot.sigma_bar = G*robot.sigma*G' + Q;

end
