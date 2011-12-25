% function robot = batch_update(robot, H_bar, Q_bar, nu_bar)
% This function performs the update process.
% You need to make sure that the output sigma_bar is symmetric.
% The last line makes sure that ouput sigma_bar is always symmetric.
%
% Inputs:
%           robot           robot structure
%           sigma_bar(t)    3X3
%           H_bar(t)        2nX3
%           Q_bar(t)		2nX2n
%           nu_bar(t)       2nX1
%
% Outputs:
%           mu(t)           3X1
%           sigma(t)        3X3
%
function robot = batch_update(robot, H_bar, Q_bar, nu_bar)

% Useful constants
dim = size(H_bar, 2);

% Kalman Gain
K = robot.sigma_bar*H_bar'/(H_bar*robot.sigma_bar*H_bar' + Q_bar);

% Update the belief
robot.mu    = robot.mu_bar + K*nu_bar;        
% Update the uncertainty
robot.sigma = (eye(dim) - K*H_bar)*robotsigma_bar;
% Assume symmetry
robot.sigma = (robot.sigma + robot.sigma')/2;                   d

end