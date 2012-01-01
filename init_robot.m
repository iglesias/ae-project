% function robot = init_robot()
% Initializes a robot structure
%
% Inputs:
%           index           identifier for the robot (must be unique)
%           nrobots         number of robots in the simulation
%
% Outputs:
%           robot           robot structure
%         
% Robot attributes:
%           P_ita           each robot needs to track its prediction steps 
%                           to later predict the cross correlation terms
%           never_updated   this refers just to CL updates
%
function robot = init_robot(index, nrobots)

robot.index = index;
robot.u     = zeros(3, 1);

robot.mu     = zeros(3, 1);   % Initial estimate of state
robot.mu_bar = zeros(3, 1);

robot.sigma              = zeros(3, 3, nrobots);
robot.sigma(:, :, index) = 1e-10*diag( [1 1 1] );  % Initial cov. matrix
robot.sigma_bar          = zeros(3, 3, nrobots);
robot.sigma_bar(:, :, index) = 1e-10*diag( [1 1 1] );  % Initial cov. matrix

robot.last_update   = false;
robot.never_updated = true;
robot.P_ita         = zeros(3, 3, nrobots);

end
