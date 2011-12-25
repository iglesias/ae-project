% function robot = init_robot()
% Initializes a robot structure
%
% Outputs:
%           robot:    robot structure
%
% Robot attributes:
%           P_ita:    each robot needs to track its prediction steps to later
%                     predict the cross correlation terms
%
function robot = init_robot()

robot.mu          = zeros(3, 1);            % Initial estimate of state
robot.sigma       = 1e-10*diag( [1 1 1] );  % Initial covariance matrix
robot.lastUpdate  = false;             
robot.P_ita       = zeros(3, 3);

end
