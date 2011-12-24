% function robot = init_robot()
% Initializes a robot structure
% Outputs:
%           robot:    robot structure
function robot = init_robot()

robot.mu    = zeros(3, 1);            % Initial estimate of state
robot.sigma = 1e-10*diag( [1 1 1] );  % Initial covariance matrix

end
