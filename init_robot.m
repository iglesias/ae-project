% function robot = init_robot()
% Initializes a robot structure
%
% Outputs:
%           robot         robot structure
%
% Robot attributes:
%           P_ita           each robot needs to track its prediction steps 
%                           to later predict the cross correlation terms
%           never_updated   this refers just to CL updates
%
function robot = init_robot()

robot.mu            = zeros(3, 1);            % Initial estimate of state
robot.sigma         = 1e-10*diag( [1 1 1] );  % Initial covariance matrix
robot.last_update   = false;
robot.never_updated = true;
robot.P_ita         = zeros(3, 3);

end
