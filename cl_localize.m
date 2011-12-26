% function [ output_args ] = cl_localize( input_args )
%
%
% Inputs:
%           robot(t)        robot structure
%           Q               model noise's covariance
%           observed(t)     robot structure
%           Rrobo           covariance in the noise of the measurements 
%                           between robots
%           z               observation from one robot to another
%
function robot = cl_localize(robot, Q, observed, Rrobo, z)

if robot.never_updated

  robot = predict_init(robot, Q);
 
  % TODO Generalize this so it works for observed robots running CL
  if nargin > 2
    robot = update_init_observer(observed, robot, Rrobo, z);
  end
  
else

  robot = predict(robot, Q);
  
  % TODO Generalize this so it works for observed robots running CL
  if nargin > 2
    robot = update_observer(observed, robot, Rrobo, z);
  end
  
end

end

