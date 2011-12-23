% function robot = predict(robot, Q, lastUpdate)
% This function performs the prediction step after the first update with a
% measurement between the robots has taken place.
% Inputs:
%           robot(t):         robot structure   
%           Q:                3X3
%           lastUpdate(t):    boolean, indicates whether this is the first prediction
%                             after an update step or not
% Outputs:   
%           robot(t):         robot structure
function robot = predict(robot, Q, lastUpdate)

g = robot.mu + robot.u;   % Linearization
G = [
        1   0   -robot.u(2);
        0   1    robot.u(1);
        0   0    1
    ];                    % Jacobian of g

% Prediction step
robot.mu_bar    = g;
robot.sigma_bar = G*robot.sigma*G' + Q;

% Compute or refresh the temporal terms required to predict the cross
% correlation terms in the next update

if lastUpdate
  
  % TODO generalize this for the case when there are more than two robots
  switch robot.index
    case 1
      [U, W, ~] = svd( robot.cross );
      robot.P_ita = U*W;
    case 2
      [~, ~, V] = svd( robot.cross' );
      robot.P_ita = V;    
    otherwise
      disp('Problem with the robot index!')
  end

else
  robot.P_ita = G*robot.P_ita;
end

end
