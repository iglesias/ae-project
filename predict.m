% function robot = predict(robot, Q)
% This function performs the prediction step after the first update with a
% measurement between the robots has taken place.
%
% Inputs:
%           robot(t):         robot structure   
%           Q:                3X3
%                              
% Outputs:   
%           robot(t):         robot structure
%                              
function robot = predict(robot, Q, nrobots)

% Useful constants
i = robot.index;

g = robot.mu + robot.u;   % Linearization
G = [
        1   0   -robot.u(2);
        0   1    robot.u(1);
        0   0    1
    ];                    % Jacobian of g

% Prediction step
robot.mu_bar              = g;
sigma                     = robot.sigma(:, :, i);   % Alias
sigma_bar                 = G*sigma*G' + Q;         % Alias
robot.sigma_bar(:, :, i)  = sigma_bar;

% Compute or refresh the temporal terms required to predict the cross
% correlation terms in the next update

% In the robot structure, last_update indicates whether this is the first
% prediction after an update step or not

if robot.last_update

  for r = 1:nrobots
    
    if i ~= r
      
      if i < r
        [U, W, ~] = svd( robot.sigma(:, :, r) );
        robot.P_ita(:, :, r) = U*W;  
        % Alternative way to do this (the elseif must correspond)
        % robot.P_ita(:, :, r) = robot.sigma(:, :, r);
      elseif i > r
        [~, ~, V] = svd( robot.sigma(:, :, r)' );
        robot.P_ita(:, :, r) = V;
        % Alternative way to do this (the if must correspond)
        % robot.P_ita(:, :, r) = eye(3);
      end
      
    end
    
  end

  robot.last_update = false;

else
  
  for r = 1:nrobots
    
    if i ~= r
      robot.P_ita(:, :, r) = G*robot.P_ita(:, :, r);
    end
    
  end
  
end

end
