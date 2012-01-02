% function [robot1, robot2] = predict_cross_terms(robot1, robot2)
% This function predicts the value of the cross correlation terms between two
% robots. This is not performed in the function predict because it must be done
% just before the update (when the two robots exchange data). Here it is not
% important which robot played the role of observer/observed but their indices
% are important.
% Inputs:
%           robot1(t):    robot structure
%           robot2(t):    robot structure
%
% Outputs:
%           robot1(t):    robot structure
%           robot2(t):    robot structure
%
function robots = predict_cross_terms(robots)

% Useful constants
nrobots = length(robots);

for ri = 1:nrobots
  for rj = 1:nrobots
   
    if ri ~= rj
    
      robots(ri).sigma_bar(:, :, rj) = robots(ri).P_ita(:, :, rj) * ...
                                       robots(rj).P_ita(:, :, ri)';
    
    end
    
  end
end


end
