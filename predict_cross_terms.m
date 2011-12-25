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
function [robot1, robot2] = predict_cross_terms(robot1, robot2)

% TODO generalize this for the case when there are more than two robots. Should
% it affect?

robot1.cross_bar = robot1.P_ita * robot2.P_ita';

robot2.cross_bar = robot2.P_ita * robot1.P_ita';

end
